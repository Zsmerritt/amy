// pcm.c

#include "amy.h"
#include "transfer.h"

#ifdef __EMSCRIPTEN__
#include "emscripten.h"
#endif

#ifdef AMY_DAISY
#define malloc_caps(a, b) qspi_malloc(a)
#define free(a) qspi_free(a)
#endif


// This is for any in-memory PCM samples.
typedef struct {
    uint8_t type; 
    char filename[MAX_FILENAME_LEN];
    uint8_t channels;
    uint32_t file_handle;
    uint32_t file_bytes_remaining;
    int16_t * sample_ram;
    uint32_t length;
    uint32_t loopstart;
    uint32_t loopend;
    uint8_t midinote;
    uint32_t samplerate;
    float log2sr;
} memorypcm_preset_t;

// linked list of memorypcm presets
typedef struct memorypcm_ll_t{
    memorypcm_preset_t *preset;
    struct memorypcm_ll_t *next;
    uint16_t preset_number;
} memorypcm_ll_t;

// FLASH FENCE: on platforms that serve PCM banks straight from memory-mapped
// flash (ESP32-S3 partitions), a flash program/erase suspends the cache those
// reads go through while the render task keeps running (code and rodata live
// in PSRAM on a separate bus) -- one sample fetch during the write window
// hard-crashes the chip (dual-core interrupt WDT). The platform sets
// [lo, hi) to its flash-mapped address window and raises amy_flash_fence
// around every filesystem write; fenced renders skip the fetch and emit
// silence for those oscs (phase held, so the voice resumes afterwards).
// Everything not in the window -- computed voices, PSRAM-loaded banks --
// renders on undisturbed.
volatile uint8_t amy_flash_fence = 0;
const void *amy_flash_fence_lo = 0;
const void *amy_flash_fence_hi = 0;


memorypcm_ll_t * memorypcm_ll_start;

#define PCM_AMY_LOG2_SAMPLE_RATE log2f(PCM_AMY_SAMPLE_RATE / ZERO_LOGFREQ_IN_HZ)

// Compile-time size check that works on any C/C++ standard (negative array
// size on failure) -- keeps amy.h's published blob sizes in lockstep with
// the baked map headers without pulling those headers into platform code.
#define AMY_PCM_SIZE_CHECK(name, cond) typedef char name[(cond) ? 1 : -1]

#ifdef GAMMA9001
#include "pcm_gamma9001.h"
AMY_PCM_SIZE_CHECK(amy_gamma9001_bytes_check,
                   AMY_GAMMA9001_PCM_BYTES == 2u * GAMMA9001_BIN_FRAMES);
// Set by the platform at boot: web links the drums.bin blob in and passes it,
// ESP32-S3 passes the esp_partition_mmap'd partition. NULL = banks unavailable.
const int16_t * gamma9001_pcm = NULL;
void amy_set_gamma9001_pcm(const int16_t * data) {
    gamma9001_pcm = data;
}
#endif

#ifdef GM_FONTS
#include "pcm_gm.h"
#include "pcm_gm_big.h"
AMY_PCM_SIZE_CHECK(amy_gm_bytes_check, AMY_GM_PCM_BYTES == 2u * GM_BIN_FRAMES);
AMY_PCM_SIZE_CHECK(amy_gm_big_bytes_check,
                   AMY_GM_BIG_PCM_BYTES == 2u * GM_BIG_BIN_FRAMES);
AMY_PCM_SIZE_CHECK(amy_gm_big_emu4_window_check,
                   AMY_GM_BIG_EMU4_FIRST_SAMPLE < AMY_GM_BIG_EMU4_END_SAMPLE &&
                   AMY_GM_BIG_EMU4_END_SAMPLE <= GM_BIG_BIN_FRAMES);
// amy_set_gm_big_pcm_window's map sanity check reads the entries for presets
// 1903 and 2060 (the emu4 window edges) -- keep them inside the map.
AMY_PCM_SIZE_CHECK(amy_gm_big_emu4_edges_check,
                   1903 >= GM_BIG_PRESET_BASE &&
                   2060 < GM_BIG_PRESET_BASE + GM_BIG_NUM_SAMPLES);
// Set by the platform at boot from separate esp_partition_mmap ranges of
// the `fonts` partition (one 12.5MB map didn't fit the S3's remaining
// contiguous data vaddr): gm_pcm = GeneralUser bank (partition offset 0),
// gm_big_pcm = big multi-font bank (0x4B0000). NULL = that bank unavailable.
const int16_t * gm_pcm = NULL;
void amy_set_gm_pcm(const int16_t * data) {
    gm_pcm = data;
}
// The big bank may be only partially resident: gm_big_pcm points at blob
// sample gm_big_first_sample, and gm_big_num_samples samples follow it.
// Presets outside that window return NULL (silent) from
// get_preset_for_preset_number -- their samples have no valid address.
// Default = whole blob (web/hosted builds that link or map all of it).
const int16_t * gm_big_pcm = NULL;
static uint32_t gm_big_first_sample = 0;
static uint32_t gm_big_num_samples = GM_BIG_BIN_FRAMES;
void amy_set_gm_big_pcm(const int16_t * data) {
    gm_big_pcm = data;
    gm_big_first_sample = 0;
    gm_big_num_samples = GM_BIG_BIN_FRAMES;
}
void amy_set_gm_big_pcm_window(const int16_t * data,
                               uint32_t first_sample, uint32_t num_samples) {
    // Refuse a stale window: if the published emu4 constants (amy.h) no
    // longer match the baked map -- e.g. the bank was rebaked and the
    // constants weren't updated -- the platform just mapped the WRONG byte
    // range, and translating preset offsets through it would play garbage
    // (or walk off the map). Silent-but-logged beats garbage-or-crash.
    if (first_sample == AMY_GM_BIG_EMU4_FIRST_SAMPLE &&
        (gm_big_map[1903 - GM_BIG_PRESET_BASE].offset != AMY_GM_BIG_EMU4_FIRST_SAMPLE ||
         gm_big_map[2060 - GM_BIG_PRESET_BASE].offset +
         gm_big_map[2060 - GM_BIG_PRESET_BASE].length != AMY_GM_BIG_EMU4_END_SAMPLE)) {
        fprintf(stderr, "gm_big: emu4 window constants out of sync with gm_big_map; big bank disabled\n");
        gm_big_pcm = NULL;
        return;
    }
    gm_big_pcm = data;
    gm_big_first_sample = first_sample;
    gm_big_num_samples = num_samples;
}
#endif


// Get either memory preset, file preset or baked in preset for preset number.
// For ROM presets, fill the caller-provided rom_local and return it.
memorypcm_preset_t * get_preset_for_preset_number(uint16_t preset_number,
                                                  memorypcm_preset_t *rom_local) {
    // Get the memory preset. If we can't find it, it could be a ROM preset. So copy params in from ROM preset
    memorypcm_ll_t *preset = memorypcm_ll_start;
    while(preset != NULL) {
        if(preset->preset_number == preset_number) {
            if(preset->preset->sample_ram != NULL || preset->preset->file_handle > 0) {
                return preset->preset;
            }
        }
        preset = preset->next;
    }

#ifdef GAMMA9001
    // Gamma9001 drum banks live at GAMMA9001_PRESET_BASE+, read straight out
    // of the platform-provided blob (memory presets above may still shadow them).
    if (preset_number >= GAMMA9001_PRESET_BASE &&
        preset_number < GAMMA9001_PRESET_BASE + GAMMA9001_NUM_SAMPLES &&
        gamma9001_pcm != NULL && rom_local != NULL) {
        const pcm_map_t *g = &gamma9001_map[preset_number - GAMMA9001_PRESET_BASE];
        memset(rom_local, 0, sizeof(*rom_local));
        rom_local->sample_ram = (int16_t *)gamma9001_pcm + g->offset;
        rom_local->length = g->length;
        rom_local->loopstart = g->loopstart;
        rom_local->loopend = g->loopend;
        rom_local->midinote = g->midinote;
        rom_local->samplerate = GAMMA9001_SAMPLE_RATE;
        rom_local->log2sr = log2f((float)GAMMA9001_SAMPLE_RATE / ZERO_LOGFREQ_IN_HZ);
        rom_local->type = AMY_PCM_TYPE_GAMMA;
        rom_local->channels = 1;
        return rom_local;
    }
#endif

#ifdef GM_FONTS
    // GM SoundFont presets live at GM_PRESET_BASE+, read straight out of the
    // mmapped fonts partition (memory presets above may still shadow them).
    if (preset_number >= GM_PRESET_BASE &&
        preset_number < GM_PRESET_BASE + GM_NUM_SAMPLES &&
        gm_pcm != NULL && rom_local != NULL) {
        const pcm_map_t *g = &gm_map[preset_number - GM_PRESET_BASE];
        memset(rom_local, 0, sizeof(*rom_local));
        rom_local->sample_ram = (int16_t *)gm_pcm + g->offset;
        rom_local->length = g->length;
        rom_local->loopstart = g->loopstart;
        rom_local->loopend = g->loopend;
        rom_local->midinote = g->midinote;
        rom_local->samplerate = GM_SAMPLE_RATE;
        rom_local->log2sr = log2f((float)GM_SAMPLE_RATE / ZERO_LOGFREQ_IN_HZ);
        rom_local->type = AMY_PCM_TYPE_GAMMA;
        rom_local->channels = 1;
        return rom_local;
    }

    // Big multi-font bank presets live at GM_BIG_PRESET_BASE+, own mmap.
    if (preset_number >= GM_BIG_PRESET_BASE &&
        preset_number < GM_BIG_PRESET_BASE + GM_BIG_NUM_SAMPLES &&
        gm_big_pcm != NULL && rom_local != NULL) {
        const pcm_map_t *g = &gm_big_map[preset_number - GM_BIG_PRESET_BASE];
        // Range guard: only a window of the blob may be resident (the
        // ESP32-S3 maps just the emu4 slice). A preset outside the window
        // has no valid address -- return NULL (silent) rather than
        // dereferencing unmapped vaddr (reachable from a raw wire message
        // naming any big-bank preset number).
        if (g->offset < gm_big_first_sample)
            return NULL;
        uint32_t rel = g->offset - gm_big_first_sample;
        if (rel >= gm_big_num_samples || g->length > gm_big_num_samples - rel)
            return NULL;
        memset(rom_local, 0, sizeof(*rom_local));
        rom_local->sample_ram = (int16_t *)gm_big_pcm + rel;
        rom_local->length = g->length;
        rom_local->loopstart = g->loopstart;
        rom_local->loopend = g->loopend;
        rom_local->midinote = g->midinote;
        rom_local->samplerate = GM_BIG_SAMPLE_RATE;
        rom_local->log2sr = log2f((float)GM_BIG_SAMPLE_RATE / ZERO_LOGFREQ_IN_HZ);
        rom_local->type = AMY_PCM_TYPE_GAMMA;
        rom_local->channels = 1;
        return rom_local;
    }
#endif

    // No memory preset found, so try ROM preset. default to 0 if out of range
    if (preset_number >= pcm_samples) preset_number = 0; 
    if (rom_local == NULL) {
        return NULL;
    }
    memset(rom_local, 0, sizeof(*rom_local));
    const pcm_map_t cpreset =  pcm_map[preset_number];
    uint32_t offset = cpreset.offset;
    uint32_t length = cpreset.length;
#ifdef PCM_LENGTH
    if (offset >= PCM_LENGTH) {
        offset = 0;
        length = 0;
    } else if (length > (PCM_LENGTH - offset)) {
        length = PCM_LENGTH - offset;
    }
#endif
    rom_local->sample_ram = (int16_t*)pcm + offset;
    rom_local->length = length;
    rom_local->loopstart = cpreset.loopstart;
    rom_local->loopend = cpreset.loopend;
    if (rom_local->loopstart > rom_local->length) {
        rom_local->loopstart = 0;
    }
    if (rom_local->loopend > rom_local->length) {
        rom_local->loopend = rom_local->length;
    }
    rom_local->midinote = cpreset.midinote;
    rom_local->samplerate = PCM_AMY_SAMPLE_RATE;
    rom_local->log2sr = PCM_AMY_LOG2_SAMPLE_RATE;
    rom_local->type = AMY_PCM_TYPE_ROM;
    rom_local->channels = 1;
    return rom_local;
}

const int16_t *pcm_get_sample_ram_for_preset(uint16_t preset_number, uint32_t *length) {
    memorypcm_preset_t rom_local;
    memorypcm_preset_t *preset = get_preset_for_preset_number(preset_number, &rom_local);
    if (length != NULL) {
        *length = (preset != NULL) ? preset->length : 0;
    }
    if (preset == NULL) {
        return NULL;
    }
    return preset->sample_ram;
}


void pcm_init() {
    memorypcm_ll_start = NULL;
}
void pcm_deinit() {
    pcm_unload_all_presets();
}

// How many bits used for fractional part of PCM table index.
#define PCM_INDEX_FRAC_BITS 8
// The number of bits used to hold the table index.
#define PCM_INDEX_BITS (31 - PCM_INDEX_FRAC_BITS)

static void fclose_if_file(memorypcm_preset_t *preset) {
    if (preset == NULL) {
        return;
    }
    if (preset->type == AMY_PCM_TYPE_FILE &&
        preset->file_handle != 0 &&
        amy_global.config.amy_external_fclose_hook != NULL) {
        amy_global.config.amy_external_fclose_hook(preset->file_handle);
        preset->file_handle = 0;
    }
}

void pcm_note_on(uint16_t osc) {
    if(AMY_IS_SET(synth[osc]->preset)) {
        memorypcm_preset_t rom_local;
        memorypcm_preset_t *preset =
            get_preset_for_preset_number(synth[osc]->preset, &rom_local);
        if (preset == NULL) {
            // Unresolvable preset (e.g. big-bank preset outside the mapped
            // emu4 window): stay silent, don't dereference.
            synth[osc]->status = SYNTH_OFF;
            return;
        }
        if (preset->type == AMY_PCM_TYPE_FILE) {
            if (preset->file_handle != 0) {
                wave_info_t info = {0};
                uint32_t data_bytes = 0;
                amy_global.config.amy_external_fseek_hook(preset->file_handle, 0);
                if (wave_parse_header(preset->file_handle, &info, &data_bytes)) {
                    preset->channels = info.channels;
                    preset->samplerate = info.sample_rate;
                    preset->log2sr = log2f((float)info.sample_rate / ZERO_LOGFREQ_IN_HZ);
                    preset->file_bytes_remaining = data_bytes;
                } else {
                    // Re-parse failed (review C7): close AND clear the handle,
                    // then force the voice off. The old code left file_handle
                    // non-zero on a now-closed handle, so the next render_pcm
                    // issued fseek/read on it. Zeroing file_handle + marking the
                    // voice off mirrors render_pcm's own sample_ram==NULL bail.
                    amy_global.config.amy_external_fclose_hook(preset->file_handle);
                    preset->file_handle = 0;
                    synth[osc]->status = SYNTH_OFF;
                }
            }
        } else if (preset->type == AMY_PCM_TYPE_ROM) {
            // baked-in PCM - don't overrun.
            if(synth[osc]->preset >= pcm_samples) synth[osc]->preset = 0;
        }
        
        // Declick: if this osc was still emitting audio (voice reuse/steal),
        // fold its last output into a decaying offset so the phase reset below
        // doesn't produce a step. Fresh/silent voices have pcm_last_out == 0.
        // (+=: a retrigger arriving before an earlier declick has drained
        // accumulates rather than dropping the earlier offset.)
        synth[osc]->pcm_declick += synth[osc]->pcm_last_out;
        synth[osc]->pcm_last_out = 0;

        synth[osc]->phase = 0; // s16.15 index into the table; as if a PHASOR into a 16 bit sample table.
        // Special case: We use the msynth feedback flag to indicate note-off for looping PCM.  As a result, it's explicitly NOT set in amy:hold_and_modify for PCM voices.  Set it here.
        msynth[osc]->feedback = synth[osc]->feedback;

        // Make sure PCM waveforms are excluded from auto-termination, so we don't cut-off samples with silent gaps.
        synth[osc]->terminate_on_silence = 0;
    }
}

void pcm_mod_trigger(uint16_t osc) {
    pcm_note_on(osc);
}


void pcm_note_off(uint16_t osc) {
    if(AMY_IS_SET(synth[osc]->preset)) {
        // feedback >= 2: "sustain through release" (from Leeman1982/amy) -- the
        // host has an amp envelope with a meaningful release stage, so don't
        // stop the sample here: looped presets keep looping, one-shots keep
        // playing to their natural end, and the EG fades the voice (the osc
        // stops when the envelope completes). Without this, the release stage
        // had at most the sample's loop tail (~tens of ms) to act on.
        if(msynth[osc]->feedback >= 2) {
            return;
        }
        uint32_t length = 0;
        memorypcm_preset_t rom_local;
        memorypcm_preset_t *preset =
            get_preset_for_preset_number(synth[osc]->preset, &rom_local);
        if(preset != NULL) {
            length = preset->length;
        }
        if(msynth[osc]->feedback == 0) {
            // Non-looping note: Set phase to the end to cause immediate stop.
            synth[osc]->phase = F2P(length / (float)(1 << PCM_INDEX_BITS));
        } else {
            // Looping is requested, disable future looping, sample will play through to end.
            // (sending a second note-off will stop it immediately).
            msynth[osc]->feedback = 0;
        }
    }
}


uint32_t fill_sample_from_file(memorypcm_preset_t *preset_p, uint32_t frames_needed) {
    //fprintf(stderr, "fsff %ld frames\n", frames_needed);
    uint32_t bytes_per_frame = preset_p->channels * 2;
    uint32_t frames_available = 0;
    if (bytes_per_frame > 0) {
        frames_available = preset_p->file_bytes_remaining / bytes_per_frame;
    }
    if (frames_available > 0 && frames_needed > frames_available) {
        frames_needed = frames_available;
    }
    uint32_t frames_read = wave_read_pcm_frames_s16(
        preset_p->file_handle,
        preset_p->channels,
        &preset_p->file_bytes_remaining,
        preset_p->sample_ram,
        frames_needed);
    return frames_read;
}

SAMPLE render_pcm(SAMPLE* buf, uint16_t osc) {
    if(AMY_IS_SET(synth[osc]->preset)) {
        SAMPLE max_value = 0;
        memorypcm_preset_t rom_local;
        memorypcm_preset_t *preset =
            get_preset_for_preset_number(synth[osc]->preset, &rom_local);
        if (preset == NULL) {
            // Unresolvable preset (e.g. big-bank preset outside the mapped
            // emu4 window): render silence and shut the voice off.
            synth[osc]->status = SYNTH_OFF;
            return 0;
        }
        float logfreq = msynth[osc]->logfreq;
        // If osc[midi_note] is set, shift the freq by the preset's default base_note.
        if (AMY_IS_SET(synth[osc]->midi_note)) {
            logfreq -= logfreq_for_midi_note(preset->midinote);
        }
        float playback_freq = freq_of_logfreq(preset->log2sr + logfreq);
        uint32_t sample_length = preset->length;
        if (preset->type == AMY_PCM_TYPE_FILE) {
            float frames_per_output = playback_freq / (float)AMY_SAMPLE_RATE;
            uint32_t frames_needed = (uint32_t)ceilf(frames_per_output * AMY_BLOCK_SIZE) + 1;
            uint32_t max_frames = AMY_BLOCK_SIZE * PCM_FILE_BUFFER_MULT;
            if (frames_needed > max_frames) {
                frames_needed = max_frames;
            }
            sample_length = fill_sample_from_file(preset, frames_needed);
            if(sample_length != frames_needed) {
                // reached end of file
                synth[osc]->status = SYNTH_OFF;
            }
            synth[osc]->phase = 0;
        }
        if (preset->sample_ram == NULL || sample_length == 0) {
            synth[osc]->status = SYNTH_OFF;
            return 0;
        }
        if (amy_flash_fence
            && (const void *)preset->sample_ram >= amy_flash_fence_lo
            && (const void *)preset->sample_ram < amy_flash_fence_hi) {
            // A flash write is in progress and this sample lives in mapped
            // flash: fetching it now would fault (cache suspended). Emit
            // silence, hold the phase; the voice resumes when the fence drops.
            return 0;
        }

        SAMPLE amp = F2S(msynth[osc]->amp);
        PHASOR step = F2P((playback_freq / (float)AMY_SAMPLE_RATE) / (float)(1 << PCM_INDEX_BITS));
        const LUTSAMPLE* table = preset->sample_ram;
        // Hoist the phase into a LOCAL for the block: synth[osc]->phase is a
        // PSRAM pointer-chase that this loop used to read AND write EVERY
        // sample -- the only renderer that didn't hoist (firmware review
        // C-5, ~1-4% of a core under drum kits). Written back once below.
        PHASOR phase = synth[osc]->phase;
        // Declick offset decays by 1/16 per sample (tau ~16 samples ~0.4ms); snap
        // to zero below ~2 LSB-of-16-bit so fixed-point decay can't stall nonzero.
        #define PCM_DECLICK_EPS F2S(0.000002f)
        SAMPLE last_out = synth[osc]->pcm_last_out;
        SAMPLE declick = synth[osc]->pcm_declick;
        uint8_t status_off = 0;
        uint32_t base_index = INT_OF_P(phase, PCM_INDEX_BITS);
        for(uint16_t i=0; i < AMY_BLOCK_SIZE; i++) {
            SAMPLE frac = S_FRAC_OF_P(phase, PCM_INDEX_BITS);
            LUTSAMPLE b = 0;
            LUTSAMPLE c = 0;
            uint32_t next_index = base_index + 1;
            if (base_index >= sample_length) {
                if (preset->type != AMY_PCM_TYPE_FILE) {
                    status_off = 1;
                }
                if (last_out != 0) { declick += last_out; last_out = 0; }
                SAMPLE dz = 0;
                if (declick != 0) {
                    dz = declick;
                    declick -= SHIFTR(declick, 4);
                    if (declick < PCM_DECLICK_EPS && declick > -PCM_DECLICK_EPS) declick = 0;
                }
                buf[i] = dz;
                // Count the emitted tail in the returned peak: the silent-voice
                // reaper (render_osc_wave) gates on max_val < AMP_THRESH, and
                // EG-released feedback>=2 PCM voices now set terminate_on_silence
                // -- an audible tail must keep the voice unreapable until it has
                // drained, whatever the decay constant or block size.
                if (dz < 0) dz = -dz;
                if (dz > max_value) max_value = dz;
                continue;
            }
            if (preset->channels == 2) {
                uint32_t base_offset = base_index * 2;
                uint32_t next_offset = next_index * 2;
                if (synth[osc]->wave == PCM_LEFT) {
                    b = table[base_offset];
                    c = (next_index < sample_length) ? table[next_offset] : b;
                } else if (synth[osc]->wave == PCM_RIGHT) {
                    b = table[base_offset + 1];
                    c = (next_index < sample_length) ? table[next_offset + 1] : b;
                } else { // PCM or PCM_MIX
                    // >> 1 instead of / 2 (review C4): signed int32 operands, so
                    // the shift floors toward -inf where / 2 truncated toward 0
                    // -- a 1-LSB rounding-direction change on negative sums,
                    // acceptable per the review, and it drops the per-sample
                    // integer divide on the drum-kit hot path.
                    LUTSAMPLE bl = table[base_offset];
                    LUTSAMPLE br = table[base_offset + 1];
                    b = (LUTSAMPLE)(((int32_t)bl + (int32_t)br) >> 1);
                    if (next_index < sample_length) {
                        LUTSAMPLE cl = table[next_offset];
                        LUTSAMPLE cr = table[next_offset + 1];
                        c = (LUTSAMPLE)(((int32_t)cl + (int32_t)cr) >> 1);
                    } else {
                        c = b;
                    }
                }
            } else {
                b = table[base_index];
                c = (next_index < sample_length) ? table[next_index] : b;
            }
            SAMPLE sample = L2S(b) + MUL4_SS(L2S(c - b), frac);
            phase = P_WRAPPED_SUM(phase, step);
            base_index = INT_OF_P(phase, PCM_INDEX_BITS);

            if(preset->type != AMY_PCM_TYPE_FILE) {
                // For non-file samples, we have to check for end of sample/looping.
                if(base_index >= sample_length) { // end
                    status_off = 1;
                    sample = 0;
                    if (last_out != 0) { declick += last_out; last_out = 0; }
                } else {
                    if(msynth[osc]->feedback > 0) { // still looping.  The feedback flag is cleared by pcm_note_off.
                        if(base_index >= preset->loopend) { // loopend
                            // back to loopstart -- but only for a real sustain
                            // loop. One-shot presets have loopend == length;
                            // wrapping those would machine-gun the whole sample
                            // (reachable via feedback >= 2 sustain-through-
                            // release on a one-shot).
                            int32_t loop_len = preset->loopend - preset->loopstart;
                            if(loop_len > 0 && (uint32_t)loop_len < preset->length) {
                                phase -= F2P(loop_len / (float)(1 << PCM_INDEX_BITS));
                                base_index -= loop_len;
                            }
                        }
                    }
                }
            }
            SAMPLE out = MUL4_SS(amp, sample);
            last_out = out;              // this osc's own contribution, excluding declick
            if (declick != 0) {
                out += declick;
                declick -= SHIFTR(declick, 4);
                if (declick < PCM_DECLICK_EPS && declick > -PCM_DECLICK_EPS) declick = 0;
            }
            SAMPLE value = buf[i] + out;
            buf[i] = value;
            if (value < 0) value = -value;
            if (value > max_value) max_value = value;
        }
        synth[osc]->phase = phase;         // hoisted local written back once
        synth[osc]->pcm_last_out = last_out;
        synth[osc]->pcm_declick = declick;
        // Defer the off until the declick tail has fully drained (<= ~2 blocks):
        // next block renders the past-end path above, which emits only the
        // decaying offset and re-raises status_off until declick snaps to 0.
        if (status_off && declick == 0) synth[osc]->status = SYNTH_OFF;
        //printf("render_pcm: osc %d preset %d len %d base_ix %d phase %f step %f tablestep %f amp %f\n",
        //       osc, synth[osc]->preset, preset->length, base_index, P2F(synth[osc]->phase), P2F(step), (1 << PCM_INDEX_BITS) * P2F(step), S2F(msynth[osc]->amp));
        return max_value;
        // i don't believe we ever need to detect silence in a sample. it will shut itself off at the end.
    }
    return 0;
}


SAMPLE compute_mod_pcm(uint16_t osc) {
    if(AMY_IS_SET(synth[osc]->preset)) {
        SAMPLE buf[AMY_BLOCK_SIZE];
        memset(buf, 0, sizeof(buf));
        render_pcm(buf, osc);
        return buf[0];
    }
    return 0;
}


int pcm_load_file() {
    // We pass the inputs to this as aliases in the amy_global structure. This is to not destroy the MP heap for amy->AMYboard
    uint8_t midinote = amy_global.transfer_stored_bytes;
    uint16_t preset_number = amy_global.transfer_file_handle;
    char * filename = amy_global.transfer_filename;

    pcm_unload_preset(preset_number);
    if (filename == NULL || filename[0] == '\0') {
        return 0;
    }
    if (amy_global.config.amy_external_fopen_hook == NULL || amy_global.config.amy_external_fclose_hook == NULL) {
        fprintf(stderr, "fopen hook not enabled on platform\n");
        return 0;
    }
    uint32_t handle = amy_global.config.amy_external_fopen_hook((char *)filename, "rb");
    if (handle == 0) {
        fprintf(stderr, "Could not open file %s\n", filename);
        return 0;
    }
    wave_info_t info = {0};
    uint32_t data_bytes = 0;
    if (!wave_parse_header(handle, &info, &data_bytes)) {
        fprintf(stderr, "Could not parse WAVE file %s\n", filename);
        amy_global.config.amy_external_fclose_hook(handle);
        return 0;
    }
    uint32_t total_frames = 0;
    if (info.channels > 0) {
        total_frames = data_bytes / (info.channels * 2);
    }
    uint32_t buffer_frames = AMY_BLOCK_SIZE * PCM_FILE_BUFFER_MULT;
    memorypcm_ll_t *new_preset_pointer = malloc_caps(
        sizeof(memorypcm_ll_t) + sizeof(memorypcm_preset_t) + buffer_frames * sizeof(int16_t),
        amy_global.config.ram_caps_sample);
    if (new_preset_pointer == NULL) {
        fprintf(stderr, "No RAM left for sample load\n");
        return 0;
    }
    new_preset_pointer->next = memorypcm_ll_start;
    memorypcm_ll_start = new_preset_pointer;
    new_preset_pointer->preset_number = preset_number;
    memorypcm_preset_t *memory_preset =
        (memorypcm_preset_t *)(((uint8_t *)new_preset_pointer) + sizeof(memorypcm_ll_t));
    strncpy(memory_preset->filename, filename, MAX_FILENAME_LEN - 1);
    memory_preset->filename[MAX_FILENAME_LEN - 1] = '\0';
    memory_preset->channels = info.channels;
    memory_preset->samplerate = info.sample_rate;
    memory_preset->log2sr = log2f((float)info.sample_rate / ZERO_LOGFREQ_IN_HZ);
    memory_preset->midinote = midinote;
    memory_preset->length = total_frames;
    memory_preset->loopstart = 0;
    memory_preset->loopend = 0;
    memory_preset->type = AMY_PCM_TYPE_FILE;
    memory_preset->file_bytes_remaining = total_frames * info.channels * 2;
    memory_preset->file_handle = handle;
    memory_preset->sample_ram = malloc_caps(buffer_frames * info.channels * sizeof(int16_t),
                                                     amy_global.config.ram_caps_sample);
    new_preset_pointer->preset = memory_preset;
    //fprintf(stderr, "read file %s frames %ld channels %d preset %d handle %ld\n", filename, total_frames, info.channels, preset_number, handle);
    return 1;
}


// load mono samples (let python parse wave files) into preset # 
// set loopstart, loopend, midinote, samplerate (and log2sr)
// return the allocated sample ram that AMY will fill in.
int16_t * pcm_load(uint16_t preset_number, uint32_t length, uint32_t samplerate, uint8_t channels, uint8_t midinote, uint32_t loopstart, uint32_t loopend) {
    // if preset was already a memorypcm, we need to unload it
    pcm_unload_preset(preset_number); // this is a no-op if preset doesn't exist or is a const pcm
    // now alloc a new LL entry and preset (the old LL entry is removed with pcm_unload_preset)
    memorypcm_ll_t *new_preset_pointer = malloc_caps(sizeof(memorypcm_ll_t) + sizeof(memorypcm_preset_t) + length * channels * sizeof(int16_t),
						     amy_global.config.ram_caps_sample);
    if(new_preset_pointer  == NULL) {
        fprintf(stderr, "No RAM left for sample load\n");
        return NULL; // no ram for sample
    }
    new_preset_pointer->next = memorypcm_ll_start;
    memorypcm_ll_start = new_preset_pointer;
    new_preset_pointer->preset_number = preset_number;
    memorypcm_preset_t *memory_preset = (memorypcm_preset_t *)(((uint8_t *)new_preset_pointer) + sizeof(memorypcm_ll_t));
    memory_preset->samplerate = samplerate;
    memory_preset->log2sr = log2f((float)samplerate / ZERO_LOGFREQ_IN_HZ);
    memory_preset->midinote = midinote;
    memory_preset->loopstart = loopstart;
    memory_preset->length = length;
    memory_preset->channels = channels;
    memory_preset->filename[0] = '\0';
    memory_preset->file_bytes_remaining = 0;
    memory_preset->file_handle = 0;
    memory_preset->type = AMY_PCM_TYPE_MEMORY;
    memory_preset->sample_ram = (int16_t *)(((uint8_t *)memory_preset) + sizeof(memorypcm_preset_t));
    if(loopend == 0) {  // loop whole sample
        memory_preset->loopend = (length > 0) ? memory_preset->length-1 : 0;
    } else {
        memory_preset->loopend = loopend;
    }
    // Clamp the loop window (review FW-13): loopstart >= loopend or
    // loopend > length produced undefined phase math in pcm_note_off.
    if (memory_preset->loopend >= length && length > 0)
        memory_preset->loopend = length - 1;
    if (memory_preset->loopstart >= memory_preset->loopend)
        memory_preset->loopstart = 0;
    new_preset_pointer->preset = memory_preset;
    return memory_preset->sample_ram;
}

void pcm_unload_preset(uint16_t preset_number) {
    // run through the LL looking for the preset
    memorypcm_ll_t **preset_pointer = &memorypcm_ll_start;
    while(*preset_pointer != NULL) {
        if((*preset_pointer)->preset_number == preset_number) {
            memorypcm_ll_t *next = (*preset_pointer)->next;
            fclose_if_file((*preset_pointer)->preset);
            // free the memory we allocated
            free((*preset_pointer));
            // close up the list
            *preset_pointer = next;
            return;
        } else {
            preset_pointer = &(*preset_pointer)->next;
        }
    }
    //fprintf(stderr, "pcm_unload_preset: preset %d not found\n", preset_number);  // This happens during a routine load_preset.
}

void pcm_unload_all_presets() {
    memorypcm_ll_t *preset_pointer = memorypcm_ll_start;
    while(preset_pointer != NULL) {
        memorypcm_ll_t *next_pointer = preset_pointer->next;
        fclose_if_file(preset_pointer->preset);
        free(preset_pointer);
        // Go to the next one
        preset_pointer = next_pointer;
    }
    memorypcm_ll_start = NULL;
}
