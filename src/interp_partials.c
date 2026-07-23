// interp_partials - AMY kernel-side implementation of the interpolated partials-based synthesis originally implemented in tulip_piano.py.

#include "amy.h"
#include <stdbool.h>

typedef struct {
    // How many sample_times_ms are there?
    uint16_t num_sample_times_ms;
    // Pointer to an array of the sample_times_ms
    const uint16_t *sample_times_ms;
    // How many velocities are defined for this voice (same for all notes)
    uint16_t num_velocities;
    // Pointer to a array of the MIDI velocities.
    const uint8_t *velocities;
    // How many different pitches do we define?  (All velocities are provided for each)
    uint16_t num_pitches;
    // Pointer to array of structures defining each note (pitch + velocity) entry.
    const uint8_t *pitches;
    // How many harmonics are allocated for each of the num_velocities * num_pitches notes.
    const uint8_t *num_harmonics;
    // MIDI Cents freqs for each harmonic.
    const uint16_t *harmonics_freq;
    // num_sample_times_ms uint8_t dB envelope values for each harmonic.
    const uint8_t *harmonics_mags;
} interp_partials_voice_t;

#include "interp_partials.h"

#define MAX_NUM_MAGNITUDES 24

#define MAX_NUM_HARMONICS 40

// Map to drop out some higher harmonics, namely the 2x and 3x overtones above 16th harmonic
const bool use_this_partial_map[MAX_NUM_HARMONICS] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 1-10
    1, 1, 1, 1, 1, 1, 1, 0, 1, 0,  // 11-20
    0, 0, 1, 0, 1, 0, 0, 0, 1, 0,  // 21-30
    1, 0, 0, 0, 1, 0, 1, 0, 0, 0,  // 31-40
};

// Runtime partial-detail knob (OPT-8): harmonics at/above this index are
// dropped in ADDITION to the static map. A sustained voice at full detail
// runs ~24 partial oscs (~14% of a core); capping at 16 recovers roughly a
// third of that for a subtle top-end change. Change it only while no
// partials notes are held.
//
// IMPORTANT (OPT-8 fix): the cap must only limit which partials SOUND.  The
// voice's osc footprint (patch_oscs[] span, note-on shielding, note-off
// release) is fixed by the STATIC map alone -- see
// _span_partials_for_partials_voice() below.  Per-voice note events are
// broadcast to EVERY osc in the voice's span (patches.c
// patches_event_has_voices), and any span osc not marked
// SYNTH_IS_ALGO_SOURCE gets note-on'd as a default full-price audible SINE
// osc by the VELOCITY delta.  When the shield loop was cap-limited, lowering
// the cap un-shielded (span - cap) oscs per voice, so LOWER detail cost MORE
// CPU (measured inverted: cap 40 ~2.99M cyc, cap 15 ~3.67M, cap 8 ~3.89M for
// a 6-note chord) plus a stray sine layered on each note.
uint8_t amy_partials_harmonic_limit = MAX_NUM_HARMONICS;

// Runtime SUSTAIN knob for the interp-partials (piano) engine: a time-stretch
// multiplier applied to the per-partial dB envelope at note-on.  The piano's
// decay is baked as breakpoint TIMES (piano_sample_times_ms, 4..4096 ms);
// multiplying every inter-breakpoint delta by this factor plays the SAME
// spectral trajectory more slowly, so the note rings ~factor-x longer without
// changing its timbre.  1.0f == natural (bit-identical to stock; the multiply
// is skipped).  Set via tulip.piano_sustain(); like amy_partials_harmonic_limit
// it only affects NEW note-ons, so change it while no piano notes are held.
float amy_partials_time_stretch = 1.0f;

static inline bool use_partial(int h) {
    return h < amy_partials_harmonic_limit && use_this_partial_map[h];
}

// Number of partials this h index COULD use, ignoring the runtime cap: the
// static-map count only.  This is the voice's reserved osc span (minus the
// control osc) and matches the generated patch_oscs[] entry; it must NOT
// shrink with amy_partials_harmonic_limit or span oscs escape shielding.
static inline bool span_partial(int h) {
    return use_this_partial_map[h];
}


// choose a preset from the .h file
void partials_note_on(uint16_t osc) {
    int num_partials = synth[osc]->preset;
    // preset arrives unclamped from the wire ('p'), and we touch oscs
    // osc+1..osc+num_partials -- clamp so we never index past synth[].
    if (num_partials > AMY_OSCS - 1 - osc) {
        fprintf(stderr, "partials_note_on: osc %d preset %d exceeds AMY_OSCS %d, clamping\n",
                osc, num_partials, AMY_OSCS);
        num_partials = (osc < AMY_OSCS - 1) ? (AMY_OSCS - 1 - osc) : 0;
    }
    for (int i = 0; i < num_partials; ++i) {
        int o = osc + 1 + i;
        ensure_osc_allocd(o, NULL);
        // Mark this PARTIAL as part of a build-your own with a flag value in its preset field.
        // This is used I think only at envelope.c:121 to avoid the normal partial preset special-case for PARTIALs.
        synth[o]->preset = synth[osc]->preset;
        synth[o]->logfreq_coefs[COEF_BEND] = 0;  // Each PARTIAL will receive pitch bend via the midi_note modulation from the parent osc, don't add it twice.
        synth[o]->status = SYNTH_IS_ALGO_SOURCE;
        synth[o]->note_on_clock = amy_global.total_blocks*AMY_BLOCK_SIZE;
        AMY_UNSET(synth[o]->note_off_clock);
        msynth[o]->logfreq = synth[o]->logfreq_coefs[COEF_CONST] + msynth[osc]->logfreq;
        partial_note_on(o);
    }
}

void partials_note_off(uint16_t osc) {
    int num_oscs = synth[osc]->preset;
    // Same clamp as partials_note_on: preset is unclamped wire data.
    if (num_oscs > AMY_OSCS - 1 - osc) num_oscs = (osc < AMY_OSCS - 1) ? (AMY_OSCS - 1 - osc) : 0;
    for(uint16_t i = osc + 1; i < osc + 1 + num_oscs; i++) {
        uint16_t o = i % AMY_OSCS;
        if (synth[o] == NULL) continue;
        AMY_UNSET(synth[o]->note_on_clock);
        synth[o]->note_off_clock = amy_global.total_blocks*AMY_BLOCK_SIZE;
    }
}


// render a full partial set at offset osc (with preset)
// freq controls pitch_ratio, amp amp_ratio, ratio controls time ratio
// do all presets have sustain point?
SAMPLE render_partials(SAMPLE *buf, uint16_t osc) {
    SAMPLE max_value = 0;
    uint16_t num_oscs = 0;
    // No preset partials map, we are in "build-your-own".  The max number of oscs is taken from algo_source[0].
    num_oscs = synth[osc]->preset;

    if (synth[osc]->wave == INTERP_PARTIALS) {
        //const interp_partials_voice_t *partials_voice = &interp_partials_map[synth[osc]->preset % NUM_INTERP_PARTIALS_PRESETS];
        //num_oscs = partials_voice->num_harmonics[0];   // Assume first preset has the max #harmonics.
        num_oscs = interp_partials_max_partials_for_patch(synth[osc]->preset);
    }
    // Same clamp as partials_note_on: preset is unclamped wire data.
    if ((int)num_oscs > AMY_OSCS - 1 - osc) num_oscs = (osc < AMY_OSCS - 1) ? (AMY_OSCS - 1 - osc) : 0;

    // now, render everything, add it up
    float midi_note = midi_note_for_logfreq(msynth[osc]->logfreq);
    //fprintf(stderr, "t=%u partials o=%d msynth[osc]->logfreq=%f midi_note=%f msynth[amp]=%f\n", amy_global.total_blocks*AMY_BLOCK_SIZE, osc, msynth[osc]->logfreq, midi_note, msynth[osc]->amp);
    for(uint16_t i = osc + 1; i < osc + 1 + num_oscs; i++) {
        uint16_t o = i % AMY_OSCS;
        if(synth[o] == NULL) continue;
        if(synth[o]->status ==SYNTH_IS_ALGO_SOURCE) {
            // We vary each partial's "velocity" on-the-fly as the way the parent osc's amplitude envelope contributes to the partials.
            synth[o]->velocity = msynth[osc]->amp;
            // We also use dynamic, fractional note to propagate parent freq modulation.
            synth[o]->midi_note = midi_note;
            // hold_and_modify contains a special case for wave == PARTIAL so that
            // envelope value are delayed by 1 frame compared to other oscs
            // so that partials fade in over one frame from zero amp.
            hold_and_modify(o);
            //printf("[%d %d] %d amp %f (%f) freq %f (%f) on %d off %d bp0 %d %f bp1 %d %f wave %d\n", amy_global.total_blocks*AMY_BLOCK_SIZE, ms_since_started, o, synth[o]->amp, msynth[o]->amp, synth[o]->freq, msynth[o]->freq, synth[o]->note_on_clock, synth[o]->note_off_clock, synth[o]->breakpoint_times[0][0], 
            //    synth[o]->breakpoint_values[0][0], synth[o]->breakpoint_times[1][0], synth[o]->breakpoint_values[1][0], synth[o]->wave);
            SAMPLE value = render_partial(buf, o);
            //fprintf(stderr, "render_partials: time %.3f osc %d ctl ampt %.6f msynth_amp %.6f max_val=%.6f\n", amy_global.time, o, msynth[osc]->amp, msynth[o]->amp, S2F(value));
            if (value > max_value) max_value = value;
        }
    }
    return max_value;
}


int _max_partials_for_partials_voice(const interp_partials_voice_t *partials_voice) {
    int max_num_partials = 0;
    for (int h = 0; h < partials_voice->num_harmonics[0]; ++h) {
        if (use_partial(h)) ++max_num_partials;
    }
    return max_num_partials;
}

// Cap-independent partial span of a voice (static map only).  Used for the
// note-on shield and note-off release loops, which must always cover the
// voice's FULL reserved osc range regardless of the current detail cap.
int _span_partials_for_partials_voice(const interp_partials_voice_t *partials_voice) {
    int span = 0;
    for (int h = 0; h < partials_voice->num_harmonics[0]; ++h) {
        if (span_partial(h)) ++span;
    }
    return span;
}

int interp_partials_max_partials_for_patch(int interp_partials_patch_number) {
    const interp_partials_voice_t *partials_voice = &interp_partials_map[interp_partials_patch_number % NUM_INTERP_PARTIALS_PRESETS];
    return _max_partials_for_partials_voice(partials_voice);
}

void _cumulate_scaled_harmonic_params(float *harm_param, int harmonic_index, float alpha, const interp_partials_voice_t *partials_voice) {
    int num_bps = partials_voice->num_sample_times_ms;
    // Pitch
    harm_param[0] += alpha * partials_voice->harmonics_freq[harmonic_index];
    // Envelope magnitudes
    for (int i = 0; i < num_bps; ++i)
        harm_param[1 + i] += alpha * partials_voice->harmonics_mags[harmonic_index * num_bps + i];
}

int _harmonic_base_index_for_pitch_vel(int pitch_index, int vel_index, const interp_partials_voice_t *partials_voice) {
    int note_number = partials_voice->num_velocities * pitch_index + vel_index;
    int harmonic_index = 0;
    for (int i = 0; i < note_number; ++i)
        harmonic_index += partials_voice->num_harmonics[i];
    return harmonic_index;
}

float _logfreq_of_midi_cents(float midi_cents) {
    // Frequency is already log scaled, but need to re-center and change from 1200/oct to 1.0/oct.
    return (midi_cents - (100 * ZERO_MIDI_NOTE)) / 1200.f;
}

float _env_lin_of_db(float db) {
    float lin =  powf(10.f, MIN(20.f, (db - 100.f)) / 20.f) - 0.001f;
    if (lin < 0)  return 0;
    return lin;
}

void _osc_on_with_harm_param(uint16_t o, float *harm_param, const interp_partials_voice_t *partials_voice) {
    // We coerce this voice into being a partial, regardless of user wishes.
    synth[o]->wave = PARTIAL;
    synth[o]->preset = 1;  // Flag that this is an envelope-based partial
    // Setup the specified frequency.
    synth[o]->logfreq_coefs[COEF_CONST] = _logfreq_of_midi_cents(harm_param[0]);
    // Setup envelope.
    //synth[o]->eg_type[0] = ENVELOPE_DB;
    synth[o]->breakpoint_times[0][0] = 0;
    synth[o]->breakpoint_values[0][0] = 0;
    int last_time = 0;
    for (int bp = 0; bp < partials_voice->num_sample_times_ms; ++bp) {
        // Base inter-breakpoint delta in samples (stock computation).
        uint32_t delta = (partials_voice->sample_times_ms[bp] - last_time) * AMY_SAMPLE_RATE / 1000;
        // SUSTAIN: stretch every segment by the same factor so the whole
        // spectral trajectory is preserved, just played slower => longer ring.
        // At stretch==1.0f this is skipped, leaving `delta` bit-identical to
        // stock.  breakpoint_times is uint32 samples: compute in double and
        // clamp to UINT32_MAX so a large stretch can never wrap the field.
        if (amy_partials_time_stretch != 1.0f) {
            double stretched = (double)delta * (double)amy_partials_time_stretch;
            delta = (stretched >= 4294967295.0) ? 4294967295u : (uint32_t)stretched;
        }
        synth[o]->breakpoint_times[0][bp + 1] = delta;
        synth[o]->breakpoint_values[0][bp + 1] = _env_lin_of_db(harm_param[bp + 1]);
        last_time = partials_voice->sample_times_ms[bp];
    }
    // Final release: deliberately NOT stretched.  This 200 ms ramp-to-zero is
    // the terminal damp of the voice; keeping it fixed means key-up / voice end
    // stays crisp regardless of the sustain setting (the audible ring is set by
    // the stretched breakpoints above, which reach far past this tail).
    synth[o]->breakpoint_times[0][partials_voice->num_sample_times_ms + 1] = 200 * AMY_SAMPLE_RATE / 1000;
    synth[o]->breakpoint_values[0][partials_voice->num_sample_times_ms + 1] = 0;
    // Decouple osc freq and amp from note and amp.
    synth[o]->logfreq_coefs[COEF_NOTE] = 0;
    synth[o]->amp_coefs[COEF_VEL] = 1.0;  // velocity is modified on-the-fly by the control osc to vary global amplitude.
    // Other osc params.
    synth[o]->status = SYNTH_IS_ALGO_SOURCE;
    synth[o]->note_on_clock = amy_global.total_blocks*AMY_BLOCK_SIZE;
    AMY_UNSET(synth[o]->note_off_clock);
    partial_note_on(o);
}

// HOW DOES INTERP_PARTIALS (e.g. DPWE_PIANO) WORK?
// The special thing about interp_partials is that the harmonic envelopes depend on the note velocity,
// so they all have to be recomputed in response at note_on time.  Then, because their values have been
// determined to reflect velocity via the note_on calculation, the parent osc should not use velocity
// as part of its overall scaling calculation, since it would otherwise be applied twice.
// Thus, when setting up a control osc for piano, we set amp_coef[COEF_VELOCITY] = 0.

void interp_partials_note_on(uint16_t osc) {
    // Choose the interp_partials preset.
    const interp_partials_voice_t *partials_voice = &interp_partials_map[synth[osc]->preset % NUM_INTERP_PARTIALS_PRESETS];
    float midi_note = synth[osc]->midi_note;
    float midi_vel = (int)roundf(synth[osc]->velocity * 127.f);
    // Clip velocity to the range covered by the tables.  Pitch is deliberately not clipped:
    // notes outside the table range are linearly extrapolated from the edge rows (pitch_alpha
    // outside [0, 1]); the index search below is bounded so table reads stay in range.
    if (midi_vel < partials_voice->velocities[0]) midi_vel = partials_voice->velocities[0];
    if (midi_vel > partials_voice->velocities[partials_voice->num_velocities - 1]) midi_vel = partials_voice->velocities[partials_voice->num_velocities - 1];
    // Find the lower bound pitch/velocity indices.
    uint8_t pitch_index = 0, vel_index = 0;
    while(pitch_index < partials_voice->num_pitches - 2   // We're going to inspect pitch_index + 1, so make sure that's in the table.
          && partials_voice->pitches[pitch_index + 1] < midi_note)
        ++pitch_index;
    while(vel_index < partials_voice->num_velocities - 1
          && partials_voice->velocities[vel_index + 1] < midi_vel)
        ++vel_index;
    // Interp weights
    float pitch_alpha = (midi_note - partials_voice->pitches[pitch_index])
        / (float)(partials_voice->pitches[pitch_index + 1] - partials_voice->pitches[pitch_index]);
    float vel_alpha = (midi_vel - partials_voice->velocities[vel_index])
        / (float)(partials_voice->velocities[vel_index + 1] - partials_voice->velocities[vel_index]);
    float harm_param[MAX_NUM_MAGNITUDES + 1];  // frequency + harmonic magnitudes.
    int note_number = partials_voice->num_velocities * pitch_index + vel_index;
    // Find the least number of harmonics across everything we're interpolating.
    int num_harmonics = MIN(MAX_NUM_HARMONICS, partials_voice->num_harmonics[note_number]);  // pl_vl note
    num_harmonics = MIN(num_harmonics, partials_voice->num_harmonics[note_number + 1]);  // pl_vh note
    num_harmonics = MIN(num_harmonics, partials_voice->num_harmonics[note_number + partials_voice->num_velocities]);  // ph_vl note
    num_harmonics = MIN(num_harmonics, partials_voice->num_harmonics[note_number + partials_voice->num_velocities + 1]);  // ph_vh note
    // Interpolate the 4 notes.
    int harmonic_base_index_pl_vl =
        _harmonic_base_index_for_pitch_vel(pitch_index, vel_index, partials_voice);
    float alpha_pl_vl = (1.f - pitch_alpha) * (1.f - vel_alpha);
    int harmonic_base_index_pl_vh =
        _harmonic_base_index_for_pitch_vel(pitch_index, vel_index + 1, partials_voice);
    float alpha_pl_vh = (1.f - pitch_alpha) * (vel_alpha);
    int harmonic_base_index_ph_vl =
        _harmonic_base_index_for_pitch_vel(pitch_index + 1, vel_index, partials_voice);
    float alpha_ph_vl = (pitch_alpha) * (1.f - vel_alpha);
    int harmonic_base_index_ph_vh =
        _harmonic_base_index_for_pitch_vel(pitch_index + 1, vel_index + 1, partials_voice);
    float alpha_ph_vh = (pitch_alpha) * (vel_alpha);
    //fprintf(stderr, "interp_partials@%u: osc %d note %.1f vel %.1f pitch_x %d vel_x %d numh %d harm_bi_ll %d pitch_a %.3f vel_a %.3f alphas %.2f %.2f %.2f %.2f\n",
    //        amy_global.total_blocks*AMY_BLOCK_SIZE, osc, midi_note, midi_vel, pitch_index, vel_index, num_harmonics,
    //        harmonic_base_index_pl_vl, pitch_alpha, vel_alpha,
    //        alpha_pl_vl, alpha_pl_vh, alpha_ph_vl, alpha_ph_vh);
    // Make sure enough oscs are alloc'd in our dynamic osc alloc world.
    // This has to be enough for any note in this map.  Assume num_harmonics[0] is largest (lowest pitch).
    uint8_t max_num_partials = _max_partials_for_partials_voice(partials_voice);
    // Alloc / shield over the voice's FULL cap-independent span: per-voice
    // note events are broadcast to every osc in the span (patches.c), so
    // every span osc must exist and be marked SYNTH_IS_ALGO_SOURCE or the
    // VELOCITY delta note-ons it as a stray audible default-SINE osc.
    // Oscs above the cap only need a default-size alloc (no 22-bp envelope);
    // if the cap is later raised, ensure_osc_allocd grows them in place.
    uint8_t span_num_partials = _span_partials_for_partials_voice(partials_voice);
    uint8_t max_num_breakpoints[MAX_BREAKPOINT_SETS] = {2 + partials_voice->num_sample_times_ms, DEFAULT_NUM_BREAKPOINTS};
    for (int o = 0; o < span_num_partials; ++o) {
        ensure_osc_allocd(osc + 1 + o, (o < max_num_partials) ? max_num_breakpoints : NULL);
    }
    int partial_osc = osc;
    for (int h = 0; h < num_harmonics; ++h) {
        if (use_partial(h)) {
            for (int i = 0; i < MAX_NUM_MAGNITUDES + 1; ++i)  harm_param[i] = 0;
            _cumulate_scaled_harmonic_params(harm_param, harmonic_base_index_pl_vl + h,
                                             alpha_pl_vl, partials_voice);
            _cumulate_scaled_harmonic_params(harm_param, harmonic_base_index_pl_vh + h,
                                             alpha_pl_vh, partials_voice);
            _cumulate_scaled_harmonic_params(harm_param, harmonic_base_index_ph_vl + h,
                                             alpha_ph_vl, partials_voice);
            _cumulate_scaled_harmonic_params(harm_param, harmonic_base_index_ph_vh + h,
                                             alpha_ph_vh, partials_voice);
            //fprintf(stderr, "harm %d freq %.2f bps %.3f %.3f %.3f %.3f\n", h, harm_param[0], harm_param[1], harm_param[2], harm_param[3], harm_param[4]);
            ++partial_osc;
            _osc_on_with_harm_param(partial_osc, harm_param, partials_voice);
        }
    }
    // Make sure any remaining oscs are still marked as ALGO_SOURCE.
    // Must cover the FULL span (not the capped count): this is the shield
    // that keeps the per-voice broadcast VELOCITY delta from note-on'ing
    // unused span oscs as stray audible sines (the OPT-8 CPU inversion).
    while(partial_osc < osc + 1 + span_num_partials)  { synth[partial_osc]->status = SYNTH_IS_ALGO_SOURCE; ++partial_osc; }
}

void interp_partials_note_off(uint16_t osc) {
    //const interp_partials_voice_t *partials_voice = &interp_partials_map[synth[osc]->preset % NUM_INTERP_PARTIALS_PRESETS];
    //int num_oscs = partials_voice->num_harmonics[0];   // Assume first preset has the max #harmonics.
    int num_oscs = 0; //MAX_NUM_HARMONICS;
    // Release the voice's FULL static-map span, NOT the cap-limited count:
    // if the detail cap was raised or lowered between this note's on and off,
    // partials beyond the current cap may still be sounding (or shield oscs
    // may exist); marking them released is always safe (they're skipped or
    // silent) and never leaves a partial ringing.
    for (int i = 0; i < MAX_NUM_HARMONICS; ++i) num_oscs += span_partial(i) ? 1 : 0;
    for(uint16_t i = osc + 1; i < osc + 1 + num_oscs; i++) {
        uint16_t o = i % AMY_OSCS;
        if (synth[o]) {  // For high notes, some partials may be unused, unintialized (?)
            AMY_UNSET(synth[o]->note_on_clock);
            synth[o]->note_off_clock = amy_global.total_blocks*AMY_BLOCK_SIZE;
        }
    }
}
