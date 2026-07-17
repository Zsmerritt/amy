// midi.c
// i deal with parsing and receiving midi on many platforms

#include "amy.h"
#if defined(TULIP) || defined(AMYBOARD)
#include "py/runtime.h"
// Forward-declare to avoid including the shared tinyusb header path. Defined
// in micropython/shared/tinyusb/mp_usbd_runtime.c and safe to call from the MP
// main thread to drain USB events (e.g. flush the MIDI IN endpoint FIFO).
extern void mp_usbd_task(void);
#endif
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#if defined(ESP_PLATFORM)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#endif

#if (defined ARDUINO_ARCH_RP2040) || (defined ARDUINO_ARCH_RP2350)
//#define TUD_USB_GADGET
#include "tusb.h"
#include "class/midi/midi.h"
#include "class/midi/midi_device.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#endif


#include "amy_midi.h"
// Parser state (running status, data-byte slot, inside-sysex) lives in ONE
// midi_stream_parser_t PER BYTE STREAM -- see midi_parsers[] below and the
// spec in amy_midi_parse.h. It used to be three shared globals, which tore
// messages whenever two sources (DIN + USB, or tulip.midi_local) arrived
// concurrently: a running-status data byte from one stream completed a
// message begun by the other.
static uint8_t external_midi_sync_enabled = 0;

void amy_external_midi_sync(uint8_t enabled) {
    external_midi_sync_enabled = enabled ? 1 : 0;
    // Turning sync off must restore internal clocking, otherwise the sequencer
    // stays latched to a (now silent) external clock and never ticks again.
    if (!external_midi_sync_enabled) sequencer_external_clock_disable();
}

#if 0
static void debug_print_midi_hex(const uint8_t *data, uint32_t len, uint8_t sysex) {
    fprintf(stderr, "MIDI %s len=%u:", sysex ? "sysex" : "msg", (unsigned)len);
    for (uint32_t i = 0; i < len; ++i) {
        fprintf(stderr, " %02X", data[i]);
    }
    fprintf(stderr, "\n");
}
#endif

// Send a MIDI note on OUT
void amy_send_midi_note_on(uint16_t osc) {
    // don't forward on a note coming in through MIDI IN 
    //fprintf(stderr, "amy_send_midi_note_on: osc %d source %d note %.1f vel %.3f\n",
    //        osc, synth[osc]->note_source_channel, synth[osc]->midi_note, synth[osc]->velocity);
    if(AMY_IS_UNSET(synth[osc]->note_source_channel)) {
        uint8_t bytes[3];
        bytes[0] = 0x90;
        bytes[1] = (uint8_t)roundf(synth[osc]->midi_note);
        bytes[2] = (uint8_t)roundf(synth[osc]->velocity*127.0f);
        midi_out(bytes, 3);
    }
}

// Send a MIDI note off OUT
void amy_send_midi_note_off(uint16_t osc) {
    // don't forward on a note coming in through MIDI IN 
    if(AMY_IS_UNSET(synth[osc]->note_source_channel)) {
        uint8_t bytes[3];
        // Send note-off as a note-on with vel 0.
        bytes[0] = 0x90;
        bytes[1] = (uint8_t)roundf(synth[osc]->midi_note);
        bytes[2] = 0;
        midi_out(bytes, 3);
    }
}

///// MPE (MIDI Polyphonic Expression)

uint8_t amy_mpe_is_member_channel(uint8_t channel) {
    mpe_state_t *mpe = &amy_global.mpe;
    if (mpe->num_members == 0 || channel < 1 || channel > 16) return 0;
    if (mpe->master_channel == 16)  // Upper zone: members descend from 15.
        return (channel >= 16 - mpe->num_members) && (channel <= 15);
    // Lower zone: members ascend from master+1.
    return (channel >= mpe->master_channel + 1) && (channel <= mpe->master_channel + mpe->num_members);
}

// The synth that should handle a message on this channel: the zone master's
// synth for member channels, otherwise the channel's own synth.
uint8_t amy_mpe_synth_for_channel(uint8_t channel) {
    if (amy_mpe_is_member_channel(channel)) return amy_global.mpe.master_channel;
    return channel;
}

void amy_mpe_config(uint8_t master_channel, int num_members, float bend_range) {
    mpe_state_t *mpe = &amy_global.mpe;
    if (num_members < 0) num_members = 0;
    if (num_members > 15) num_members = 15;
    if (master_channel < 1 || master_channel > 16) {
        // Callers pass the synth number as the zone master (parse.c iE routes
        // through e->synth), and synths can be numbered past 16. A master
        // outside 1-16 can never match a MIDI channel, so storing it would
        // just leave a zone that silently never fires -- reject it instead.
        fprintf(stderr, "amy_mpe_config: master channel %d out of range 1-16; ignoring\n",
                master_channel);
        return;
    }
    mpe->master_channel = master_channel;
    mpe->num_members = num_members;
    if (AMY_IS_SET(bend_range) && bend_range > 0) mpe->member_bend_range = bend_range;
    for (int ch = 0; ch < 17; ++ch) {
        mpe->channel_bend[ch] = 0;
        mpe->channel_pressure[ch] = 0;
        mpe->channel_timbre[ch] = 0;
    }
    //fprintf(stderr, "MPE: master %d members %d bend range %.1f\n", mpe->master_channel, mpe->num_members, mpe->member_bend_range);
}

void amy_mpe_reset(void) {
    mpe_state_t *mpe = &amy_global.mpe;
    mpe->num_members = 0;
    mpe->master_channel = 1;
    mpe->member_bend_range = MPE_DEFAULT_MEMBER_BEND_RANGE;
    for (int ch = 0; ch < 17; ++ch) {
        mpe->channel_bend[ch] = 0;
        mpe->channel_pressure[ch] = 0;
        mpe->channel_timbre[ch] = 0;
        mpe->rpn_msb[ch] = 0x7F;  // Null RPN.
        mpe->rpn_lsb[ch] = 0x7F;
    }
}

void amy_received_control_change(uint8_t channel, uint8_t control, uint8_t value, uint32_t time) {
    mpe_state_t *mpe = &amy_global.mpe;
    if (control == 0) {
        // Bank select coarse.
        instrument_set_bank_number(channel, value);
    } else if (control == 74 && amy_mpe_is_member_channel(channel)) {
        // MPE per-note timbre ("slide").
        mpe->channel_timbre[channel] = (float)value / 127.0f;
    } else if (control == 101) {
        mpe->rpn_msb[channel] = value;
    } else if (control == 100) {
        mpe->rpn_lsb[channel] = value;
    } else if (control == 6) {
        // Data Entry MSB for the currently-selected RPN.
        if (mpe->rpn_msb[channel] == 0 && mpe->rpn_lsb[channel] == 6
            && (channel == 1 || channel == 16)) {
            // MPE Configuration Message: sets/clears the zone whose master is this channel.
            amy_mpe_config(channel, value, AMY_UNSET_FLOAT);
        } else if (mpe->rpn_msb[channel] == 0 && mpe->rpn_lsb[channel] == 0
                   && amy_mpe_is_member_channel(channel)) {
            // Pitch bend sensitivity on a member channel sets the zone's member bend range.
            // Spec deviation: the MPE spec also defines RPN 0 on the MASTER
            // channel (master-channel bend range); we deliberately leave that
            // to AMY's global pitch_bend handling and only honor member-channel
            // sensitivity here.
            mpe->member_bend_range = (float)value;
        }
    }
}

void amy_received_program_change(uint8_t channel, uint8_t program, uint32_t time) {
    amy_event e = amy_default_event();
    e.time = time;
    e.synth = channel;
    e.note_source_channel = channel;
    // MIDI patches are in blocks of 128, potentially set by an earlier CC0.
    int bank_number = instrument_bank_number(channel);
    if (bank_number < 0) {
        // If the bank hasn't been set, stay within the block of 128 of the current patch
        // (so e.g. DX7 voices remain DX7).
        bank_number = (instrument_get_patch_number(e.synth) & 0xFF80) >> 7;
        // Banks 0 (Juno, patches 0-127) and 1 (DX7, 128-255) are full 128-patch
        // banks, and bank 3 (384+) is the Gamma9001 drum kit bank -- a synth
        // sitting on a drum kit patch should stay in the kit bank so a bare PC
        // switches kits.  Patches 256-383 (additive piano, AMYboard Web Editor
        // base, reserved space) infer bank 2, whose PC targets are almost all
        // undefined and silence the board (issue #758), so fall back to bank 0
        // (Juno) only for those.
        if (bank_number == 2) bank_number = 0;
    }
    e.patch_number = program + 128 * bank_number;
    //if (channel != AMY_MIDI_CHANNEL_DRUMS) {  // What would that even mean?
        amy_add_event(&e);
    //}
}

void amy_received_pedal(uint8_t channel, uint8_t value, uint32_t time) {
    amy_event e = amy_default_event();
    e.time = time;
    e.synth = channel;
    e.note_source_channel = channel;
    e.pedal = value;
    amy_add_event(&e);
}

void amy_received_all_notes_off(uint8_t channel, uint32_t time) {
    amy_event e = amy_default_event();
    e.time = time;
    e.synth = channel;
    e.note_source_channel = channel;
    // All notes off is indicated by vel = 0 and note = 0
    e.velocity = 0;
    e.midi_note = 0;
    amy_add_event(&e);
}

void amy_received_pitch_bend(uint8_t channel, uint8_t low_byte, uint8_t high_byte, uint32_t time) {
    amy_event e = amy_default_event();
    e.time = time;
    // Currently, pitch bend is global and not applied per-channel, but preserve the info.
    e.synth = channel;
    e.note_source_channel = channel;
    // The integer range is -8192 to 8191, the float range is -1/6th to +1/6th,
    // units are octaves, so +/- 2 semitones.
    e.pitch_bend = ((float)(((int)((high_byte << 7) | low_byte)) - 8192)) / (6.0f * 8192.0f);
    amy_add_event(&e);
}

// I'm called when we get a fully formed MIDI message from any interface -- usb, gadget, uart, mac, and either sysex or normal
void amy_event_midi_message_received(uint8_t * data, uint32_t len, uint8_t sysex, uint32_t time) {
    if(!sysex) {
        uint8_t status_byte = data[0];
        uint8_t status = status_byte & 0xF0;
        uint8_t channel = status_byte & 0x0F;
        // Do the AMY instrument things here
        // Pedal and all-notes-off on an MPE member channel act on the zone's synth.
        if(status == 0xB0 && data[1] == 0x40) amy_received_pedal(amy_mpe_synth_for_channel(channel+1), data[2], time);
        else if(status == 0xB0 && data[1] == 0x7B) amy_received_all_notes_off(amy_mpe_synth_for_channel(channel+1), time);
        else if(status == 0XB0) amy_received_control_change(channel+1, data[1], data[2], time);
        else if(status == 0xC0) amy_received_program_change(channel+1, data[1], time);
        else if(status == 0xD0) {
            // Channel pressure: per-note pressure on MPE member channels (read as ext0).
            if (amy_mpe_is_member_channel(channel+1))
                amy_global.mpe.channel_pressure[channel+1] = (float)data[1] / 127.0f;
        }
        else if(status == 0xE0) {
            if (amy_mpe_is_member_channel(channel+1)) {
                // MPE per-note pitch bend: applies only to this channel's notes,
                // scaled by the zone's bend range (semitones -> octaves).
                int bend = (int)((data[2] << 7) | data[1]) - 8192;
                amy_global.mpe.channel_bend[channel+1] =
                    ((float)bend / 8192.0f) * (amy_global.mpe.member_bend_range / 12.0f);
            } else {
                amy_received_pitch_bend(channel+1, data[1], data[2], time);
            }
        }
        // MIDI transport (Start/Stop) only drives the sequencer when the user
        // has opted into external sync; otherwise a connected DAW's transport
        // would hijack the AMYboard's own internal sequence.
        else if(status_byte == 0xFA) { if(external_midi_sync_enabled) sequencer_midi_start(); }
        else if(status_byte == 0xFC) { if(external_midi_sync_enabled) sequencer_midi_stop(); }
    }
    midi_msg_handler(data, len, sysex, time);

    // Also send the external hooks if set
    if(amy_global.config.amy_external_midi_input_hook != NULL) {
        amy_global.config.amy_external_midi_input_hook(data, len, sysex);
    }

    // Update web MIDI out hook if set
    #ifdef __EMSCRIPTEN__
    EM_ASM(
        if(typeof amy_external_midi_input_js_hook === 'function') {
            amy_external_midi_input_js_hook(HEAPU8.subarray($0, $0+$1), $1, $2);
        }, data, len, sysex);
    #endif
}


void midi_clock_received() {
    if (external_midi_sync_enabled) {
        sequencer_midi_clock_tick();
    }
}


/*
    3 0x8X - note off    |   note number    |  velocity 
    3 0x9X - note on     |   note number    |  velocity
    3 0xAX - Paftertouch |   note number    |  pressure
    3 0xBX - CC          |   controller #   |  value 
    2 0xCX - PC          |   program        |  XXXX
    2 0xDX - Caftertouch |   pressure       |  XXXX
    3 0xEX - pitch bend  |    LSB.          |  MSB
    X 0xF0  - sysex start|  ... wait until F7
    2 0xF1  - time code  | data
    3 0xF2 song pos      | lsb              | msb
    2 0xF3 song sel      | data
    1 0xF4 reserved      | XXXX
    1 0xF5 reserved.     | XXXX
    1 0xF6 tune request  | XXXX
    X 0xF7 end of sysex  | XXXX
    1 0xF8 timing clock  | XXXX
    1 0xF9 reserved.     | XXXX
    1 0xFA start         | XXXX
    1 0xFB continue      | XXXX
    1 0xFC stop          | XXXX
    1 0xFD reserved      | XXXX
    1 0xFE sensing       | XXXX
    1 0xFF reset         | XXXX
*/

uint16_t sysex_len = 0;
// Latched when a sysex overruns sysex_buffer. The message is then dropped at
// its closing F7 rather than truncated -- see convert_midi_bytes_to_messages().
static uint8_t sysex_overflow = 0;
#if defined(TULIP) || defined(AMYBOARD)
extern const mp_obj_fun_builtin_var_t tulip_amy_send_sysex_obj;
#endif
uint8_t * sysex_buffer = NULL;
// Ring buffer of sysex payload snapshots for deferred MicroPython processing.
// parse_sysex() copies each payload into a separate slot so that a new sysex
// arriving before the scheduled callback fires doesn't overwrite the previous
// message. This matters when the sketch's loop() is CPU-heavy and the
// mp_sched callback is delayed.
char * sysex_message_copies[SYSEX_COPY_SLOTS_DIM];  // dim floored at 1 for MSVC; loops use SYSEX_COPY_SLOTS (0 off-board)
uint8_t sysex_copy_write_idx = 0;  // MIDI task writes here
uint8_t sysex_copy_read_idx = 0;   // MP callback reads here
void parse_sysex() {
    uint32_t time = AMY_UNSET_VALUE(time);
    if(sysex_len>3) {
        // let's use 0x00 0x03 0x45 for SPSS
        if(sysex_buffer[0] == 0x00 && sysex_buffer[1] == 0x03 && sysex_buffer[2] == 0x45) {
            sysex_buffer[sysex_len] = 0;
            // zB[mode]: Reboot. Handled in pure C — no mp_sched_schedule
            // needed, works even when loop() is hogging the scheduler.
            //   zBZ  / zB0Z — bootloader mode (skip sketch.py)
            //   zB1Z       — normal reboot (run sketch.py)
            //   zB2Z       — ROM download/flash mode
            if (sysex_len > 4 && sysex_buffer[3] == 'z' && sysex_buffer[4] == 'B') {
                uint8_t mode = 0;
                if (sysex_len > 5 && sysex_buffer[5] >= '0' && sysex_buffer[5] <= '9') {
                    mode = sysex_buffer[5] - '0';
                }
                if (amy_global.config.amy_external_reboot_hook) {
                    amy_global.config.amy_external_reboot_hook(mode);
                }
                sysex_len = 0;
                return;
            }
            // zI: Ping/identity — reply with a short sysex so the web side
            // knows the board is alive and ready. Pure C, no scheduler needed.
            if (sysex_len > 4 && sysex_buffer[3] == 'z' && sysex_buffer[4] == 'I') {
                uint8_t frame[] = { 0xF0, 0x00, 0x03, 0x45, 'O', 'K', 0xF7 };
                midi_out(frame, sizeof(frame));
                sysex_len = 0;
                return;
            }
            // For Micropython hosted systems, we run MIDI on a separate "thread" (task)
            // than MP, so just calling amy_send_message here can fail if it needs to access
            // underlying MP resources. So we schedule it to run in the MP main loop instead.
            // Each message gets its own ring-buffer slot so a fast-arriving next sysex
            // doesn't overwrite an unprocessed message.
            #if defined(TULIP) || defined(AMYBOARD)
            {
                // NOTE: ACK is sent from the callback (tulip_amy_send_sysex)
                // AFTER the message is processed, not here in parse_sysex.
                // This ensures the sender only proceeds once the ring buffer
                // slot has been drained — receiving the ACK here would just
                // confirm receipt, allowing the ring buffer to overflow if
                // callbacks are slow.
                //
                // Do NOT stop the sequencer here. We used to do that to
                // prevent loop() from stealing MP scheduler slots during
                // large file transfers, but it caused sequencer_midi_start
                // to reset next_amy_tick_us on every sysex, effectively
                // speeding up the sequencer when knob updates arrive.
                char *slot = sysex_message_copies[sysex_copy_write_idx];
                if(slot) {
                    uint16_t payload_len = sysex_len - 3;
                    memcpy(slot, (char*)sysex_buffer + 3, payload_len);
                    slot[payload_len] = '\0';
                    sysex_copy_write_idx = (sysex_copy_write_idx + 1) % SYSEX_COPY_SLOTS;
                }
                mp_sched_schedule(MP_OBJ_FROM_PTR(&tulip_amy_send_sysex_obj), mp_const_none);
            }
            #else
            amy_add_message((char*)sysex_buffer+3);
            #endif
            sysex_len = 0; // handled
        } else {
           amy_event_midi_message_received(sysex_buffer, sysex_len, 1, time);
        }
    }
}

// The parser core lives in amy_midi_parse.h so tulipcc's host concurrency
// harness (tests/midi_input/) can compile the EXACT same code natively and
// differential-test it. These macros bind it to the real firmware sinks.
// Emitted messages carry an UNSET time, exactly like the old inline parser.
#define AMY_MIDI_PARSE_EMIT(d, l) do { \
        uint32_t t_ = 0; t_ = AMY_UNSET_VALUE(t_); \
        amy_event_midi_message_received((d), (l), 0, t_); \
    } while (0)
#define AMY_MIDI_PARSE_CLOCK()      midi_clock_received()
#define AMY_MIDI_PARSE_SYSEX_DONE() parse_sysex()
#define AMY_MIDI_PARSE_LOG(...)     fprintf(stderr, __VA_ARGS__)
#ifdef AMY_MIDI_MPSC
#define AMY_MIDI_PARSE_MPSC 1
#endif
#include "amy_midi_parse.h"

// One context per byte stream (see amy_midi_source_t). A context is only
// ever advanced by one task: in the default funnel build that task is the
// AMY MIDI task for ALL of them (foreign sources hand their bytes over via
// amy_midi_inject); in the experimental AMY_MIDI_MPSC build each producer
// task parses its own context in place.
static midi_stream_parser_t midi_parsers[AMY_MIDI_SOURCE_COUNT];

void convert_midi_bytes_to_messages(uint8_t * data, size_t len, uint8_t usb) {
    // Single-stream compatibility entry -- and the UART/DIN path on ESP.
    // Platforms with exactly one MIDI byte source (pico, teensy, macos, web)
    // parse in the UART context. A SECOND concurrent source must NOT call
    // this: on ESP it goes through amy_midi_inject() so both the parser
    // context and the executing task match the stream.
    midi_parse_stream(&midi_parsers[AMY_MIDI_SOURCE_UART], data, len, usb);
}

// This is used for web emscripten hooks + external linkers of AMY
// set from_web_or_usb to 1 if this is a 4 packet type interface -- WebMIDI or USB MIDI gadget/host, 0 otherwise
void amy_process_single_midi_byte(uint8_t byte, uint8_t from_web_or_usb) {
    uint8_t data[1];
    data[0] = byte;
    convert_midi_bytes_to_messages(data, 1, from_web_or_usb); 
}

// for external programs to send MIDI data OUT using AMY
void amy_external_midi_output(uint8_t * data, uint32_t len) {
    midi_out(data, len);
}


///// Per platform MIDI in and out stuff
///////////////////////////////////////////////


#if (defined __EMSCRIPTEN__)
void run_midi() {
    // do nothing, this is all done with callbacks
}

void stop_midi() {
}

void midi_out(uint8_t * bytes, uint16_t len) {
    EM_ASM(
            if(midiOutputDevice != null) {
                midiOutputDevice.send(HEAPU8.subarray($0, $0 + $1));
            }, bytes, len
        );
}
#endif

#if !defined(MACOS) && !defined(__EMSCRIPTEN__) // this code is for NOT macos desktop , which is in macos_midi.m

// "run_midi" sets up MIDI on MCU platforms


#if (defined ESP_PLATFORM)
TaskHandle_t midi_handle;

// ---- Cross-task MIDI input funnel (default build) ----------------------
// Everything downstream of the stream parser -- amy_add_event(), the MPE
// globals, sequencer transport, parse_sysex()'s mp_sched hop, and Tulip's
// last_midi ring (whose writer must be exactly ONE task for its SPSC
// release/acquire discipline to mean anything) -- is single-threaded by
// design. The UART already parses on the AMY MIDI task; the Tulip USB host
// task (core 1) and MicroPython's tulip.midi_local() (MP task, core 1) used
// to call the parser directly from their own tasks, racing all of it. They
// now enqueue raw bytes here and the MIDI task drains the queue in
// esp_poll_midi(): the single-writer invariant is restored rather than
// defended against.
//
// Latency cost of the hop: the MIDI task's UART poll blocks at most one
// FreeRTOS tick (1ms at CONFIG_FREERTOS_HZ=1000), so a funneled message
// waits <=~1ms before parsing -- the same order as ONE message's own wire
// time on DIN (3 bytes at 31250 baud = 960us) and far below perceptibility.
//
// SRAM cost: the queue struct and storage are allocated with
// ram_caps_sysex (SPIRAM on Tulip) -- internal SRAM is designed-full on the
// deck (28 bytes free, measured) and none of it may be spent here. Do NOT
// swap this for xQueueCreate(): that puts the storage in internal heap.
#ifndef AMY_MIDI_MPSC
typedef struct {
    uint8_t source;   // amy_midi_source_t: selects the parser context
    uint8_t usb;      // packetized-source flag for the parser
    uint8_t len;      // 1..3 bytes used
    uint8_t bytes[3];
} midi_inject_item_t;

// 128 messages of backlog. The MIDI task drains the whole queue at least
// once per ms, so overflow means >128 messages/ms sustained -- beyond what
// MIDI hardware can produce; a full queue really means the MIDI task is
// wedged, which the drop counter below makes visible instead of hiding.
#define MIDI_INJECT_QUEUE_DEPTH 128
static QueueHandle_t midi_inject_queue = NULL;
#endif
volatile uint32_t amy_midi_inject_drops = 0;

static void midi_inject_report_drop(const char *why) {
    uint32_t n = ++amy_midi_inject_drops;
    // First drop logs immediately, then every power of two: loud enough to
    // see the first failure the moment it happens, quiet enough that a
    // wedged MIDI task doesn't turn the console into its own flood.
    if ((n & (n - 1)) == 0)
        fprintf(stderr, "amy_midi_inject: %s -- %u MIDI messages dropped so far\n",
                why, (unsigned)n);
}

#ifdef AMY_MIDI_MPSC
// ---- Experimental MPSC build: parse in the CALLER's task ----------------
// Each source owns its parser context, so per-stream state is race-free by
// construction; the shared sysex buffer is claimed by atomic CAS (see
// amy_midi_parse.h); Tulip's last_midi ring must then be built MPSC too
// (tulip/shared/midi_in_ring.h, same gate). What this build does NOT
// serialize: the semantic layer under amy_event_midi_message_received()
// (MPE state, bank select, sequencer transport, midi_msg_handler) now runs
// concurrently from up to three tasks. See the harness verdict notes before
// shipping this.
void amy_midi_inject(amy_midi_source_t source, const uint8_t *bytes, uint16_t len) {
    if (source >= AMY_MIDI_SOURCE_COUNT) { midi_inject_report_drop("bad source"); return; }
    uint8_t usb = (source == AMY_MIDI_SOURCE_USB_HOST || source == AMY_MIDI_SOURCE_GADGET);
    midi_parse_stream(&midi_parsers[source], (uint8_t *)bytes, len, usb);
}
#else
void amy_midi_inject(amy_midi_source_t source, const uint8_t *bytes, uint16_t len) {
    if (midi_inject_queue == NULL) {
        // Not created yet, or its allocation failed at boot (already
        // reported, loudly, by run_midi). Parsing here instead would be
        // exactly the cross-task race this funnel exists to remove -- so
        // drop, count, and say so.
        midi_inject_report_drop("no queue");
        return;
    }
    uint8_t usb = (source == AMY_MIDI_SOURCE_USB_HOST || source == AMY_MIDI_SOURCE_GADGET);
    while (len) {
        midi_inject_item_t it;
        it.source = (uint8_t)source;
        it.usb = usb;
        it.len = (len > 3) ? 3 : (uint8_t)len;
        for (uint8_t i = 0; i < it.len; i++) it.bytes[i] = bytes[i];
        // Never block the caller: the USB event task and the MP task both
        // have other work, and a wedged MIDI task must show up as counted
        // drops, not as a second wedged task. Drop-NEWEST, matching the
        // last_midi ring's discipline. NOTE: dropping mid-sysex can strand
        // the shared sysex buffer with this source as owner until this
        // source's next status byte -- acceptable only because a full queue
        // already means the MIDI pipeline is down, and the drop counter
        // says so.
        if (xQueueSend(midi_inject_queue, &it, 0) != pdTRUE) {
            midi_inject_report_drop("queue full");
            return;  // the rest of this buffer is even newer: drop it too
        }
        bytes += it.len;
        len -= it.len;
    }
}
#endif

int8_t esp_get_uart(int8_t index) {
    if(index==0) return UART_NUM_0;
    if(index==1) return UART_NUM_1;
    if(index==2) return UART_NUM_2;
    return -1;
}
#if defined (AMYBOARD) || defined(AMYBOARD_ARDUINO)
#define TUD_USB_GADGET
#include "tusb.h"
#include "class/midi/midi.h"
#include "class/midi/midi_device.h"
#ifdef AMYBOARD_ARDUINO
#include "usb.h"
#endif

void check_tusb_midi() {
    while ( tud_midi_available() ) {
        uint8_t packet[4];
        tud_midi_packet_read(packet);
        // Own context: this runs on the MIDI task right next to the UART
        // poll, but it is a DIFFERENT byte stream -- sharing the UART's
        // parser context interleaved gadget packets into DIN running status.
        midi_parse_stream(&midi_parsers[AMY_MIDI_SOURCE_GADGET], packet+1, 3, 1);
    }
}
#endif

void esp_init_midi(void) {
    uart_config_t uart_config = {
        .baud_rate = 31250,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
    };

    // Configure UART parameters
    const int uart_num = esp_get_uart(amy_global.config.midi_uart);
    if (!uart_is_driver_installed(uart_num)) {
        ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(uart_num, amy_global.config.midi_out, amy_global.config.midi_in, UART_PIN_NO_CHANGE , UART_PIN_NO_CHANGE ));

        const int uart_buffer_size = (MAX_MIDI_BYTES_TO_PARSE);
        // Install UART driver using an event queue here
        ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size,  0, 0, NULL, 0));

        uart_intr_config_t uart_intr = {
            .intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M
            | UART_RXFIFO_TOUT_INT_ENA_M
            | UART_FRM_ERR_INT_ENA_M
            | UART_RXFIFO_OVF_INT_ENA_M
            | UART_BRK_DET_INT_ENA_M
            | UART_PARITY_ERR_INT_ENA_M,
            .rxfifo_full_thresh = 1,
            .rx_timeout_thresh = 0,
            .txfifo_empty_intr_thresh = 10
        };

        uart_intr_config(uart_num, &uart_intr);
    }
}

void esp_deinit_midi(void) {
    const int uart_num = esp_get_uart(amy_global.config.midi_uart);
    if (uart_is_driver_installed(uart_num)) {
        uart_intr_config_t uart_intr = {
            .intr_enable_mask = 0,
            .rxfifo_full_thresh = 1,
            .rx_timeout_thresh = 0,
            .txfifo_empty_intr_thresh = 10
        };
        uart_intr_config(uart_num, &uart_intr);
        uart_driver_delete(uart_num);
    }
}

void esp_poll_midi(void) {
    const int uart_num = esp_get_uart(amy_global.config.midi_uart);
    uint8_t data[MAX_MIDI_BYTES_TO_PARSE];
    int length = uart_read_bytes(uart_num, data, MAX_MIDI_BYTES_TO_PARSE /*MAX_MIDI_BYTES_PER_MESSAGE*MIDI_QUEUE_DEPTH*/, 1/portTICK_PERIOD_MS);
    if(length > 0) {
        convert_midi_bytes_to_messages(data,length,0);
    }
#ifndef AMY_MIDI_MPSC
    // Drain the cross-task funnel (USB-host MIDI, tulip.midi_local). Each
    // item parses in ITS source's context, so streams can't tear each other
    // even though one task parses them all. Budgeted to one queue-depth per
    // pass so a producer flood can never starve the UART read above -- at
    // this drain rate (>= once per ms) the budget is unreachable in normal
    // operation.
    if (midi_inject_queue != NULL) {
        midi_inject_item_t it;
        int budget = MIDI_INJECT_QUEUE_DEPTH;
        while (budget-- > 0 && xQueueReceive(midi_inject_queue, &it, 0) == pdTRUE) {
            uint8_t src = (it.source < AMY_MIDI_SOURCE_COUNT) ? it.source
                                                              : AMY_MIDI_SOURCE_LOCAL;
            midi_parse_stream(&midi_parsers[src], it.bytes, it.len, it.usb);
        }
    }
#endif
}

void run_midi_task() {

    while(1) {
        esp_poll_midi();
        #if defined (AMYBOARD) || defined(AMYBOARD_ARDUINO)
        check_tusb_midi();
        #endif
    } // end loop forever
}

void run_midi() {
#ifndef AMY_MIDI_MPSC
    // Create the inject funnel BEFORE any producer can run. Created once and
    // deliberately never destroyed (see stop_midi): the producers live on
    // OTHER tasks and cannot be fenced away from a freed queue -- a stale
    // xQueueSend into freed memory is a heap smash. Cost: ~850 bytes of
    // ram_caps_sysex (SPIRAM on Tulip), zero internal SRAM, held for life.
    if (midi_inject_queue == NULL) {
        StaticQueue_t *qs = (StaticQueue_t *)malloc_caps(sizeof(StaticQueue_t),
                                                amy_global.config.ram_caps_sysex);
        uint8_t *storage = (uint8_t *)malloc_caps(
                MIDI_INJECT_QUEUE_DEPTH * sizeof(midi_inject_item_t),
                amy_global.config.ram_caps_sysex);
        if (qs != NULL && storage != NULL) {
            midi_inject_queue = xQueueCreateStatic(MIDI_INJECT_QUEUE_DEPTH,
                    sizeof(midi_inject_item_t), storage, qs);
        }
        if (midi_inject_queue == NULL) {
            // Boot-time, once, unmissable: without the queue every USB-MIDI
            // and midi_local message is dropped (and counted -- see
            // amy_midi_inject_drops / tulip.midi_in_drops()).
            fprintf(stderr, "run_midi: FAILED to allocate the MIDI inject queue "
                    "(%u bytes) -- USB and midi_local input will be DROPPED\n",
                    (unsigned)(sizeof(StaticQueue_t)
                               + MIDI_INJECT_QUEUE_DEPTH * sizeof(midi_inject_item_t)));
        }
    }
#endif
    if (sysex_buffer == NULL) {
        sysex_buffer = malloc_caps(MAX_SYSEX_BYTES, amy_global.config.ram_caps_sysex);
        for (int i = 0; i < SYSEX_COPY_SLOTS; i++) {
            sysex_message_copies[i] = malloc_caps(MAX_SYSEX_BYTES, amy_global.config.ram_caps_sysex);
        }
        #if defined(AMYBOARD_ARDUINO)
        // Initialize TinyUSB with amy's MIDI+CDC descriptors before starting MIDI polling
        if(amy_global.config.midi & AMY_MIDI_IS_USB_GADGET) {
            amy_arduino_usb_setup();
        }
        #endif
        if(amy_global.config.midi & AMY_MIDI_IS_UART) {
            esp_init_midi();
            if (amy_global.config.platform.multithread) {
                xTaskCreatePinnedToCore(run_midi_task, MIDI_TASK_NAME, (MIDI_TASK_STACK_SIZE) / sizeof(StackType_t), NULL, MIDI_TASK_PRIORITY, &midi_handle, MIDI_TASK_COREID);
            }  // otherwise esp_poll_midi is called from amy_update_tasks()
        }
    }
}

void stop_midi() {
    if(amy_global.config.midi & AMY_MIDI_IS_UART) {
        if (amy_global.config.platform.multithread) {
            vTaskDelete(midi_handle);
        }
        esp_deinit_midi();
    }
    free(sysex_buffer);
    sysex_buffer = NULL;
    for (int i = 0; i < SYSEX_COPY_SLOTS; i++) {
        free(sysex_message_copies[i]);
        sysex_message_copies[i] = NULL;
    }
}




#endif

#if (defined ARDUINO_ARCH_RP2040) || (defined ARDUINO_ARCH_RP2350)

uart_inst_t * rp_get_uart(int8_t index) {
    if(index==0) return uart0;
    if(index==1) return uart1;
    return NULL;
}

// RX interrupt handler
void on_pico_uart_rx() {
    const int midi_buffer_size = 16;
    uint8_t bytes[midi_buffer_size];
    uint8_t i = 0;
    while (uart_is_readable(rp_get_uart(amy_global.config.midi_uart)) && i < midi_buffer_size) {
        uart_read_blocking (rp_get_uart(amy_global.config.midi_uart), bytes + i, 1);
        i++;
    }
    //if (i >= midi_buffer_size)
    //    fprintf(stderr, "midi_buffer_size %d of %d\n", i, midi_buffer_size);
    convert_midi_bytes_to_messages(bytes,i,0);
}

extern void pico_setup_midi();
extern void pico_teardown_midi();

void run_midi() {
    if (sysex_buffer == NULL) {
        // sysex_buffer is allocated on every platform; the SYSEX_COPY_SLOTS
        // backup ring is sized 0 (no alloc) on Pico, so this loop is a no-op here.
        sysex_buffer = malloc_caps(MAX_SYSEX_BYTES, amy_global.config.ram_caps_sysex);
        for (int i = 0; i < SYSEX_COPY_SLOTS; i++) {
            sysex_message_copies[i] = malloc_caps(MAX_SYSEX_BYTES, amy_global.config.ram_caps_sysex);
        }
        if(amy_global.config.midi & AMY_MIDI_IS_UART) {
            uart_init(rp_get_uart(amy_global.config.midi_uart), 31250);
            gpio_set_function(amy_global.config.midi_out, UART_FUNCSEL_NUM(rp_get_uart(amy_global.config.midi_uart), amy_global.config.midi_out));
            gpio_set_function(amy_global.config.midi_in, UART_FUNCSEL_NUM(rp_get_uart(amy_global.config.midi_uart), amy_global.config.midi_in));
            uart_set_hw_flow(rp_get_uart(amy_global.config.midi_uart), false, false);
            uart_set_format(rp_get_uart(amy_global.config.midi_uart), 8, 1, UART_PARITY_NONE);
            uart_set_fifo_enabled(rp_get_uart(amy_global.config.midi_uart), true);
        } else if(amy_global.config.midi & AMY_MIDI_IS_USB_GADGET) {
            pico_setup_midi();
        }
    }
}

void stop_midi() {
    if (sysex_buffer) {
        if(amy_global.config.midi & AMY_MIDI_IS_UART) {
            uart_set_fifo_enabled(rp_get_uart(amy_global.config.midi_uart), false);
            uart_deinit(rp_get_uart(amy_global.config.midi_uart));
        } else if(amy_global.config.midi & AMY_MIDI_IS_USB_GADGET) {
            pico_teardown_midi();
        }
        free(sysex_buffer);
        sysex_buffer = NULL;
        for (int i = 0; i < SYSEX_COPY_SLOTS; i++) {
            free(sysex_message_copies[i]);
            sysex_message_copies[i] = NULL;
        }
    }
}

#endif

#ifdef __IMXRT1062__
extern void teensy_start_midi();

void run_midi() {
    if(amy_global.config.midi & AMY_MIDI_IS_UART) teensy_start_midi();
}

void stop_midi() {
}
#endif


#ifdef __linux__
void stop_midi() {
}

void run_midi() {
    //fprintf(stderr, "no MIDI support on linux yet\n");
}
#endif

#ifdef AMY_DAISY
// Daisy seed
void run_midi() {
    // MIDI handled in main.
}
#endif

#ifdef _WIN32
void stop_midi() {
}

void run_midi() {
}
#endif

void midi_out(uint8_t * bytes, uint16_t len) {

// Is there USB gadget midi? Send it
#if defined TUD_USB_GADGET
    if(amy_global.config.midi & AMY_MIDI_IS_USB_GADGET) {
        // tud_midi_stream_write uses a small FIFO (e.g. 64 bytes). For long
        // messages (e.g. zD sysex dumps) we must loop and yield until the
        // USB task flushes the FIFO, otherwise bytes are silently dropped.
        if (len > 64) fprintf(stderr, "midi_out: USB gadget, want to send %d bytes\n", (int)len);
        uint32_t sent = 0;
        int stall_ticks = 0;
        while (sent < len) {
            uint32_t n = tud_midi_stream_write(0, bytes + sent, len - sent);
            if (n == 0) {
#if defined(TULIP) || defined(AMYBOARD)
                // We're running on the MP main thread; the USB task also runs
                // here, so vTaskDelay alone won't drain the FIFO. Pump USB
                // events directly.
                mp_usbd_task();
#endif
#if defined ESP_PLATFORM
                vTaskDelay(pdMS_TO_TICKS(1));
#endif
                if (++stall_ticks > 1000) {
                    fprintf(stderr, "midi_out: STALLED after %u of %u bytes\n",
                            (unsigned)sent, (unsigned)len);
                    break;
                }
            } else {
                stall_ticks = 0;
            }
            sent += n;
        }
        if (len > 64) fprintf(stderr, "midi_out: USB gadget sent %u/%u bytes\n",
                              (unsigned)sent, (unsigned)len);
    }
#endif

// Also do UART midi on supported platforms
#if defined ESP_PLATFORM
    if(amy_global.config.midi & AMY_MIDI_IS_UART) {
        uart_write_bytes(esp_get_uart(amy_global.config.midi_uart), bytes, len);
    }
#elif (defined ARDUINO_ARCH_RP2040) || (defined ARDUINO_ARCH_RP2350)
    if(amy_global.config.midi & AMY_MIDI_IS_UART) uart_write_blocking(rp_get_uart(amy_global.config.midi_uart), bytes, len);
#else
    // teensy
    // linux
#endif

}

#endif // check for macos desktop 
