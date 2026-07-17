// midi.h

#ifndef __MIDI_H
#define __MIDI_H

#ifdef ESP_PLATFORM 
#include "driver/uart.h"
#include "soc/uart_reg.h"
#include "esp_task.h"
#else
// virtualmidi Cocoa stubs
#endif
#define MIDI_SLOTS 4

void convert_midi_bytes_to_messages(uint8_t * data, size_t len, uint8_t usb);
void amy_process_single_midi_byte(uint8_t byte, uint8_t from_web_or_usb);
void amy_external_midi_output(uint8_t * data, uint32_t len);
void amy_external_midi_sync(uint8_t enabled);

// One parser context per MIDI byte STREAM. The stream parser keeps state
// across calls (running status, collected data bytes, inside-sysex), so two
// byte streams must never share a context -- and, in the default build, the
// parser plus everything downstream of it may only run on the AMY MIDI task
// (see amy_midi_inject). Spec and rationale: amy_midi_parse.h.
typedef enum {
    AMY_MIDI_SOURCE_UART = 0,  // DIN/UART, parsed directly by the MIDI task
                               // (also the context behind the single-source
                               // compatibility entry convert_midi_bytes_to_messages)
    AMY_MIDI_SOURCE_USB_HOST,  // Tulip USB-host MIDI (packetized: usb flag 1)
    AMY_MIDI_SOURCE_LOCAL,     // tulip.midi_local() from MicroPython (byte stream)
    AMY_MIDI_SOURCE_GADGET,    // TinyUSB gadget MIDI (packetized), AMYBOARD
    AMY_MIDI_SOURCE_COUNT
} amy_midi_source_t;

#ifdef ESP_PLATFORM
// Hand bytes from a task that is NOT the AMY MIDI task to the MIDI input
// path. Default build: enqueues into a small funnel queue drained by the
// MIDI task within ~1ms -- never blocks; on a full/missing queue the bytes
// are dropped LOUDLY (counted in amy_midi_inject_drops, logged on a
// power-of-two schedule, readable from Python as tulip.midi_in_drops()).
// AMY_MIDI_MPSC build: parses immediately in the caller's task using the
// source's own context.
void amy_midi_inject(amy_midi_source_t source, const uint8_t *bytes, uint16_t len);
extern volatile uint32_t amy_midi_inject_drops;
#endif


#define MAX_MIDI_BYTES_TO_PARSE 1024
#define MAX_MIDI_BYTES_PER_MESSAGE 3
#define MIDI_QUEUE_DEPTH 1024
#define MAX_SYSEX_BYTES (16384)
extern uint8_t *sysex_buffer;
// Every platform allocates the single sysex_buffer above. The sysex copy-slot
// ring buffer below holds backup snapshots so a fast-arriving message isn't lost
// while a previous one waits for the deferred MicroPython mp_sched callback.
// Only AMYBOARD creates and reads these slots (see parse_sysex()): the ring
// exists for the SPSS z* sketch-transfer bursts over USB-gadget MIDI, which
// only AMYboard receives. Size the ring to 0 everywhere else so we don't
// malloc SYSEX_COPY_SLOTS x MAX_SYSEX_BYTES (32 x 16KB = 512KB): that alone
// would exhaust e.g. the rp2350's 520KB of SRAM, and on Tulip ESP32-S3 it ate
// a third of the free SPIRAM. With 0 slots, parse_sysex() finds a NULL slot
// and the scheduled callback ACKs SPSS messages without processing them;
// non-SPSS sysex is unaffected.
#if defined(AMYBOARD)
#define SYSEX_COPY_SLOTS 32
#else
#define SYSEX_COPY_SLOTS 0
#endif
// The alloc/free loops use SYSEX_COPY_SLOTS directly (0 => no slots off-board),
// but the array dimension must be >= 1: MSVC (the Godot Windows build includes
// this header transitively via amy.h) rejects a zero-length array with C2466,
// unlike the GCC/Clang extension. Floor the dimension at 1; that lone slot is
// never touched off-board because the loops iterate 0 times.
#define SYSEX_COPY_SLOTS_DIM ((SYSEX_COPY_SLOTS) > 0 ? (SYSEX_COPY_SLOTS) : 1)
extern char *sysex_message_copies[SYSEX_COPY_SLOTS_DIM];
extern uint8_t sysex_copy_write_idx;
extern uint8_t sysex_copy_read_idx;
extern uint16_t sysex_len;
extern void parse_sysex();
extern uint8_t last_midi[MIDI_QUEUE_DEPTH][MAX_MIDI_BYTES_PER_MESSAGE];
extern uint8_t last_midi_len[MIDI_QUEUE_DEPTH];
// volatile: ring shared across cores (reader = the host MP task); the
// defining declarations on Tulip are volatile and these must agree or the
// build fails on conflicting qualifiers.
// The midi_queue_head/midi_queue_tail cursors are NOT declared here: amy never
// touches them (they are tulip's ring), and tulip's modtulip.c includes this
// header only `#ifndef __EMSCRIPTEN__` -- so declaring them here made them
// invisible to the web build while the code using them still compiled. They
// now live in tulipcc tulip/shared/midi_in_ring.h, which both their definer
// and their user include unconditionally.

void midi_out(uint8_t * bytes, uint16_t len);
void midi_local(uint8_t * bytes, uint16_t len);
void amy_send_midi_note_off(uint16_t osc);
void amy_send_midi_note_on(uint16_t osc);
// For pyamy inject_midi
void amy_event_midi_message_received(uint8_t * data, uint32_t len, uint8_t sysex, uint32_t time);

#ifdef ESP_PLATFORM
#define MIDI_TASK_COREID (0)
#define MIDI_TASK_STACK_SIZE (8 * 1024)
#define MIDI_TASK_NAME      "amy_midi_task"
// BELOW the render tasks (review FW-5): at MAX-2 a sysex burst preempted
// core-0 rendering for its whole parse -- the fill task then blocked on
// the render notify and the entire audio pipeline stalled past the DMA
// ring. One block (5.8ms) of MIDI latency is inaudible; a stalled block
// is not.
#define MIDI_TASK_PRIORITY (ESP_TASK_PRIO_MAX - 4)
#endif

void run_midi();
void stop_midi();
#ifdef MACOS
void *run_midi_macos(void*vargp);
#endif

void check_tusb_midi();
void init_tusb_midi();

#endif // __MIDI_H
