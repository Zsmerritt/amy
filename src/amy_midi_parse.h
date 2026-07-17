// amy_midi_parse.h
// MIDI byte-stream parser CORE, shared verbatim between the firmware
// (amy_midi.c includes it) and the tulipcc host concurrency harness
// (tulipcc tests/midi_input/ includes it with recorder macros). Do not add
// platform includes here; everything environment-specific comes in through
// the macros below, which the including .c file must define FIRST:
//
//   AMY_MIDI_PARSE_EMIT(data, len)  a complete non-sysex message (uint8_t*, int)
//   AMY_MIDI_PARSE_CLOCK()          a 0xF8 timing clock byte
//   AMY_MIDI_PARSE_SYSEX_DONE()     a complete sysex sits in sysex_buffer[0..sysex_len)
//   AMY_MIDI_PARSE_LOG(...)         fprintf(stderr, ...) equivalent (already loud/rare)
//
// and must declare these objects before including:
//   uint8_t *sysex_buffer;  uint16_t sysex_len;  <integer> sysex_overflow;
//   MAX_SYSEX_BYTES
//
// ---------------------------------------------------------------------------
// WHY PER-STREAM CONTEXTS (the spec -- keep this comment true):
//
// The parser keeps state ACROSS calls: the current status byte (MIDI running
// status), which data byte comes next, and whether the stream is inside an
// F0..F7 sysex. That state belongs to a byte STREAM, never to the device as
// a whole. Tulip receives up to three concurrent streams (DIN UART, USB-host
// MIDI, tulip.midi_local() from Python; AMYboard adds USB-gadget). When they
// shared one global context, a data byte from one stream completed a message
// begun by another -- torn messages -- and two sysexes merged into one
// buffer. Guarantees this core provides, per context:
//
//  1. NO TORN MESSAGES: every emitted message's bytes come from exactly one
//     stream, because each stream parses in its own midi_stream_parser_t.
//  2. PER-STREAM FIFO: bytes of one stream are parsed in arrival order
//     (calls for one context must themselves be ordered -- the caller's job:
//     the funnel orders them by queue FIFO; the MPSC build orders them
//     because each context is only ever touched by its one producer task).
//  3. ONE SYSEX AT A TIME: there is a single sysex assembly buffer, so the
//     first F0 claims it and a second stream's overlapping sysex is dropped
//     WHOLE and LOUDLY (once per attempt) -- the same policy a hardware MIDI
//     merger applies, and strictly better than interleaving two payloads.
//
// THREADING: with AMY_MIDI_PARSE_MPSC undefined (the default, "funnel"
// build) every context is parsed by the single AMY MIDI task, and the sysex
// owner arbitration is plain loads/stores. With AMY_MIDI_PARSE_MPSC defined,
// each context is parsed by its own producer task and ownership of the
// shared sysex buffer is claimed with an atomic compare-exchange; the
// context structs themselves still need no atomics because each is
// single-task by construction. Everything DOWNSTREAM of the emit macro must
// then be safe for concurrent callers -- which on Tulip it is only partially
// (see the verdict notes in tulipcc tests/midi_input/) -- that is why the
// funnel is the default.
// ---------------------------------------------------------------------------

#ifndef __AMY_MIDI_PARSE_H
#define __AMY_MIDI_PARSE_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint8_t msg[3];    // status byte + the data bytes being collected
    uint8_t slot;      // which data byte the next data byte fills (0 or 1)
    uint8_t in_sysex;  // this stream is between an F0 and its F7
} midi_stream_parser_t;

// The stream currently assembling into the single shared sysex buffer, or
// NULL. Claimed at F0, released at F7 (or by the owner's next status byte,
// defensively). In the funnel build only the MIDI task touches this.
static midi_stream_parser_t *sysex_owner = NULL;

#ifdef AMY_MIDI_PARSE_MPSC
// MPSC build: F0s can race from different tasks; claim with CAS. Release is
// only ever done by the owner, so a plain release-store suffices there.
static inline int sysex_try_claim(midi_stream_parser_t *p) {
    midi_stream_parser_t *expected = NULL;
    return __atomic_compare_exchange_n(&sysex_owner, &expected, p, 0,
                                       __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}
static inline void sysex_release(midi_stream_parser_t *p) {
    (void)p;
    __atomic_store_n(&sysex_owner, NULL, __ATOMIC_RELEASE);
}
static inline int sysex_is_owner(midi_stream_parser_t *p) {
    return __atomic_load_n(&sysex_owner, __ATOMIC_ACQUIRE) == p;
}
#else
// Funnel build: single task, plain accesses.
static inline int sysex_try_claim(midi_stream_parser_t *p) {
    if (sysex_owner != NULL) return 0;
    sysex_owner = p;
    return 1;
}
static inline void sysex_release(midi_stream_parser_t *p) { (void)p; sysex_owner = NULL; }
static inline int sysex_is_owner(midi_stream_parser_t *p) { return sysex_owner == p; }
#endif

// Parse a chunk of one stream's bytes in its own context.
// This can start in the middle of a MIDI message; running status is handled
// by keeping the status byte around after a completed message. USB MIDI
// (and mac IAC) hand us fixed 3-byte groups even for shorter messages, and
// set `usb` so we end the loop once a message is parsed.
static void midi_parse_stream(midi_stream_parser_t *p, uint8_t *data, size_t len, uint8_t usb) {
    for (size_t i = 0; i < len; i++) {

        uint8_t byte = data[i];

        // Skip sysex in this parser until we get an F7.
        if (p->in_sysex) {
            if (byte == 0xF7) {
                p->in_sysex = 0;
                if (sysex_is_owner(p)) {
                    if (sysex_overflow) {
                        // What we hold is a prefix of a message we could not
                        // store. Dropping is the only honest option (see below).
                        sysex_overflow = 0;
                        sysex_len = 0;
                    } else {
                        AMY_MIDI_PARSE_SYSEX_DONE();
                    }
                    sysex_release(p);
                }
                // Not the owner: this stream's sysex lost the buffer to a
                // concurrent one and its bytes were dropped (reported at the
                // F0); nothing to deliver.
            } else if (sysex_is_owner(p)) {
                // Bound the write. sysex_len is a uint16_t and MAX_SYSEX_BYTES is
                // 16KB, so a sysex whose F7 never arrives (cable pulled mid-message,
                // line noise) used to run sysex_len all the way to 65535 -- ~48KB
                // written past a buffer that shares the PSRAM heap with the
                // MicroPython GC heap and the GM bank fallback. The failure landed
                // arbitrarily later, nowhere near the cause.
                //
                // Stop at MAX_SYSEX_BYTES-1, not MAX_SYSEX_BYTES: parse_sysex()
                // NUL-terminates with sysex_buffer[sysex_len], which at exactly
                // MAX_SYSEX_BYTES would be a 1-byte overflow of its own. Reserving
                // the last byte here is what makes that store in-bounds.
                //
                // DROP the message rather than truncate it. A truncated sysex is a
                // silent corruption: an SPSS z* transfer would hand a half-written
                // payload to amy_add_message()/MicroPython as if it were complete.
                // So: stop storing, latch the overflow, discard at the F7, and say
                // so ONCE per message on stderr -- not once per byte, which at MIDI
                // rates would flood the console. sysex_buffer is NULL if its
                // (unchecked) malloc in run_midi() failed; same policy, and it
                // names itself.
                if (sysex_buffer == NULL) {
                    if (!sysex_overflow) {
                        sysex_overflow = 1;
                        AMY_MIDI_PARSE_LOG("sysex_buffer not allocated, dropping sysex message\n");
                    }
                } else if (sysex_len < MAX_SYSEX_BYTES - 1) {
                    sysex_buffer[sysex_len++] = byte;
                } else if (!sysex_overflow) {
                    sysex_overflow = 1;
                    AMY_MIDI_PARSE_LOG("sysex longer than %d bytes, dropping message\n", MAX_SYSEX_BYTES - 1);
                }
            }
            // Not the owner: drop the byte (its F0 already logged the loss).
        } else {
            if (byte & 0x80) { // new status byte
                // System Real-Time messages (0xF8-0xFF) may be interleaved
                // anywhere in the stream -- even between the data bytes of
                // another message -- and must NOT disturb running status. So
                // handle them with a scratch buffer, leaving p->msg[] and
                // p->slot untouched.
                if (byte >= 0xF8) {
                    if (byte == 0xF8) { // clock. don't forward this on to Tulip userspace
                        AMY_MIDI_PARSE_CLOCK();
                    } else { // start/continue/stop/active-sensing/reset/etc
                        uint8_t rt[1] = { byte };
                        AMY_MIDI_PARSE_EMIT(rt, 1);
                    }
                    if (usb) i = len + 1; // exit the loop if usb
                } else {
                    // Channel Voice (0x80-0xE0) or System Common (0xF0-0xF7):
                    // these begin a fresh message and cancel running status.
                    if (sysex_is_owner(p)) {
                        // Defensive: an owner that somehow left sysex without
                        // an F7 must release the buffer here (this was the
                        // shared-state reset the old global parser did on
                        // every status byte), or sysex wedges for every
                        // stream. Clears sysex_overflow too: this is the
                        // other way an F7-less sysex ends, and the next F0
                        // must start clean.
                        sysex_len = 0;
                        sysex_overflow = 0;
                        sysex_release(p);
                    }
                    p->msg[0] = byte;
                    p->slot = 0; // drop any half-collected data bytes
                    if (byte == 0xF4 || byte == 0xF5 || byte == 0xF6) {
                        // 1-byte System Common (undefined / tune request)
                        AMY_MIDI_PARSE_EMIT(p->msg, 1);
                        if (usb) i = len + 1; // exit the loop if usb
                    } else if (byte == 0xF0) { // sysex start
                        // everything is an AMY message until 0xF7
                        p->in_sysex = 1;
                        if (sysex_try_claim(p)) {
                            sysex_len = 0;
                            sysex_overflow = 0;
                        } else {
                            // Two sources streaming sysex at once: keep the
                            // first, drop this one WHOLE. Say so -- silently
                            // losing a sysex (patch dump, SPSS transfer) is
                            // undebuggable; once per attempt is loud enough.
                            AMY_MIDI_PARSE_LOG("midi: concurrent sysex from a second source dropped\n");
                        }
                    }
                    // else: channel voice or F1/F2/F3 -- status stored, await data bytes
                }
            } else { // data byte of some kind
                uint8_t status = p->msg[0] & 0xF0;

                // a 2 bytes of data message
                if (status == 0x80 || status == 0x90 || status == 0xA0 || status == 0xB0 || status == 0xE0 || p->msg[0] == 0xF2) {
                    if (p->slot == 0) {
                        p->msg[1] = byte;
                        p->slot = 1;
                    } else {
                        p->msg[2] = byte;
                        p->slot = 0;
                        AMY_MIDI_PARSE_EMIT(p->msg, 3);
                    }
                // a 1 byte data message
                } else if (status == 0xC0 || status == 0xD0 || p->msg[0] == 0xF3 || p->msg[0] == 0xF1) {
                    p->msg[1] = byte;
                    AMY_MIDI_PARSE_EMIT(p->msg, 2);
                    if (usb) i = len + 1; // exit the loop if usb
                }
            }
        }
    }
}

#endif // __AMY_MIDI_PARSE_H
