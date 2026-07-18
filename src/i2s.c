// i2s.c
// handle i2s audio in & out on many platforms
// esp32 --> esp32, esp32-s3, esp32-p4 
// AMYBOARD, which is a esp32s3 but with special i2s setup
// rp2350, rp2040
// teensy 3.6, 4.0, 4.1

// Only run this code on MCUs
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO) || defined(ARDUINO_ARCH_RP2350)

#include "amy.h"


#ifdef ESP_PLATFORM
#include <esp_task.h>

///////////////////////////////////////////////////////////////
// ESP32, S3, P4 (maybe others)

// A note on RTOS synchronization:
// For maximumum flexibility, AMY offers two flags:
//  - config.platform.multicore - whether to use a second core, if avaliable.
//  - config.platform.multithread - whether to use RTOS multithreading, if avaliable.
// (ESP32 is the only platform where we've tried RTOS, so it's the only context where multithread matters).
// When using RTOS threads (tasks), synchronization is handled by xTaskNotifyGive(task_handle) / ulTaskNotifyTake():
// A service task waits to be notified by calling ulTaskNotifyTake(); that call does not specify where that
// notification comes from.  Another task calls xTaskNotifyGive() naming the particular task to be notified,
// which then releases the ulTaskNotifyTake() in that task.
// We use this mechanism as follows:
//   * esp_render_task (the task that runs on the second core):
//      - Waits for notification (from main fill_audio thread)
//      - renders half the oscs
//      - notifies either fill_buffer (multithread) or amy_update (single thread) when done.
//      - loop
//   * esp_fill_audio_buffer_task (launched as one parallel task):
//      - if not using I2S, waits for notification (from amy_update task)
//      - read I2S input (if used)
//      - execute AMY command updates (deltas)
//      - Notify esp_render_task
//      - render the other half of oscs
//      - Wait for notification (indicating that esp_render_task is done)
//      - call amy_fill_buffer (to combine the rendered oscs into output samples)
//      - Notify amy_update_handle that the new block is ready
//      - If I2S enabled, write samples to I2S.
//      - loop
//   * ESP's amy_platform_init
//      - if multicore, launch esp_render_task
//      - if multithread, launch esp_fill_audio_buffer_task
//      - if not I2S, Notify fill_buffer to get it started
//   * ESP's amy_render_audio
//      - If multithread, wait for Notification (from fill_audio_buffer)
//      - If not I2S, Notify fill_buffer
//      - If not multithread
//        - If multicore, Notify amy_render_handle to get task on second core to render
//          - wait for Notification indicating esp_render_task is done
//        - else render all oscs
//        - call amy_fill_buffer

#include "driver/i2s_std.h"
#ifdef AMYBOARD_ARDUINO
#include "driver/i2c_master.h"
#endif
i2s_chan_handle_t tx_handle;
i2s_chan_handle_t rx_handle;


#if !defined(AMYBOARD) && !defined(AMYBOARD_ARDUINO)
// default ESP setup i2s
amy_err_t esp32_setup_i2s(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    if(AMY_HAS_AUDIO_IN) {
        i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);
    } else {
        i2s_new_channel(&chan_cfg, &tx_handle, NULL);        
    }
// PCM5101 DAC works at either 32 bit or (default) 16 bit
// PCM1808 ADC needs I2S_32BIT to work
#define I2S_32BIT
#ifdef I2S_32BIT
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AMY_SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = (amy_global.config.i2s_mclk == -1)? I2S_GPIO_UNUSED : amy_global.config.i2s_mclk,
            .bclk = amy_global.config.i2s_bclk,
            .ws = amy_global.config.i2s_lrc,
            .dout = amy_global.config.i2s_dout,
            .din = amy_global.config.i2s_din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
#else // 16 bit I2S
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AMY_SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = (amy_global.config.i2s_mclk == -1)? I2S_GPIO_UNUSED : amy_global.config.i2s_mclk,
            .bclk = amy_global.config.i2s_bclk,
            .ws = amy_global.config.i2s_lrc,
            .dout = amy_global.config.i2s_dout,
            .din = amy_global.config.i2s_din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
#endif
    /* Initialize the channel */
    i2s_channel_init_std_mode(tx_handle, &std_cfg);
    if(AMY_HAS_AUDIO_IN) i2s_channel_init_std_mode(rx_handle, &std_cfg);

    /* Before writing data, start the TX channel first */
    i2s_channel_enable(tx_handle);
    if(AMY_HAS_AUDIO_IN) i2s_channel_enable(rx_handle);
    return AMY_OK;
}

#else
// AMYBOARD or AMYBOARD_ARDUINO i2s setup, which uses two audio codecs, for audio in and SPDIF
amy_err_t esp32_setup_i2s(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_SLAVE);  // ************* I2S_ROLE_SLAVE - needs external I2S clock input.
    i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);

#define I2S_32BIT
    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = AMY_SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_EXTERNAL,
            .ext_clk_freq_hz = AMY_SAMPLE_RATE * 512,
            .mclk_multiple = 512, 
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_width = 32,
            .ws_pol = false, // false in STD_PHILIPS macro
            .bit_shift = false, // true for STD_PHILIPS macro, but that results in *2* bits delay of dout vs lrclk in Follower mode. false gives 1 bit delay, as expected for i2s.
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false,
        },
        .gpio_cfg = {
            .mclk = amy_global.config.i2s_mclk, 
            .bclk = amy_global.config.i2s_bclk,
            .ws = amy_global.config.i2s_lrc,
            .dout = amy_global.config.i2s_dout,
            .din = amy_global.config.i2s_din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = true, // invert bclk for pcm9211 
                .ws_inv = false,
            },
        },
    };
    /* Initialize the channel */
    i2s_channel_init_std_mode(tx_handle, &std_cfg);
    i2s_channel_init_std_mode(rx_handle, &std_cfg);

    /* Before writing data, start the TX channel first */
    i2s_channel_enable(tx_handle);
    i2s_channel_enable(rx_handle);

#ifdef AMYBOARD_ARDUINO
    // Initialize PCM9211 SPDIF transceiver via I2C.
    // On the MicroPython AMYBOARD path this is done in Python (amyboard.py).
    // Uses the new (driver_ng) I2C master API so it coexists with Arduino's Wire,
    // which also uses driver_ng on ESP32 Arduino core 3.x.
    {
        #define PCM9211_I2C_ADDR   0x40
        #define PCM9211_I2C_SDA    17
        #define PCM9211_I2C_SCL    18
        #define PCM9211_I2C_FREQ   400000

        static const uint8_t pcm9211_regs[][2] = {
            { 0x40, 0x33 },  // Power down ADC, DIR, DIT, OSC
            { 0x40, 0xC0 },  // Normal operation for all
            { 0x34, 0x00 },  // Initialize DIR - biphase amps on, input from RXIN0
            { 0x26, 0x01 },  // Main Out is DIR/ADC if no DIR sync
            { 0x6B, 0x00 },  // Main output pins are DIR/ADC AUTO
            { 0x30, 0x04 },  // PLL sends 512fs as SCK
            { 0x31, 0x0A },  // XTI SCK as 512fs too
            { 0x60, 0x44 },  // DIT sends SPDIF from AUXIN1 through MPO0
            { 0x61, 0x20 },  // DIT SCK ratio = 512fs (must match PLL config in 0x30/0x31)
            { 0x78, 0x3D },  // MPO0 = TXOUT, MPO1 = VOUT
            { 0x6F, 0x40 },  // MPIO_A = CLKST / MPIO_B = AUXIN2 / MPIO_C = AUXIN1
        };

        i2c_master_bus_config_t bus_conf = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = PCM9211_I2C_SDA,
            .scl_io_num = PCM9211_I2C_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        i2c_master_bus_handle_t bus_handle = NULL;
        i2c_master_dev_handle_t dev_handle = NULL;
        esp_err_t ret = i2c_new_master_bus(&bus_conf, &bus_handle);
        if (ret == ESP_OK) {
            i2c_device_config_t dev_conf = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = PCM9211_I2C_ADDR,
                .scl_speed_hz = PCM9211_I2C_FREQ,
            };
            ret = i2c_master_bus_add_device(bus_handle, &dev_conf, &dev_handle);
        }
        if (ret != ESP_OK) {
            fprintf(stderr, "PCM9211: I2C init failed: %s\n", esp_err_to_name(ret));
        } else {
            for (int i = 0; i < sizeof(pcm9211_regs) / sizeof(pcm9211_regs[0]); i++) {
                uint8_t buf[2] = { pcm9211_regs[i][0], pcm9211_regs[i][1] };
                ret = i2c_master_transmit(dev_handle, buf, 2, 100);
                if (ret != ESP_OK) {
                    fprintf(stderr, "PCM9211: reg 0x%02x write 0x%02x failed: %s\n",
                        buf[0], buf[1], esp_err_to_name(ret));
                }
            }
        }

        // Tear down the I2C driver so Arduino Wire can use bus 0 later
        if (dev_handle) i2c_master_bus_rm_device(dev_handle);
        if (bus_handle) i2c_del_master_bus(bus_handle);
    }
#endif // AMYBOARD_ARDUINO

    return AMY_OK;
}

#endif

amy_err_t esp32_teardown_i2s(void) {
    i2s_channel_disable(tx_handle);
    if(AMY_HAS_AUDIO_IN) i2s_channel_disable(rx_handle);
    i2s_del_channel(tx_handle);
    if(AMY_HAS_AUDIO_IN) i2s_del_channel(rx_handle);
    return AMY_OK;
}


/////////////////////////////////////
// ESP mulithread/multicore rendering

TaskHandle_t amy_render_handle;
TaskHandle_t amy_fill_buffer_handle;

// Task to notify when amy_update is waiting for a completed buffer
TaskHandle_t amy_update_handle = NULL;

// Who esp_render_task tells that it is done.
TaskHandle_t amy_render_task_done_handle = NULL;

// ---------------------------------------------------------------------------
// OPT-9 core-split probe: per-core cycle cost of amy_render() per block.
//
// The osc split across the two cores is STATIC (AMY_OSCS/2): esp_render_task
// (pinned core 0, which also runs Tulip's display) renders oscs [0, N/2);
// esp_render_on_cores' caller (fill-buffer task, pinned core 1) renders
// [N/2, N).  Voices allocate from LOW osc numbers, so core 0 fills up first
// while core 1 may idle.  These counters measure that imbalance on-device:
// CCOUNT (cycle counter, one per core) is read right around each core's
// amy_render() call and the worst (max) per-block delta is kept per core,
// plus the most recent block's delta.  Read/reset from MicroPython via
// tulip.render_cyc() (tulipcc branch core-split-probe-binding).
//
// Always-on rather than #ifdef'd: the whole probe is two CCOUNT reads, a
// subtract, a compare and two stores per block per core (~20 cycles against
// a ~1.39M-cycle block budget at 240MHz, <0.002%), so it cannot meaningfully
// perturb what it measures, and keeping it unconditional means the measured
// firmware is the play firmware.  Per-core slots in internal DRAM (uncached
// on S3), each written only by its own core, so there is no cross-core race;
// aligned 32-bit stores are atomic.  A reset from core 1 (the binding) can
// lose at most one in-flight max update from one block -- irrelevant since
// the procedure resets BEFORE driving the workload.  CCOUNT wraps every
// ~17.9s at 240MHz; unsigned end-start subtraction stays correct across a
// wrap as long as one render block is < 2^32 cycles (it is, by ~3000x).
// Deltas include any preemption of the render task (ISRs, flash-guard
// parks), which is what the GO/NO-GO question wants anyway: worst real
// wall-cycles spent before the block is done.
#include "esp_cpu.h"
volatile uint32_t amy_render_worst_cyc[2] = { 0, 0 };  // [core] max cycles/block since reset
volatile uint32_t amy_render_last_cyc[2]  = { 0, 0 };  // [core] most recent block

static inline void amy_render_timed(uint16_t start, uint16_t end, uint8_t core) {
    uint32_t c0 = esp_cpu_get_cycle_count();
    amy_render(start, end, core);
    uint32_t dt = esp_cpu_get_cycle_count() - c0;
    uint32_t cid = (uint32_t)esp_cpu_get_core_id() & 1;   // index by CPU, not by amy's buffer arg
    amy_render_last_cyc[cid] = dt;
    if (dt > amy_render_worst_cyc[cid]) amy_render_worst_cyc[cid] = dt;
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// OPT-10 ADAPTIVE core split.
//
// The static split rendered oscs [0, AMY_OSCS/2) on physical core 0
// (esp_render_task) and [AMY_OSCS/2, AMY_OSCS) on physical core 1 (the fill
// task).  Voices allocate from LOW osc indices, so core 0 got nearly all the
// audible oscs and ran ~124-161% of the block budget while core 1 idled at
// ~34-37% (measured, additive-piano chord).  A block is not done until BOTH
// cores' renders finish, so max(R0,R1) sets the critical path -- the imbalance
// is pure lost throughput.
//
// We replace the fixed AMY_OSCS/2 boundary with an adaptive index `s`: core 0
// renders [0, s), core 1 renders [s, AMY_OSCS).  Each block a proportional
// controller nudges `s` to equalise the two cores' *render* times, using the
// previous block's per-core cycle counts the probe already captured
// (amy_render_last_cyc[cpu]).
//
// Why balance RENDER times (R0 == R1) and NOT R0 == R1 + fill:  the fill work
// (combine + per-bus FX + I2S pack) runs on core 1 only AFTER esp_render_on_cores
// returns, and the combine consumes BOTH cores' fbl[] output, so it cannot begin
// until core 0's render is also done (the ulTaskNotifyTake barrier below).  Fill
// is therefore serial after the render barrier, never overlapped with core 0's
// render, and its cost F is on the critical path for ANY split.  The block's
// wall time is max(R0,R1) + F; with total render T = R0+R1 roughly fixed, that
// is minimised at R0 == R1 == T/2.  Targeting R0 == R1 + F would instead idle
// core 1 for F before the barrier and make the path R0 + F = T/2 + fill + fill
// -- strictly worse.  (See the report for the break-even analysis: even a
// perfect split only removes the *sustained* underrun; a single worst 2.77M
// total-render block still needs F < ~5k cyc to fit 1.39M, which it won't, so
// the DMA ring must absorb the rare worst block.)
//
// Correctness: `s` only repartitions a contiguous, complete, disjoint osc range
// -- [0,s) U [s,AMY_OSCS) == [0,AMY_OSCS) for every s in [0,AMY_OSCS], every osc
// rendered exactly once.  The `core` buffer argument (1 for core 0, 0 for the
// fill task) is UNCHANGED, so per_osc_fb[]/fbl[] accumulator roles, the OPT-11
// per-core bus masks, and the chorus-mod-source pass (gated on core==0) are all
// unaffected; the mask-aware combine already sums the two cores' buses
// split-invariantly.  `s` is published once per block by the fill task BEFORE it
// notifies core 0 (release via xTaskNotifyGive); core 0 reads it only after its
// ulTaskNotifyTake wakes (acquire), so both cores use the same `s` for a block
// and it never changes mid-block (single writer, aligned 16-bit store, atomic).
volatile uint16_t amy_split_index = 0;   // boundary osc; seeded to AMY_OSCS/2 on first block
static bool amy_split_inited = false;    // distinct from a legitimately-clamped s==0

// Controller constants.  DEADBAND exceeds ~2x a single audible osc's cost so a
// settled split does not limit-cycle around the balance point; GAIN_SHIFT gives
// step ~= imbalance / 16k-cyc-per-osc; MAX_STEP bounds the per-block move so one
// noisy (ISR-preempted) block cannot fling the split far (self-corrects anyway).
#define AMY_SPLIT_DEADBAND   65536u   // cycles (~4.7% of a 1.39M block); hysteresis
#define AMY_SPLIT_GAIN_SHIFT 14       // step = err >> 14  (~1 osc per 16384 cyc)
#define AMY_SPLIT_MAX_STEP   8        // clamp osc-index move per block

// Snap a proposed split index to a VOICE boundary so neither core is handed a
// partial voice.  A voice is a chain: a silent head osc renders the chain sum
// then applies its envelope + filter (amy.c), while the chain-tail oscs are
// SYNTH_AUDIBLE at constant amp 1.0 (no EG, no velocity coef).  If the split s
// lands strictly inside a voice, the FW-6 atomic render_clock claim lets the
// two cores render the head and the tail separately: the tail oscs then render
// RAW -- full-scale, un-enveloped, un-filtered -- into the mix while the head's
// filtered sum loses them.  That is the velocity-invariant, block-gated
// full-scale buzz on soft notes ("wavefolder at 11").  Aligning s to a voice
// boundary keeps every chain whole on ONE core.
//
// Voices occupy CONTIGUOUS osc blocks carrying a single voice id (patches.c
// allocates a run and stamps osc_to_voice[osc..osc+n) = voice); osc_to_voice is
// AMY_UNSET in the gaps between voices.  Snap DOWN to the base of the voice
// containing s; if that voice starts at osc 0 (snapping down would empty core
// 0), snap UP to its end instead so both halves stay meaningful -- only a
// single voice spanning the whole osc range collapses to one core, which is
// unavoidable (you cannot split one chain without tearing it).
//
// osc_to_voice cannot change between this snap and the render pass: it is only
// mutated by amy_execute_deltas(), which the fill task runs at the TOP of the
// block loop BEFORE esp_render_on_cores() -> amy_update_split(); core 0 is
// parked until the notify below.  So the map this reads is exactly the one both
// cores render against this block.
static uint16_t amy_split_snap(uint16_t s) {
    if (osc_to_voice == NULL) return s;             // no voice map: nothing to align
    if (s == 0 || s >= AMY_OSCS) return s;          // already at an edge boundary
    if (AMY_IS_UNSET(osc_to_voice[s])) return s;    // s is in a gap: aligned
    if (osc_to_voice[s] != osc_to_voice[s - 1]) return s;  // s is a voice base: aligned
    // s is strictly inside a voice.  Find that voice's base osc.
    uint8_t v = osc_to_voice[s];
    uint16_t base = s;
    while (base > 0 && osc_to_voice[base - 1] == v) base--;
    if (base > 0) return base;                      // snap down: tear-free, both cores fed
    // Voice starts at osc 0: snapping down would empty core 0.  Snap UP to the
    // voice's end (== AMY_OSCS only for a single all-spanning voice).
    uint16_t end = s;
    while (end < AMY_OSCS && osc_to_voice[end] == v) end++;
    return end;
}

// Compute and publish the next block's split index from the last block's
// per-core render times.  Runs on the fill task (core 1) only, and only while
// core 0 is parked between blocks, so it is the sole writer of amy_split_index.
static void amy_update_split(void) {
    uint16_t n = AMY_OSCS;
    if (!amy_split_inited) {                             // first block: match static split
        amy_split_index = n / 2;                        // (s==0 is a valid clamped state, not "unset")
        amy_split_inited = true;
    }
    uint32_t r0 = amy_render_last_cyc[0];   // physical core 0 render == oscs [0, s)
    uint32_t r1 = amy_render_last_cyc[1];   // physical core 1 render == oscs [s, N)
    int32_t err = (int32_t)r0 - (int32_t)r1; // >0 => core 0 too heavy => lower s
    int32_t mag = err < 0 ? -err : err;
    if (mag <= (int32_t)AMY_SPLIT_DEADBAND) return;      // balanced enough; hold
    int32_t step = err >> AMY_SPLIT_GAIN_SHIFT;          // proportional
    if (step >  AMY_SPLIT_MAX_STEP) step =  AMY_SPLIT_MAX_STEP;
    if (step < -AMY_SPLIT_MAX_STEP) step = -AMY_SPLIT_MAX_STEP;
    if (step == 0) step = (err > 0) ? 1 : -1;            // guarantee progress past deadband
    int32_t s = (int32_t)amy_split_index - step;         // err>0 -> s decreases
    if (s < 0) s = 0;
    if (s > (int32_t)n) s = (int32_t)n;
    // Align to a voice boundary before publishing so a chain is never split
    // across cores (see amy_split_snap): the balancing controller still drives
    // s from the render-time error, but the published boundary is quantized to
    // whole voices.  The move may exceed MAX_STEP by up to one voice's width;
    // the deadband keeps that from limit-cycling.
    amy_split_index = amy_split_snap((uint16_t)s);       // publish (single writer)
}
// ---------------------------------------------------------------------------

// Render the second core
void esp_render_task( void * pvParameters) {
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // from esp_render_on_cores
        // amy_split_index was published by the fill task before this notify
        // (release/acquire across the notify), so it is fixed for this block.
        amy_render_timed(0, amy_split_index, 1);
        // Tell (someone) we're done.
        xTaskNotifyGive(amy_render_task_done_handle);  // to esp_render_on_cores
    }
}

void esp_render_on_cores() {
    // Call amy_render on all the oscs, using multicore if available.
    if (amy_global.config.platform.multicore) {
        // Tell the esp_render_task to inform *us* when it's done.
        amy_render_task_done_handle = xTaskGetCurrentTaskHandle();
        // Choose and publish this block's split BEFORE waking core 0, so both
        // cores render against the same boundary (see OPT-10 note above).
        amy_update_split();
        uint16_t s = amy_split_index;   // snapshot: core 1's half must match core 0's
        // Tell the other core to start rendering.
        xTaskNotifyGive(amy_render_handle);  // to esp_render_task
        // Render me: the high half [s, AMY_OSCS)
        amy_render_timed(s, AMY_OSCS, 0);
        // Wait for the other core to finish
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // from esp_render_task
    } else {
        // We render everything on this core.
        amy_render_timed(0, AMY_OSCS, 0);
    }
}

#ifdef I2S_32BIT
  static int32_t block32[AMY_BLOCK_SIZE * AMY_NCHANS];
  #define I2S_BYTES_PER_SAMPLE 4
#else
  #define I2S_BYTES_PER_SAMPLE AMY_BYTES_PER_SAMPLE
#endif
extern output_sample_type * amy_in_block;

// Place where render thread leaves address of samples.
// Set by esp_fill_audio_buffer_task, cleared when returned by amy_render_audio (if used).
int16_t *volatile last_audio_buffer = NULL;
// (see also amy_get_output_buffer, I should choose only one of these)

void esp_read_i2s_input() {
    // Read a block of i2s input.  Separated to isolate noise from different i2s formats.
    size_t read = 0;
#ifdef I2S_32BIT
    i2s_channel_read(rx_handle, block32, AMY_BLOCK_SIZE * AMY_NCHANS * sizeof(int32_t), &read, portMAX_DELAY);
    for (int i = 0; i < AMY_BLOCK_SIZE * AMY_NCHANS; ++i)
        amy_in_block[i] = (block32[i] >> 16);
#else
    i2s_channel_read(rx_handle, amy_in_block, AMY_BLOCK_SIZE * AMY_NCHANS * sizeof(output_sample_type), &read, portMAX_DELAY);
#endif
    if(read != AMY_BLOCK_SIZE * AMY_NCHANS * I2S_BYTES_PER_SAMPLE) {
        fprintf(stderr,"i2s input underrun: %d vs %d\n", read, AMY_BLOCK_SIZE * AMY_NCHANS * I2S_BYTES_PER_SAMPLE);
    }
}

// Make AMY's FABT run forever , as a FreeRTOS task

#ifdef ARDUINO_SPEEDTEST
/* rl-patch */
#include "esp_timer.h"
#include <stdio.h>
static int64_t _rl_last_print = 0;
static int32_t _rl_render_us = 0;
#endif // ARDUINO_SPEEDTEST

void esp_fill_audio_buffer_task() {
    while(1) {
        int64_t t;
        uint32_t blocked_us = 0;
        AMY_PROFILE_START(AMY_ESP_FILL_BUFFER)
        if(AMY_HAS_I2S && AMY_HAS_AUDIO_IN) {
            t = amy_get_us();
            esp_read_i2s_input();
            blocked_us += (uint32_t)(amy_get_us() - t);
	}
        t = amy_get_us();
#ifdef ARDUINO_SPEEDTEST
        int64_t _rl_start_t = esp_timer_get_time();
#endif // ARDUINO_SPEEDTEST
        // Get ready to render
        amy_execute_deltas();

        // Render on whichever cores we have available.
        esp_render_on_cores();

        // Write to i2s
        output_sample_type *block = amy_fill_buffer();
        uint32_t busy_us = (uint32_t)(amy_get_us() - t);
	AMY_PROFILE_STOP(AMY_ESP_FILL_BUFFER)

        last_audio_buffer = block;

        // Notify amy_update() that a block is ready (so it can return from amy_render_audio).
        if (amy_update_handle)
            xTaskNotifyGive(amy_update_handle);  // to amy_render_audio

#ifdef ARDUINO_SPEEDTEST
        {
            int64_t _rl_now = esp_timer_get_time();
            int32_t _rl_render_us_unsmooth = (int32_t)(_rl_now - _rl_start_t);
            _rl_render_us += (_rl_render_us_unsmooth - _rl_render_us) >> 5;  // 0.03125 * delta, settle time ~30 steps
            if (_rl_now - _rl_last_print > 500000) {
                _rl_last_print = _rl_now;
                fprintf(stderr, "RENDER_LOAD ms=%lu render_us=%d\n", (unsigned long)(_rl_now/1000), _rl_render_us);
                fflush(stderr);
            }
        }
#endif // ARDUINO_SPEEDTEST

        t = amy_get_us();
        if (AMY_HAS_I2S) {
            amy_i2s_write((uint8_t *)block, AMY_BLOCK_SIZE * AMY_NCHANS * sizeof(int16_t));
        } else {
            // Wait for update sync.
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // from amy_render_audio:!AMY_HAS_I2S
        }
        blocked_us += (uint32_t)(amy_get_us() - t);

        // When rendering keeps up, this task spends most of each block parked in the
        // i2s DMA write (or the update-sync wait) above, which is when lower-priority
        // tasks on this core get to run.
        amy_overload_check(busy_us);
        // If the audio output didn't block at all, we're past overloaded, and this
        // max-priority task would starve everything else on this core (USB, MIDI,
        // the host app).  Audio is already breaking up, so give the rest of the
        // system a tick.
        if (blocked_us < 150) vTaskDelay(1);
    }
}

// init AMY from the esp. wraps some amy funcs in a task to do multicore rendering on the ESP32 
void amy_platform_init() {
    // If we're running amy_update, this should be the task we need to return to.
    // However, if we're not using amy_update (e.g., Tulip/AMYboard native), we don't
    // want to do this - the un-handled xTaskNotifyGives cause Tulip to crash when it
    // turns on WiFi.
#ifdef ARDUINO
    amy_update_handle = xTaskGetCurrentTaskHandle();
#endif
    if (AMY_HAS_I2S) {
        // Start i2s
        esp32_setup_i2s();
    }
    if (amy_global.config.platform.multicore) {
        // On ESP, multicore starts a second thread even if multithread is not requested.
        // Create the second core rendering task
        xTaskCreatePinnedToCore(&esp_render_task, AMY_RENDER_TASK_NAME, AMY_RENDER_TASK_STACK_SIZE, NULL, AMY_RENDER_TASK_PRIORITY, &amy_render_handle, AMY_RENDER_TASK_COREID);
    }
    if (amy_global.config.platform.multithread) {
        // Create the fill audio buffer thread, combines, does volume & filters
        xTaskCreatePinnedToCore(&esp_fill_audio_buffer_task, AMY_FILL_BUFFER_TASK_NAME, AMY_FILL_BUFFER_TASK_STACK_SIZE, NULL, AMY_FILL_BUFFER_TASK_PRIORITY, &amy_fill_buffer_handle, AMY_FILL_BUFFER_TASK_COREID);
        // Let amy_update know we have I2S covered.
        if (AMY_HAS_I2S) {
            amy_global.i2s_is_in_background = 1;
        }
    }
}

void amy_platform_deinit() {
    if (AMY_HAS_I2S) {
        esp32_teardown_i2s();
    }
    if (amy_global.config.platform.multicore) {
        vTaskDelete(amy_render_handle);
    }
    if (amy_global.config.platform.multithread) {
        vTaskDelete(amy_fill_buffer_handle);
    }
}

extern void esp_poll_midi(void);

void amy_update_tasks() {
    if (!amy_global.config.platform.multithread) {
        amy_execute_deltas();
        esp_poll_midi();
    } else{
        // Rendering is happening on separate thread, nothing to do.
    }
}

int16_t *amy_render_audio() {
    // Called by api.amy_update() to render the audio.  Not used for non-Arduino.
    int16_t *buf = NULL;
    if (amy_global.config.platform.multithread) {
        // Wait for esp_fill_audio_buffer_task to indicate a buffer is ready.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // from esp_fill_audio_buffer_task
        buf = last_audio_buffer;
        // Allow the FABT to generate another block
        if (!AMY_HAS_I2S) {
            xTaskNotifyGive(amy_fill_buffer_handle);  // to esp_fill_audio_buffer_task:!AMY_HAS_I2S
        }
    } else {
        // No multithread, we have to render here.
        int64_t t0 = amy_get_us();
        esp_render_on_cores();
        buf = amy_fill_buffer();
        amy_overload_check((uint32_t)(amy_get_us() - t0));
    }
    return buf;
}

size_t amy_i2s_write(const uint8_t *buffer, size_t nbytes) {
    size_t written = 0;
    int16_t *block = (int16_t *)buffer;
    
#ifdef I2S_32BIT // including AMYBOARD
    // Convert to 32 bits
    for (int i = 0; i < AMY_BLOCK_SIZE * AMY_NCHANS; ++i)
        block32[i] = ((int32_t)block[i]) << 16;
    i2s_channel_write(tx_handle, block32, AMY_BLOCK_SIZE * AMY_NCHANS * I2S_BYTES_PER_SAMPLE, &written, portMAX_DELAY);
#else  // 16 bit I2S
    i2s_channel_write(tx_handle, block, AMY_BLOCK_SIZE * AMY_NCHANS * I2S_BYTES_PER_SAMPLE, &written, portMAX_DELAY);
#endif

    if(written != AMY_BLOCK_SIZE * AMY_NCHANS * I2S_BYTES_PER_SAMPLE) {
        fprintf(stderr,"i2s output underrun: %d vs %d\n", written, AMY_BLOCK_SIZE * AMY_NCHANS * I2S_BYTES_PER_SAMPLE);
    }
    return 1;
}

#elif (defined ARDUINO_ARCH_RP2040) || (defined ARDUINO_ARCH_RP2350)

#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico-audio/audio_i2s.h"
#include "pico/binary_info.h"
#include "pico/util/queue.h"

// Provided by pico_support.cpp
extern void pico_setup_i2s(amy_config_t *config);
extern void pico_teardown_i2s(amy_config_t *config);
extern void pico_i2s_read_write_buffer(int16_t *in_samples, const int16_t *out_samples, int nframes);

struct audio_buffer_pool *ap;

static inline uint32_t _millis(void)
{
    return to_ms_since_boot(get_absolute_time());
}

typedef struct
{
    int32_t (*func)(int32_t);
    int32_t data;
} queue_entry_t;

queue_t call_queue;
queue_t results_queue;

volatile bool core1_running = true;
// This is the ram allocated for the core1 stack.
// It's also the flag that core1 task is running, if non-NULL.
uint32_t * my_core1_separate_stack_address = NULL;

extern void on_pico_uart_rx();

void amy_update_tasks() {
    amy_execute_deltas();
    if(amy_global.config.midi & AMY_MIDI_IS_UART) on_pico_uart_rx();
#ifdef TUD_USB_GADGET
    if(amy_global.config.midi & AMY_MIDI_IS_USB_GADGET) on_pico_uart_rx();
#endif
}

#define USE_SECOND_CORE

int32_t render_other_core(int32_t data) {
    amy_render(AMY_OSCS/2, AMY_OSCS, 1);
    return AMY_OK;
}

int16_t *amy_render_audio() {
    int64_t t0 = amy_get_us();
#ifdef USE_SECOND_CORE
    if (amy_global.config.platform.multicore) {
        int32_t res;
        queue_entry_t entry = {render_other_core, AMY_OK};
        queue_add_blocking(&call_queue, &entry);
        amy_render(0, AMY_OSCS/2, 0);
        queue_remove_blocking(&results_queue, &res);
    } else
#endif
        amy_render(0, AMY_OSCS, 0);
    int16_t *block = amy_fill_buffer();
    amy_overload_check((uint32_t)(amy_get_us() - t0));
    return block;
}

size_t amy_i2s_write(const uint8_t *buffer, size_t nbytes) {
    // Single function to update buffers.
    // len is the number of int16 sample frames.
    pico_i2s_read_write_buffer(amy_in_block, (const int16_t *)buffer, AMY_BLOCK_SIZE);
    return nbytes;
}


struct audio_buffer_pool *init_audio() {
    static audio_format_t audio_format = {
            .format = AUDIO_BUFFER_FORMAT_PCM_S16,
            .sample_freq = AMY_SAMPLE_RATE,
            .channel_count = AMY_NCHANS,
    };

    static struct audio_buffer_format producer_format = {
            .format = &audio_format,
            .sample_stride = sizeof(int16_t) * AMY_NCHANS,
    };

    struct audio_buffer_pool *producer_pool = audio_new_producer_pool(&producer_format, 3, AMY_BLOCK_SIZE);

    bool __unused ok;
    const struct audio_format *output_format;
    struct audio_i2s_config config = {
            .data_pin = amy_global.config.i2s_dout,
            .clock_pin_base = amy_global.config.i2s_bclk,
            .dma_channel = 0,
            .pio_sm = 0,
    };

    output_format = audio_i2s_setup(&audio_format, &config);
    if (!output_format) {
        panic("PicoAudio: Unable to open audio device.\n");
    }

    ok = audio_i2s_connect(producer_pool);
    assert(ok);
    audio_i2s_set_enabled(true);

    return producer_pool;
}

void core1_main() {
    while (core1_running) {
        queue_entry_t entry;
        queue_remove_blocking(&call_queue, &entry);
        int32_t result = entry.func(entry.data);
        queue_add_blocking(&results_queue, &result);
    }
}

void amy_platform_init() {
    if (AMY_HAS_I2S) {
        pico_setup_i2s(&amy_global.config);
    }
#ifdef USE_SECOND_CORE
    if (amy_global.config.platform.multicore) {
        if (my_core1_separate_stack_address == NULL) {  // Task is not already running.
            my_core1_separate_stack_address = (uint32_t*)malloc(0x2000);
            queue_init(&call_queue, sizeof(queue_entry_t), 2);
            queue_init(&results_queue, sizeof(int32_t), 2);
            core1_running = true;
            multicore_launch_core1_with_stack(core1_main, my_core1_separate_stack_address, 0x2000);
            sleep_ms(500);
        }
    }
#endif
}

void amy_platform_deinit() {
#ifdef USE_SECOND_CORE
    if (amy_global.config.platform.multicore) {
        if (my_core1_separate_stack_address) {  // Task is actually running.
            core1_running = false;  // signal the core1 task to exit.
            sleep_ms(100);  // time for it to exit
            queue_free(&results_queue);
            queue_free(&call_queue);
            free(my_core1_separate_stack_address);
            my_core1_separate_stack_address = NULL;
        }
    }
#endif
    pico_teardown_i2s(&amy_global.config);
}


#elif defined __IMXRT1062__


extern void teensy_setup_i2s();
extern void teensy_teardown_i2s();
extern size_t teensy_i2s_write(const uint8_t *buffer, size_t nbytes);

extern int16_t teensy_get_serial_byte();

void amy_platform_init() {
    if (AMY_HAS_I2S) {
        teensy_setup_i2s();
    }
}

void amy_platform_deinit() {
    if (AMY_HAS_I2S) {
        teensy_teardown_i2s();
    }
}

void amy_update_tasks() {
    if(amy_global.config.midi & AMY_MIDI_IS_UART) {
        // do midi in here
        uint8_t bytes[1];
        int t;
        while((t = teensy_get_serial_byte()) >= 0) {
            bytes[0] = t;
            convert_midi_bytes_to_messages(bytes,1,0);
        }
    }
    amy_execute_deltas();
}

int16_t *amy_render_audio() {
    int64_t t0 = amy_get_us();
    amy_render(0, AMY_OSCS, 0);
    int16_t *block = amy_fill_buffer();
    amy_overload_check((uint32_t)(amy_get_us() - t0));
    return block;
}

size_t amy_i2s_write(const uint8_t *buffer, size_t nbytes) {
    return teensy_i2s_write(buffer, nbytes);
}

#else

//...

#endif


#endif
