# Aux-send reverb architecture — spike + implementation plan

## Why

AMY's FX buses are **insert** chains: each bus owns a full chorus/reverb/echo/EQ
that processes only what renders into that bus. The deck uses buses as
per-instrument inserts (chorus/EQ character per patch), which forced a choice
for reverb: one reverb per instrument (CPU-prohibitive) or one master reverb on
the summed output (`AMY_MASTER_REVERB`, shipped) where **every instrument gets
the same amount of room**.

The classic mixer answer is an **aux send**: one shared reverb, and each
channel taps a controllable amount of its post-fader signal into it. The pad
can be drenched while the piano stays dry — still paying for exactly one
reverb.

## The spike (this branch)

Compiles under `-DAMY_AUX_REVERB` (and `-DAMY_MASTER_REVERB` unchanged);
not yet run on hardware.

- `bus_state_t.reverb_send` (float, **default 1.0** — which reproduces
  master-room behavior bit-for-bit, so migration is a no-op until a send is
  changed).
- Wire: `q<float>` with an optional `y<bus>` → `amy.send(bus=2, reverb_send=0.3)`.
  Event field `reverb_send`, param `REVERB_SEND`, routed through the existing
  bus-directed-command path (`d.osc` carries the bus).
- Render (`amy.c`, in the mixdown): one fold pass computes both the dry master
  sum and the send accumulation (`send_scale[bus] = volume_scale[bus] *
  reverb_send[bus]`, i.e. post-fader). Bus 0's reverb processes the aux buffer
  once; its wet output is added to the master.
- Spike shortcut: `stereo_reverb()` emits `dry + level*wet`, so the spike keeps
  a copy of the aux input and adds `(out - in)` to the master. **Production
  replaces this with a wet-only `stereo_reverb` variant in delay.c** (saves a
  2KB static buffer and a subtract pass).

## Cost

- CPU: identical to `AMY_MASTER_REVERB` plus one extra multiply-accumulate per
  bus per sample in the fold (buses ≤ 4, negligible next to the reverb itself).
  Still exactly one reverb.
- RAM: two static 2KB block buffers in the spike; one after the wet-only
  variant lands.

## Implementation plan (production)

1. **Wet-only reverb** (`src/delay.c`): variant of `stereo_reverb` that adds
   `level*wet` into a separate accumulate target and never mixes dry. Reuse the
   locals-cached loop; ~30 lines. Drop `auxdry`.
2. **Replace `AMY_MASTER_REVERB` with `AMY_AUX_REVERB`** in the Tulip builds
   (`tulip/esp32s3/esp32_common.cmake`, `tulip/shared/tulip.mk`). Default
   send 1.0 keeps behavior identical until the deck sets sends.
3. **Patch store/recall**: persist `reverb_send` in patches.c (EPRINT /
   RET_TRUE_IF_SET / CASE / EVENT_FROM_OSC family) so a stored patch can carry
   its room amount. Optional first pass: skip — the deck owns sends via config.
4. **Python lib**: `('reverb_send', 'qF')` in `amy/__init__.py`'s wire map.
5. **Deck router** (`forwarder._apply_device_fx`): per-instrument send from the
   instrument's FX dict (`fx['reverb_send']`, default 1.0) →
   `amy.send(bus=B, reverb_send=v)` in the per-bus baseline pass.
6. **Deck UI**: "Reverb send" slider in the instrument editor's Sound/FX
   surface (per-instrument), while the device FX panel keeps the ROOM's
   character (level/liveness/damping). Editor seeding: user > default(1.0) —
   patches don't carry sends until (3).
7. **Tests**: desktop render test — two buses, send 0 vs 1, assert the dry bus
   contributes no wet energy; wire round-trip for `q`; deck unit tests for the
   send plumbing.
8. **Validation on deck**: A/B `AMY_MASTER_REVERB` vs `AMY_AUX_REVERB` with all
   sends at 1.0 (should be indistinguishable), then piano send 0 / pad send 1.

Estimated effort: an evening — the spike proved every integration point; the
only new DSP code is the wet-only reverb loop.
