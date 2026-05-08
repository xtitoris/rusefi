# Trigger Sync: Angle-Defined Time-Domain Predictor — Design Document

## Problem Statement

The current synchronization logic in `isSyncPoint()` (`trigger_decoder.cpp:679`) detects the sync point by checking whether consecutive tooth gap ratios fall within static windows:

```
toothDurations[i] > toothDurations[i+1] * from
toothDurations[i] < toothDurations[i+1] * to
```

This implicitly assumes **constant engine speed** between the two teeth being compared.  Under rapid acceleration or deceleration this assumption fails, causing false sync or missed sync.  Additionally, the Miata NB wheel requires a completely separate branch (`if (triggerType == TT_MIATA_VVT)`) precisely because the ratio-window approach is too fragile for that wheel.

### Root Cause

A ratio `toothDurations[i] / toothDurations[i+1]` equals the angle ratio only when angular velocity is constant.  Under acceleration the same angular interval takes less time, and the ratio drifts away from its steady-state value even on a perfectly-machined wheel.

---

## Proposed Solution

The redesign separates two concerns that are currently tangled together in `isSyncPoint()`:

1. **Kinematics** — what should this tooth look like given current engine speed and acceleration?
2. **Geometry** — is the observed pattern consistent with being at the sync position of this wheel?

This is achieved with a three-layer architecture that runs on every tooth event:

### Layer 1 — Innovation-Variance Tracker (every tooth)

Maintains a running estimate of the normalized crank rate $\hat r$ and its first derivative $\widehat{\dot r}$, updated from every tooth that passes the gate.  Also tracks the running variance of the prediction error ("innovation") as an exponential moving average.  This replaces the static ratio-window comparison and makes the speed estimate always fresh — there is no stale-history problem across sync loss.

### Layer 2 — Gated Innovation Filter (every tooth)

Before updating the tracker or the sync detector, the current tooth's prediction error (innovation) is tested against a gate.  If $e^2 > k \cdot \text{var}$, the tooth is declared a glitch: the tracker state and the rolling hash key are both frozen.  Because the gate threshold $k \cdot \text{var}$ scales with the running variance, the gate widens automatically during transients (when `var` is large from legitimate kinematic variation) and narrows at steady state (when `var` is small).  No fixed tolerance constant needs tuning per wheel.

### Layer 3 — Dual-Mode Sync Decision (non-glitch teeth only)

**Acquisition mode** (before sync, or after sync loss): each tooth is classified into {VERY_SHORT, SHORT, NORMAL, LONG} relative to the tracker prediction.  The last $N$ classifications form a rolling key (base-4, 2 bits per symbol, pure bit-shift update) that is compared against a small precomputed list of **(key, sync-position)** pairs.  Only the patterns that uniquely identify a sync position are stored — for most wheels this is 1–2 entries.  Sync is declared when the key matches one of those entries.

**Tracking mode** (after sync is established): on every tooth, decrement a countdown counter.  When it reaches zero, the current rolling key is compared against the expected sync key for that position (indexed by `syncExpectedKeyIndex`).  A match confirms sync and reloads the countdown and index for the next sync position on the wheel.  A mismatch immediately declares sync lost and re-enters acquisition mode.  This catches silent position drift within one revolution with zero per-tooth list scanning.

This architecture:
- Handles all wheel types with a single code path — no per-wheel branches.
- Is robust to compression artifacts: the tracker explicitly models $\dot r$, so compression-stroke deceleration is predicted and classified away.
- Is robust to single-tooth glitches: gated teeth are skipped without corrupting tracker or key.
- Requires no per-wheel tuning: the sync-position key list is derived deterministically from wheel geometry.
- Has minimal RAM overhead: the key list is typically 1–2 entries regardless of wheel complexity.

---

## Data Structures

### Fixed-Point Constants

```cpp
// Fractional shift for precomputed angle reciprocals.
// Q=12 gives ~0.024% resolution for angles ≥ 1° (100 units in 0.01° units)
// and keeps worst-case multiply (uint16 × uint16) well within 32 bits.
#define TRIGGER_SYNC_Q 12

// Fractional shift for the innovation-variance tracker's internal rate units.
// Rate r = (dt >> SYNC_SHIFT) * invAngleQ >> TRIGGER_SYNC_Q
// SYNC_SHIFT prevents overflow: at cranking speeds dt can be ~10M ticks.
// SYNC_SHIFT = 4 divides by 16, keeping r representable in 32 bits.
#define TRIGGER_SYNC_SHIFT 4
```

### Per-Event Angle Model — `sync_event_model_s`

Stored in `TriggerWaveform` for every trigger event across the full engine cycle (not just the sync window).  This is required so the tracker can normalize every tooth, not just the ones near the sync point.

```cpp
typedef struct {
    // Nominal angular width of the gap ending at this event, in 0.01° units.
    // Range 100–72000, fits in uint16.
    uint16_t angle;
    // Precomputed reciprocal: (1 << TRIGGER_SYNC_Q) / angle
    uint16_t invAngleQ;
} sync_event_model_s;

// One entry per trigger event in the engine cycle.
// Sized to the maximum trigger event count.
sync_event_model_s eventModel[PWM_PHASE_MAX_COUNT];
```

### Sync-Position Key List — `TriggerWaveform`

Only the patterns that uniquely identify a sync position are stored.  For most wheels (36-2, 60-2, Miata NB) there is exactly one such position.  For 36-2-2-2 there are three.

```cpp
// Maximum number of sync positions on any supported wheel.
#define MAX_SYNC_POSITIONS 4

typedef struct {
    uint32_t key;      // base-4 quad key for this sync position (2 bits per symbol)
    uint8_t  pos;      // trigger event index of the sync point
    uint8_t  countdown; // teeth from this sync position to the next one on this wheel
                        // (= getLength() for single-sync-point wheels)
} sync_key_entry_s;

// Number of consecutive tooth classifications forming the key.
// Minimum depth such that every sync position has a globally unique key
// (no non-sync position produces the same key).
// Determined at waveform init; typically 3–4.
// At 2 bits/symbol, 32 bits holds up to 16 symbols.
uint8_t hashDepth;

// List of (key, position, countdown) entries, one per sync position on this wheel.
// Sorted in wheel order so syncExpectedKeyIndex can advance with simple modulo.
uint8_t syncKeyCount;
sync_key_entry_s syncKeys[MAX_SYNC_POSITIONS];

// Gate constant: tooth is a glitch if e^2 > gateK * var.
// Default 9 (≈ 3-sigma for Gaussian noise).
uint8_t gateK = 9;

// Classification thresholds relative to predicted duration (all integer, no division).
// VERY_SHORT if actual * vshortDen < predicted * vshortNum  (< 0.4×)
// SHORT      if actual * shortDen  < predicted * shortNum   (< 0.7×)
// LONG       if actual * longDen   > predicted * longNum    (> 1.5×)
// Defaults produce 4 buckets encoded as 2-bit values 0–3.
uint8_t vshortThreshNum = 2;  uint8_t vshortThreshDen = 5;
uint8_t shortThreshNum  = 7;  uint8_t shortThreshDen  = 10;
uint8_t longThreshNum   = 3;  uint8_t longThreshDen   = 2;
```

### Tracker State — `TriggerDecoderBase`

Added to `TriggerDecoderBase`:

```cpp
// Innovation-variance tracker state.
int32_t  syncRateHat;     // r_hat: filtered normalized crank rate (ticks/angle-unit)
int32_t  syncRateDotHat;  // dr_hat: filtered rate derivative (acceleration)
uint32_t syncVar;         // running variance of innovations (EMA of e²)
bool     syncTrackerValid; // true once tracker has converged

// Rolling hash key: base-4, 2 bits per symbol, depth = triggerShape.hashDepth.
// Updated on every non-glitch tooth via bit-shift: no multiply needed.
uint32_t syncHashKey;
uint8_t  syncHashFillCount; // teeth classified since last sync loss; gates acquisition lookup

// Tracking-mode state.
uint8_t  syncExpectedKeyIndex; // index into syncKeys[] of the next expected sync position
uint8_t  syncCountdown;        // teeth remaining until that sync position is due
```

`toothDurations[]` remains `[GAP_TRACKING_LENGTH + 1]` — no size change needed since the tracker state replaces the need for deeper history.

---

## Precomputation (trigger_structure.cpp)

Two new helpers are called at the end of `initializeTriggerWaveform()`.

### `buildEventModel()` — angle reciprocals for every event

```cpp
void TriggerWaveform::buildEventModel() {
    int len = (int)getLength();
    for (int ev = 0; ev < len; ev++) {
        int prevEv = (ev == 0) ? len - 1 : ev - 1;
        angle_t gapAngleDeg = getSwitchAngle(ev) - getSwitchAngle(prevEv);
        if (gapAngleDeg <= 0) gapAngleDeg += getCycleDuration();

        uint16_t angleUnits = (uint16_t)efiRound(gapAngleDeg * 100.0f, 1.0f);
        eventModel[ev].angle = angleUnits;
        eventModel[ev].invAngleQ = (angleUnits > 0)
            ? (uint16_t)((1u << TRIGGER_SYNC_Q) / angleUnits)
            : 0;
    }
}
```

### `buildSyncKeyList()` — sync-position key list

Only processes the sync positions (those marked as `triggerShapeSynchPointIndex` or equivalent).  For each sync position, computes its key and verifies that no non-sync position produces the same key.  If there is a collision, `hashDepth` is incremented and all sync positions are re-keyed.

Nominal classification uses ratio of successive nominal gap angles (4 buckets, 2 bits each):
- `angle[k] * vshortDen < angle[k+1] * vshortNum` → VERY_SHORT (0)
- `angle[k] * shortDen  < angle[k+1] * shortNum`  → SHORT (1)
- `angle[k] * longDen   > angle[k+1] * longNum`   → LONG (3)
- else → NORMAL (2)

Key update is a pure left-shift: `key = (key >> 2) | (bucket << (2*(depth-1)))`.  32 bits holds up to 16 symbols.

```cpp
static uint32_t computeKey(const TriggerWaveform& w, int pos, int depth) {
    int len = (int)w.getLength();
    uint32_t key = 0;
    for (int d = 0; d < depth; d++) {
        int ev   = ((pos - d)     % len + len) % len;
        int prev = ((pos - d - 1) % len + len) % len;
        uint32_t aEv   = w.eventModel[ev].angle;
        uint32_t aPrev = w.eventModel[prev].angle;
        uint8_t bucket;
        if (aEv * w.vshortThreshDen < aPrev * w.vshortThreshNum) bucket = 0; // VERY_SHORT
        else if (aEv * w.shortThreshDen < aPrev * w.shortThreshNum) bucket = 1; // SHORT
        else if (aEv * w.longThreshDen  > aPrev * w.longThreshNum)  bucket = 3; // LONG
        else bucket = 2; // NORMAL
        key = (key >> 2) | ((uint32_t)bucket << (2 * (depth - 1)));
    }
    return key;
}

void TriggerWaveform::buildSyncKeyList() {
    int len = (int)getLength();

    // Max depth: 32 bits / 2 bits per symbol = 16 symbols.
    for (hashDepth = 3; hashDepth <= 16; hashDepth++) {
        // Compute candidate keys for each sync position.
        for (int s = 0; s < syncKeyCount; s++)
            syncKeys[s].key = computeKey(*this, syncKeys[s].pos, hashDepth);

        // Verify: no non-sync position produces one of these keys.
        bool unique = true;
        for (int pos = 0; pos < len && unique; pos++) {
            bool isSyncPos = false;
            for (int s = 0; s < syncKeyCount; s++)
                if (syncKeys[s].pos == pos) { isSyncPos = true; break; }
            if (isSyncPos) continue;

            uint32_t key = computeKey(*this, pos, hashDepth);
            for (int s = 0; s < syncKeyCount; s++)
                if (syncKeys[s].key == key) { unique = false; break; }
        }

        if (unique) break;
    }
    // If hashDepth > 16 and still not unique, set shapeDefinitionError.

    // Populate countdown for each sync position: teeth from this position
    // to the next one (wrapping), i.e. the inter-sync-point distance.
    for (int s = 0; s < syncKeyCount; s++) {
        int next = (s + 1) % syncKeyCount;
        int dist = syncKeys[next].pos - syncKeys[s].pos;
        if (dist <= 0) dist += len;
        syncKeys[s].countdown = (uint8_t)dist;
    }
}
```

The sync positions (`syncKeys[].pos`) are populated by `initializeTriggerWaveform()` from `triggerShapeSynchPointIndex` before calling `buildSyncKeyList()`.

Call both helpers at the end of `initializeTriggerWaveform()`.

---

## Runtime Pipeline — Per-Tooth Processing

The three layers run in sequence on every tooth event, before the existing sync-counter logic.  `isSyncPoint()` is replaced entirely by the hash lookup result.

### Step 1 — Predict

```cpp
// Current event index (0-based within trigger cycle, or running modulo if not yet synced).
int ev = currentEventIndex % triggerShape.getLength();
uint32_t invQ = triggerShape.eventModel[ev].invAngleQ;
uint32_t ang  = triggerShape.eventModel[ev].angle;

// Predicted duration from tracker state.
// r_hat is ticks-per-angle-unit.  dtPred is in raw ticks.
uint32_t dtPred = ((uint32_t)syncRateHat * ang) >> TRIGGER_SYNC_Q;
// (Only used when syncTrackerValid; otherwise fall through to bootstrap path.)
```

### Step 2 — Compute Innovation and Gate

```cpp
// Normalize the actual duration to rate units (same scale as syncRateHat).
uint32_t dtShifted = toothDurations[0] >> TRIGGER_SYNC_SHIFT;
int32_t  rMeas     = (int32_t)((dtShifted * invQ) >> TRIGGER_SYNC_Q);
int32_t  innov     = rMeas - syncRateHat;   // prediction error

// Gate: reject tooth as glitch if innovation² > gateK × syncVar.
// Integer approximation: compare |innov| > sqrt(gateK * syncVar).
// Avoid sqrt by squaring the gate: innov² > gateK * syncVar.
// Guard against overflow: cap innov before squaring.
const int32_t innovCap = 32767;
int32_t innovCapped = (innov > innovCap) ? innovCap : (innov < -innovCap) ? -innovCap : innov;
bool isGlitch = syncTrackerValid &&
                ((uint32_t)(innovCapped * innovCapped) > (uint32_t)triggerShape.gateK * syncVar);
```

### Step 3 — Update Tracker (non-glitch teeth only)

```cpp
if (!isGlitch) {
    // Alpha = 1/8 for rate, 1/16 for acceleration, 1/16 for variance.
    // These are independent knobs — tweak without affecting each other.
    syncRateDotHat += (innov - syncRateDotHat) >> 4;  // acceleration filter
    syncRateHat    += (innov >> 3) + (syncRateDotHat >> 4); // rate filter with accel feedforward

    // Variance: EMA of innovation².
    uint32_t innovSq = (uint32_t)(innovCapped * innovCapped) >> 4; // scale to prevent overflow
    syncVar += (innovSq - syncVar) >> 4;

    // Mark tracker valid after enough teeth.
    if (!syncTrackerValid && syncVar > 0) syncTrackerValid = true;
}
// Glitch: freeze all state.  dtPred remains from the previous step — still valid
// as a prediction for the *next* tooth (tracker state is unchanged).
```

### Step 4 — Classify and Update Rolling Hash Key (non-glitch teeth only)

```cpp
if (!isGlitch) {
    // 4-bucket classification relative to predicted duration.
    // All comparisons are integer multiply — no division.
    uint8_t bucket;
    uint32_t dt = toothDurations[0];
    if      (dt * triggerShape.vshortThreshDen < dtPred * triggerShape.vshortThreshNum)
        bucket = 0; // VERY_SHORT
    else if (dt * triggerShape.shortThreshDen  < dtPred * triggerShape.shortThreshNum)
        bucket = 1; // SHORT
    else if (dt * triggerShape.longThreshDen   > dtPred * triggerShape.longThreshNum)
        bucket = 3; // LONG
    else
        bucket = 2; // NORMAL

    // Roll the key: shift right by 2, insert new bucket at top.
    // Pure bit operations — no multiply, no modulo.
    int depth = triggerShape.hashDepth;
    syncHashKey = (syncHashKey >> 2) | ((uint32_t)bucket << (2 * (depth - 1)));

    if (syncHashFillCount < depth) syncHashFillCount++;
}
// Glitch: all state frozen — glitch tooth is invisible to pattern matcher.
```

### Step 5 — Sync Decision (acquisition vs tracking mode)

```cpp
bool isSynchronizationPoint = false;

if (!getShaftSynchronized()) {
    // --- Acquisition mode ---
    // Scan the short key list (typically 1–4 entries).
    if (syncTrackerValid && syncHashFillCount >= triggerShape.hashDepth) {
        for (int s = 0; s < triggerShape.syncKeyCount; s++) {
            if (syncHashKey == triggerShape.syncKeys[s].key) {
                isSynchronizationPoint = true;
                currentCycle.current_index = triggerShape.syncKeys[s].pos;
                // Arm the tracking countdown for the *next* sync position.
                syncExpectedKeyIndex = (s + 1) % triggerShape.syncKeyCount;
                syncCountdown = triggerShape.syncKeys[s].countdown;
                break;
            }
        }
    }
    // If tracker not yet valid, fall back to existing ratio-window bootstrap.
} else {
    // --- Tracking mode ---
    // Decrement countdown on every tooth (glitch or not — position still advances).
    if (syncCountdown > 0) {
        syncCountdown--;
    } else {
        // Expected sync position is now.  Check the rolling key.
        uint8_t idx = syncExpectedKeyIndex;
        if (syncHashKey == triggerShape.syncKeys[idx].key) {
            // Still in sync.  Reload for next sync position.
            syncExpectedKeyIndex = (idx + 1) % triggerShape.syncKeyCount;
            syncCountdown = triggerShape.syncKeys[idx].countdown - 1;
        } else {
            // Position has drifted — declare sync lost.
            setShaftSynchronized(false);
            syncHashFillCount = 0;  // force key re-fill before next acquisition
        }
    }
    // Position advances normally via nextTriggerEvent() — no other change.
}
```

The Miata NB special-case branch (`if (triggerType == TT_MIATA_VVT)`) is **removed** — the key list encodes the Miata geometry like any other wheel, and the tracker handles its kinematic oddities.

---

## Migration Plan

### Phase 1 — Precomputation infrastructure (no runtime behaviour change)

1. Add `TRIGGER_SYNC_Q`, `TRIGGER_SYNC_SHIFT` constants to `trigger_structure.h`.
2. Add `sync_event_model_s` struct and `eventModel[PWM_PHASE_MAX_COUNT]` to `TriggerWaveform`.
3. Add classification threshold fields (`shortThreshNum/Den`, `longThreshNum/Den`) and `gateK` to `TriggerWaveform`.
4. Implement `buildEventModel()` in `trigger_structure.cpp`; call from `initializeTriggerWaveform()`.
5. Add unit test: verify `eventModel[ev].angle` sum equals `getCycleDuration() * 100` for 36-2, 60-2, Miata.

### Phase 2 — Sync-position key list

1. Add `sync_key_entry_s` (with `countdown` field), `syncKeys[]`, `syncKeyCount`, `hashDepth` and classification threshold fields to `TriggerWaveform`.
2. Implement `buildSyncKeyList()` in `trigger_structure.cpp`; call from `initializeTriggerWaveform()` after `buildEventModel()`.  Include countdown population.
3. Populate `syncKeys[].pos` from `triggerShapeSynchPointIndex` in `initializeTriggerWaveform()`.
4. Add unit test: verify `hashDepth` and all keys are unique against all non-sync positions for all supported wheels; verify no wheel requires `hashDepth > 16`.

### Phase 3 — Tracker and gated filter in `TriggerDecoderBase`

1. Add tracker fields (`syncRateHat`, `syncRateDotHat`, `syncVar`, `syncTrackerValid`), key fields (`syncHashKey`, `syncHashFillCount`), and tracking fields (`syncExpectedKeyIndex`, `syncCountdown`) to `TriggerDecoderBase`.
2. Implement the 5-step per-tooth pipeline in `decodeTriggerEvent()`, gated behind `syncTrackerValid` for the hash lookup (ratio-window bootstrap kept active until tracker is valid).
3. Run full unit-test suite — both paths must produce correct sync for all wheels.

### Phase 4 — Remove ratio-window fallback

1. Once the tracker reliably converges within 1–2 revolutions in unit tests and log replay, remove the bootstrap ratio-window path.
2. Delete the `if (triggerType == TT_MIATA_VVT)` branch.
3. Delete `synchronizationRatioFrom[]`, `synchronizationRatioTo[]` from `TriggerWaveform` (or keep as deprecated for a release).

### Phase 5 — Validation

Validate against trigger log playback for:
- 36-2 under fast cranking acceleration (primary motivation)
- 60-2
- Miata NB VVT (must eliminate special-case branch)
- 36-2-2-2 / 60-2-2-2
- GM 24x
- Bosch quick-start
- 6-cylinder cranking with compression artifacts

---

## Open Questions

1. **Angle source**: `getSwitchAngle()` returns a float derived from `wave.getSwitchTime()`.  Confirm it returns cumulative angle from cycle start, not a gap width — the gap width is computed as the difference between successive entries, with wrap-around correction.

2. **Fixed-point overflow budget**: At cranking (100 RPM, 10° gap): `dt` ≈ 16.7M ticks.  After `TRIGGER_SYNC_SHIFT=4`: ≈ 1.04M.  Times `invAngleQ` for 10° (= 4096/1000 ≈ 4): ≈ 4.16M, within uint32.  Verify worst case: the slowest plausible crank speed and smallest gap angle on any supported wheel.

3. **Tracker filter coefficients**: Rate alpha=1/8, acceleration alpha=1/16, variance alpha=1/16 are initial proposals.  These should be validated empirically against log replay at cranking and at full-throttle transient.  Consider making them compile-time constants rather than runtime fields.

4. **`syncHashFillCount` bootstrap**: Rolling key is meaningless until `hashDepth` consecutive non-glitch teeth have been classified since last sync loss.  `syncHashFillCount` gates the acquisition lookup; it is cleared on sync loss but not on sync gain (key keeps rolling in tracking mode, ready for the next re-sync check).

5. **`eventModel` RAM cost**: `PWM_PHASE_MAX_COUNT` × 4 bytes per `TriggerWaveform` instance.  If `PWM_PHASE_MAX_COUNT = 720` this is 2880 bytes per instance.  Check if this fits given the number of `TriggerWaveform` instances (primary + VVT channels).  The key list itself is negligible (≤ 4 × 6 = 24 bytes including countdown).

6. **Wheels with non-unique sync patterns at depth 16**: In practice, all standard automotive wheels are resolved well below depth 8 with 4-bucket classification.  The unit test assertion catches any exception at build time.

7. **Tracker reset on sync loss**: Recommended: keep `syncTrackerValid` and tracker state valid (rate estimate stays useful for fast re-acquisition), clear only `syncHashFillCount` (forces key re-fill before next acquisition lookup).

8. **Countdown glitch behaviour**: The countdown decrements on every tooth including glitched ones, matching the position counter in `nextTriggerEvent()`.  This ensures `syncCountdown == 0` fires at the geometrically correct position regardless of which teeth were gated.

---

## Key Existing Code Locations

| Symbol | File | Line |
|--------|------|------|
| `isSyncPoint()` — **to be replaced** | `trigger_decoder.cpp` | 679 |
| `TT_MIATA_VVT` special branch — **to be deleted** | `trigger_decoder.cpp` | 685 |
| Ratio-window loop — **bootstrap, then delete** | `trigger_decoder.cpp` | 706 |
| `toothDurations` shift loop | `trigger_decoder.cpp` | 628 |
| `toothDurations` declaration | `trigger_decoder.h` | 138 |
| `synchronizationRatioFrom/To` — **deprecated** | `trigger_structure.h` | ~90 |
| `setTriggerSynchronizationGap3()` — **deprecated** | `trigger_structure.cpp` | 377 |
| `getSwitchAngle()` — used by `buildEventModel()` | `trigger_structure.cpp` | 367 |
| `getLength()` / `getSize()` | `trigger_structure.cpp` | — |
| 36-2 gap config | `trigger_structure.cpp` | 668–671 |
| Miata NB VVT config | `trigger_decoder.cpp` | 685 |
