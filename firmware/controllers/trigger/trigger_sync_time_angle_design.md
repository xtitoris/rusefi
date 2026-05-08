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

### Layer 1 — Innovation-Variance Tracker (every tooth, unified)

Maintains `tHat` (EMA of normalized crank rate: ticks per angle-unit, Q12, right-shifted by `TRIGGER_SYNC_SHIFT`) and `tDotHat` (EMA of rate trend, same units).  These units are meaningful in both modes:

- **Pre-sync**: the angle is unknown, so the prediction uses `normalAngle` (average tooth pitch, precomputed once from wheel geometry) — `dtPred = (tHat + tDotHat) × normalAngle >> Q`.  `tDotHat` tracks the compression-stroke ramp in rate units, so `dtPred` already expects the next tooth to be longer during deceleration.  No position knowledge required.
- **Post-sync**: the prediction uses `eventModel[ev].angle` for the exact upcoming tooth — `dtPred = (tHat + tDotHat) × eventModel[ev].angle >> Q`.  Full per-tooth precision; the same formula, just with a different angle constant.

The two modes share identical update equations.  The only difference is which angle constant (`normalAngle` vs `eventModel[ev].angle`) feeds the prediction and normalization.  Both constants are selected once at the top of the tooth handler and passed through as `useAngle`/`useInvQ`; **zero branching in the update code itself**.

`syncVar` (EMA of $e^2$) is in rate-units² throughout both modes and requires **no reset at sync transition**.

### Layer 2 — Gated Innovation Filter (every tooth)

The current tooth's prediction error $e = t_{\text{meas}} - dtPred$ is tested against the gate $e^2 > k \cdot \text{var}$ before the tracker is updated.  If the gate fires the tooth is declared a glitch: tracker state is frozen and the measurement is discarded.

The critical improvement over the existing code: the existing sync ratio uses `toothDurations[1]` (the immediately preceding tooth) as its denominator.  If that tooth was a sensor glitch the denominator is corrupted and the resulting ratio is garbage.  With the tracker, `dtPred` is computed from the frozen pre-glitch state — the corrupted tooth is completely invisible to the sync check.

The gate threshold `gateK × syncVar` adapts automatically: it widens during transients (large innovations from acceleration or compression push `syncVar` up) and narrows at steady state.  For non-uniform wheels before sync, unresolved geometry variation inflates `syncVar`, appropriately widening the gate.

### Layer 3 — Tracker-Enhanced Ratio Detection

The sync check retains the existing ratio-window structure (`synchronizationRatioFrom/To[]`) and all tooth-count conditions in `isSyncPoint()`.  The only change is the **denominator**:

| | Denominator | Problem |
|---|---|---|
| **Existing** | `toothDurations[1]` — raw previous tooth | Carries speed error from acceleration and compression |
| **Proposed** | `dtPred_raw` — tracker prediction of a normal tooth | Kinematically corrected; compression ramp absorbed by `tDotHat` |

`dtPred_raw = (tHat + tDotHat) << TRIGGER_SYNC_SHIFT` converts the shifted prediction back to raw ticks for the ratio comparison.

When the tracker has not yet converged (`syncTrackerValid = false`, first partial revolution), the original `toothDurations[1]` denominator is used as fallback, preserving full backward compatibility.

This architecture:
- Handles all wheel types with a single code path — no per-wheel branches.
- Is robust to compression artifacts: `tDotHat` tracks the compression ramp in rate units; `dtPred` already expects stretched intervals before the sync gap.
- Is robust to single-tooth glitches: the tracker denominator uses frozen pre-glitch state; the corrupted tooth is invisible to the ratio check.
- Requires no per-wheel tuning beyond the existing ratio windows.
- Minimal new state: 4 decoder fields total (`tHat`, `tDotHat`, `syncVar`, `syncTrackerValid`); no transition logic, no reset at sync.

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

### Gate Constant and Normal-Pitch Constants — `TriggerWaveform`

```cpp
// Gate constant: tooth is a glitch if e² > gateK × syncVar.
// Default 9 (≈ 3-sigma for Gaussian noise).
uint8_t gateK = 9;

// Average tooth pitch across the full wheel cycle, in 0.01° units.
// = getCycleDuration() * 100 / getLength()
// Used as the pre-sync angle constant: same units as eventModel[ev].angle.
uint16_t normalAngle;

// Precomputed reciprocal of normalAngle: (1 << TRIGGER_SYNC_Q) / normalAngle.
// Used as the pre-sync normalization constant: same units as eventModel[ev].invAngleQ.
uint16_t normalInvQ;
```

### Tracker State — `TriggerDecoderBase`

Added to `TriggerDecoderBase`:

```cpp
// Unified tracker: tHat is normalized rate (ticks/angle-unit, Q12, >>TRIGGER_SYNC_SHIFT).
// Pre-sync:  angle-unit = normalAngle  (average tooth pitch)
// Post-sync: angle-unit = eventModel[ev].angle  (exact per-tooth pitch)
// Same field, same units, no conversion at sync transition.
int32_t  tHat;             // EMA of crank rate in Q12 fixed-point, >>SHIFT
int32_t  tDotHat;          // EMA of rate trend (captures compression ramp and acceleration)
uint32_t syncVar;          // EMA of e² in rate-units² (continuous across sync transition)
bool     syncTrackerValid; // true once tracker has seen enough teeth (~4)
```

`toothDurations[]` remains `[GAP_TRACKING_LENGTH + 1]` — no size change.

---

## Precomputation (trigger_structure.cpp)

One new helper is called at the end of `initializeTriggerWaveform()`.

### `buildEventModel()` — angle reciprocals for every event

Computes per-event angle constants and the average-pitch constants used before sync.

```cpp
void TriggerWaveform::buildEventModel() {
    int len = (int)getLength();
    uint32_t totalAngle = 0;
    for (int ev = 0; ev < len; ev++) {
        int prevEv = (ev == 0) ? len - 1 : ev - 1;
        angle_t gapAngleDeg = getSwitchAngle(ev) - getSwitchAngle(prevEv);
        if (gapAngleDeg <= 0) gapAngleDeg += getCycleDuration();

        uint16_t angleUnits = (uint16_t)efiRound(gapAngleDeg * 100.0f, 1.0f);
        eventModel[ev].angle    = angleUnits;
        eventModel[ev].invAngleQ = (angleUnits > 0)
            ? (uint16_t)((1u << TRIGGER_SYNC_Q) / angleUnits)
            : 0;
        totalAngle += angleUnits;
    }
    // Average pitch: used pre-sync when position is unknown.
    normalAngle = (uint16_t)(totalAngle / len);
    normalInvQ  = (normalAngle > 0)
        ? (uint16_t)((1u << TRIGGER_SYNC_Q) / normalAngle)
        : 0;
}
```

Call at the end of `initializeTriggerWaveform()`.

---

## Runtime Pipeline — Per-Tooth Processing

The three layers run in sequence on every tooth event, before the existing sync-counter logic.  The ratio check in `isSyncPoint()` is preserved but its denominator is replaced with the tracker prediction.

### Step 0 — Select angle constants (one branch, everything downstream is branch-free)

```cpp
uint16_t useAngle, useInvQ;
if (getShaftSynchronized()) {
    int ev = currentCycle.current_index;
    useAngle = triggerShape.eventModel[ev].angle;
    useInvQ  = triggerShape.eventModel[ev].invAngleQ;
} else {
    // Pre-sync: use average pitch.  No position knowledge needed.
    useAngle = triggerShape.normalAngle;
    useInvQ  = triggerShape.normalInvQ;
}
```

### Step 1 — Predict (identical for both modes)

```cpp
// tHat is normalized rate (Q12, >>SHIFT).  Multiply by the selected angle to get
// predicted tooth duration in the same shifted-ticks units.
int32_t tBase  = tHat + tDotHat;
int32_t dtPred = (int32_t)(((uint32_t)tBase * useAngle) >> TRIGGER_SYNC_Q);
// Pre-sync:  tBase × normalAngle   >> Q  → predicted average-pitch tooth duration
// Post-sync: tBase × exact angle   >> Q  → predicted this-tooth duration
```

### Step 2 — Compute Innovation and Gate (identical for both modes)

```cpp
int32_t tMeas = (int32_t)(toothDurations[0] >> TRIGGER_SYNC_SHIFT);
int32_t innov = tMeas - dtPred;

const int32_t innovCap = 32767;
int32_t innovCapped = (innov >  innovCap) ?  innovCap
                    : (innov < -innovCap) ? -innovCap
                    : innov;

bool isGlitch = syncTrackerValid &&
                ((uint32_t)(innovCapped * innovCapped) > (uint32_t)triggerShape.gateK * syncVar);
```

### Step 3 — Update Tracker (identical for both modes)

```cpp
if (!isGlitch) {
    // Normalize tMeas to rate units using the selected reciprocal.
    // Pre-sync:  tMeas × normalInvQ >> Q  → rate estimate assuming average-pitch tooth
    // Post-sync: tMeas × invAngleQ  >> Q  → rate estimate from exact tooth geometry
    int32_t rMeas  = (int32_t)(((uint32_t)tMeas * useInvQ) >> TRIGGER_SYNC_Q);
    int32_t rInnov = rMeas - tBase;   // innovation in rate units

    const int32_t rCap = 32767;
    int32_t rInnovC = (rInnov >  rCap) ?  rCap
                    : (rInnov < -rCap) ? -rCap
                    : rInnov;

    tDotHat += (rInnovC - tDotHat) >> 4;         // trend alpha = 1/16
    tHat    += (rInnovC >> 3) + (tDotHat >> 4);  // rate alpha = 1/8 with trend feedforward

    uint32_t rInnovSq = (uint32_t)(rInnovC * rInnovC) >> 4;
    syncVar += (rInnovSq - syncVar) >> 4;

    if (!syncTrackerValid && syncVar > 0) syncTrackerValid = true;
}
// Glitch: all state frozen.  dtPred from Step 1 remains the best prediction for next tooth.
```

### Step 4 — Ratio Sync Check

```cpp
// Build the denominator for the existing ratio windows.
// Tracker predicts the "normal" (average-pitch) tooth duration in raw ticks.
// This is the kinematically-corrected replacement for toothDurations[1].
uint32_t denominator;
if (syncTrackerValid) {
    // tBase × normalAngle >> Q gives predicted normal-tooth duration >> SHIFT;
    // shift back to raw ticks.
    uint32_t tPredNorm = (uint32_t)(((uint32_t)tBase * triggerShape.normalAngle)
                                    >> TRIGGER_SYNC_Q);
    denominator = tPredNorm << TRIGGER_SYNC_SHIFT;
} else {
    denominator = toothDurations[1];  // fallback: first partial revolution
}

// Pass denominator into the existing ratio-window check.
// All other conditions (tooth count, consecutive-teeth checks) are unchanged.
bool isSynchronizationPoint = checkRatioWindows(toothDurations[0], denominator)
                            && <existing_tooth_count_conditions>;
// No transition code needed: tHat/tDotHat are already in rate units valid post-sync.
```

The Miata NB special-case branch (`if (triggerType == TT_MIATA_VVT)`) can be removed once the tracker is validated on that wheel — the kinematic correction handles its acceleration-sensitive geometry identically to any other missing-tooth wheel.

---

## Migration Plan

### Phase 1 — Precomputation infrastructure (no runtime behaviour change)

1. Add `TRIGGER_SYNC_Q`, `TRIGGER_SYNC_SHIFT` constants to `trigger_structure.h`.
2. Add `sync_event_model_s` struct and `eventModel[PWM_PHASE_MAX_COUNT]` to `TriggerWaveform`.
3. Add `gateK`, `normalAngle`, `normalInvQ` fields to `TriggerWaveform`.
4. Implement `buildEventModel()` in `trigger_structure.cpp`; call from `initializeTriggerWaveform()`.
5. Add unit test: verify `eventModel[ev].angle` sum equals `getCycleDuration() * 100` for 36-2, 60-2, Miata NB, Subaru 6+7.

### Phase 2 — Add tracker fields to `TriggerDecoderBase`

1. Add `tHat`, `tDotHat`, `syncVar`, `syncTrackerValid` to `TriggerDecoderBase`.
2. Initialize all fields to zero in the reset path.

### Phase 3 — Implement 4-step pipeline in `decodeTriggerEvent()`

1. Implement Steps 0–4; hook into `isSyncPoint()` by passing the tracker-derived `denominator`.
2. Gate the new denominator behind `syncTrackerValid`; the original `toothDurations[1]` fallback is used until the tracker converges (first partial revolution).
3. No transition code at sync detection — `tHat`/`tDotHat` are valid in rate units from the first tooth.
4. Run full unit-test suite — all wheels must produce correct sync at steady speed.

### Phase 4 — Validate under transients

1. Validate against trigger log playback with fast cranking acceleration (primary motivation).
2. Validate Miata NB VVT — confirm tracker absorbs its kinematic signature without special-casing.
3. Validate Subaru 6+7 cam wheel — confirm tracker converges and ratio check fires despite non-uniform geometry pre-sync.
4. Once validated, remove the `if (triggerType == TT_MIATA_VVT)` special-case branch.

### Phase 5 — Remove deprecated ratio infrastructure (optional, later release)

1. Delete `synchronizationRatioFrom[]`, `synchronizationRatioTo[]` from `TriggerWaveform`, replacing with the fields used by the new pipeline.
2. Delete `setTriggerSynchronizationGap3()` or keep as a deprecated wrapper.

---

## Open Questions

1. **Angle source**: `getSwitchAngle()` returns a float derived from `wave.getSwitchTime()`.  Confirm it returns cumulative angle from cycle start — the gap width is computed as the difference between successive entries with wrap-around correction at the cycle boundary.

2. **Fixed-point overflow budget**: At cranking (100 RPM, 10° gap): raw `dt` ≈ 16.7M ticks.  After `>> SHIFT=4`: `tMeas` ≈ 1.04M.  `tMeas * normalInvQ (=4)` ≈ 4.16M `>> Q=12` = 1016: `tHat` ≈ 1000, fits in int32.  Denominator reconstruction: `tHat × normalAngle (=1000) >> 12 × 16` = 1000×1000/4096×16 ≈ 3.9M ticks, fits in uint32.  Verify worst case: slowest plausible crank speed on the lowest-tooth-count wheel.

3. **Tracker convergence before the sync gap**: `syncTrackerValid` gates the improved denominator.  On 36-2 there are 34 normal teeth before the gap — ample.  On short cam triggers (4 teeth) the tracker may not converge; the `toothDurations[1]` fallback handles this.  Mitigation: unconditionally update `tHat` on tooth 0 (skip gate check on first tooth) to set `syncTrackerValid` sooner.

4. **Tracker filter coefficients**: Rate alpha=1/8, trend alpha=1/16, variance alpha=1/16 are initial proposals.  Validate empirically against log replay at cranking (fast acceleration) and full-throttle transient.  Consider making them compile-time constants.

5. **`eventModel` RAM cost**: `PWM_PHASE_MAX_COUNT` × 4 bytes per `TriggerWaveform` instance.  If `PWM_PHASE_MAX_COUNT = 720` this is 2880 bytes per instance.  Check against the number of `TriggerWaveform` instances (primary + VVT channels) and available SRAM.  If RAM is tight, `eventModel` can be dropped and the post-sync tracker degrades to using `normalAngle` for all teeth (same as pre-sync) — still better than the current code.

6. **Tracker state on sync loss**: Keep all tracker fields valid across sync loss — rate estimate remains useful for fast re-acquisition.  The only thing that changes at sync loss is `getShaftSynchronized()` returning false, which switches `useAngle`/`useInvQ` back to `normalAngle`/`normalInvQ` at Step 0.

7. **Non-uniform cam wheels (Subaru 6+7)**: Pre-sync, geometry variation inflates `syncVar`, widening the gate.  This is correct but the sync gap must still exceed the widened gate.  Verify the Subaru cam sync gap ratio is large enough to fire even with an inflated `syncVar`.  If marginal, consider not applying the gate to the ratio check itself (gate only the tracker update) on cam wheels.

---

## Key Existing Code Locations

| Symbol | File | Line |
|--------|------|------|
| `isSyncPoint()` — **denominator to be replaced** | `trigger_decoder.cpp` | 679 |
| `TT_MIATA_VVT` special branch — **to be deleted post-validation** | `trigger_decoder.cpp` | 685 |
| Ratio-window loop — **kept; denominator replaced** | `trigger_decoder.cpp` | 706 |
| `toothDurations` shift loop | `trigger_decoder.cpp` | 628 |
| `toothDurations` declaration | `trigger_decoder.h` | 138 |
| `synchronizationRatioFrom/To` — **kept short-term; deprecated Phase 5** | `trigger_structure.h` | ~90 |
| `setTriggerSynchronizationGap3()` — **deprecated Phase 5** | `trigger_structure.cpp` | 377 |
| `getSwitchAngle()` — used by `buildEventModel()` | `trigger_structure.cpp` | 367 |
| `getLength()` / `getSize()` | `trigger_structure.cpp` | — |
| 36-2 gap config | `trigger_structure.cpp` | 668–671 |
