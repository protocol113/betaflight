# Manual Home Override — MVP Plan (v3)

## Context

Today, Betaflight only sets the home point when the FC has a valid GPS fix
(≥ minSats) at the moment of arming, inside `GPS_reset_home_position()`. In a
training environment where the drone often can't acquire a fix on the ground
(covered launch, urban canyon, fast turnaround), this leaves two options:

1. The pilot can't arm with `gps_rescue` configured (`ARMING_DISABLED_GPS` is set), or
2. They arm with `gps_rescue_allow_arming_without_fix` and accept that GPS Rescue
   is disabled for that flight.

We want a third option: **pre-load a known-good home into the FC out-of-band** so
the drone can arm and have a sensible rescue target if its own GPS later acquires
a fix in flight.

The data path is a Lua script on the pilot's radio that captures a coord from a
donor drone (with a quality-gated fix) and beams it to receiving drones via MSP.
The donor performs a brief validation flight to kick off the session, then is shut
down. All other drones in the session use the same captured coord — unless any of
them happens to acquire its own ground fix before takeoff, in which case existing
Betaflight behavior takes over.

A single-slot persistent save on the radio allows the pilot to skip the
capture-and-validation flow on subsequent sessions at the same field, gated by a
confirmation prompt and backed by documented operational procedure.

## Hard safety boundaries (non-negotiable)

1. The worst case for any drone using this feature is identical to today's
   `gps_rescue_allow_arming_without_fix` worst case. If the manual home cannot
   be validated, the drone falls through to existing no-home behavior — drop on
   failsafe. We never fly a drone toward an unverified coordinate.
2. **No PG persistence of the manual home on the FC.** Captured coord is volatile
   on the FC and clears on every power cycle. Persistence exists only on the radio,
   in the script's storage.
3. **No FC-side bypass of `STATE(GPS_FIX)`** for arming, distance, or bearing math.
   The manual home only populates `GPS_home_llh`. Live-position math continues to
   require the FC's own fix.
4. **Manual home cannot be modified in flight.** The FC rejects any MSP write that
   arrives while armed.
5. **Rescue with manual home is OFF by default.** Gated behind a new opt-in CLI
   flag (`gps_rescue_allow_external_home`) which itself requires
   `gps_rescue_allow_arming_without_fix`.
6. **If the FC GPS never acquires its own fix during the flight, rescue is never
   enabled.** No-own-fix + failsafe = drop, same as today. This is the load-bearing
   wrong-field backstop.
7. Captured coords are session-scoped on the radio, with one exception: a single
   saved-slot allows loading a previously-validated coord at session start, gated
   by a hold-to-confirm warning that displays full lat/lon and last-validated age.
   The saved slot does not bypass the FC's first-fix promotion check.

## Background: how home flows today

Bytes hit the GPS UART → `gpsUpdate()` (`io/gps.c:1379`) parses NMEA/UBX into
`gpsSol`. Once `numSat > 3`, `gpsSetFixState(GPS_FIX)` runs. On arm,
`tryArm()` (`fc/core.c:518`) calls `GPS_reset_home_position()` (`io/gps.c:2591`),
which copies `gpsSol.llh` into `GPS_home_llh` and sets `STATE(GPS_FIX_HOME)`. The
OSD home arrow/distance and rescue both gate on `STATE(GPS_FIX_HOME)`. The
arming-disable flag `ARMING_DISABLED_GPS` clears in `updateArmingStatus()`
(`fc/core.c:383-396`) once `STATE(GPS_FIX) && numSat ≥ minSats`.

Critically, rescue uses **two** inputs: `GPS_home_llh` and `gpsSol.llh`. Setting
only home is necessary but not sufficient for rescue to function — the FC's own fix
is required for distance/bearing/IMU-yaw math.

## Session model

A "session" begins with the pilot establishing a session-active coord on the radio
(either via fresh capture-and-validation, or by loading the saved slot) and ends
when training is done. During the session:

- The donor drone is on briefly at the start (capture + validation flight only when
  doing a fresh capture), then powered down.
- Receiving drones are armed and flown using the session-active coord as their
  manual home.
- Any receiving drone that acquires its own ground fix before takeoff uses that fix
  instead (existing Betaflight behavior wins).
- The session-active coord lives on the radio, in volatile script memory, until the
  radio is powered off or the session is ended.

The validation flight by the donor (when performed) serves three purposes:

1. Confirms the captured coord is plausible (the donor flew with this coord as its
   rescue target without incident).
2. Confirms the field's RF and GPS conditions are workable for the session.
3. Gives the pilot a "this works today" signal before any other drones launch.

Per training procedure, a validation flight is required whenever the field changes.
The script enforces validation only on fresh captures; reuse via the saved slot is
gated by the load warning and operational procedure.

## FC-side state machine

Five states:

- **NO_HOME** — boot default. Standard arming gates apply.
- **MANUAL_HOME_PROVISIONAL** — Lua wrote a coord; FC has no own fix yet.
  `STATE(GPS_FIX_HOME)` is set. OSD shows a distinct indicator (e.g., `HOME: EXT`)
  plus the coord ID for pilot verification. Arming uses
  `gps_rescue_allow_arming_without_fix` semantics. Rescue is armed but inert:
  failsafe in this state = drop, not rescue.
- **MANUAL_HOME_VALIDATED** — FC acquired its own fix mid-flight; distance from
  current position to manual home is within `gps_rescue_max_home_distance`. Manual
  home is promoted; rescue is fully active. OSD switches to normal home indicator.
- **MANUAL_HOME_REJECTED** — FC acquired its own fix; manual home is too far away.
  `STATE(GPS_FIX_HOME)` is cleared, beeper fires, OSD shows `RESCUE OFF`. Rescue is
  unavailable for the rest of the flight.
- **NORMAL_HOME** — FC's own GPS acquired a fix on the ground before arming;
  manual home was overwritten or never loaded. Existing Betaflight behavior in full.

### Transitions

- `NO_HOME → MANUAL_HOME_PROVISIONAL`: Lua MSP message arrives, payload integrity
  passes, FC is disarmed.
- `MANUAL_HOME_PROVISIONAL → MANUAL_HOME_VALIDATED`: FC's own GPS reports
  `numSat ≥ gps_rescue_min_sats` AND `PDOP ≤ gps_rescue_max_pdop` for ≥3
  consecutive readings (counter resets if either condition fails mid-stream),
  AND distance from current `gpsSol.llh` to manual `GPS_home_llh`
  `≤ gps_rescue_max_home_distance`.
- `MANUAL_HOME_PROVISIONAL → MANUAL_HOME_REJECTED`: same fix conditions, distance
  `> gps_rescue_max_home_distance`.
- `MANUAL_HOME_PROVISIONAL → NORMAL_HOME`: FC's own GPS acquires fix on the ground
  before arming and existing `GPS_reset_home_position()` runs. Real fix overwrites
  manual home.
- Any state + Lua write while armed: rejected.
- `MANUAL_HOME_VALIDATED` + Lua write: rejected (home is locked for the flight).
- Any state + power cycle: → `NO_HOME`.
- `MANUAL_HOME_REJECTED` does not recover within the same flight. Promotion is a
  one-shot per arm.

### Interaction with `gps_set_home_point_once`

Real fix > manual home > no home. If `gps_set_home_point_once` is enabled and the
FC acquires a real ground fix on first arm, the real-fix home wins.

## New CLI parameters

| Parameter | Type | Default | Range |
|-----------|------|---------|-------|
| `gps_rescue_allow_external_home` | off/on | off | — (master enable; requires `gps_rescue_allow_arming_without_fix`) |
| `gps_rescue_max_home_distance` | meters | 1500 | 100–10000 |
| `gps_rescue_max_pdop` | ×10 | 25 (PDOP 2.5) | 10–50 |

## MSP protocol

New MSP V2 command ID for `MSP2_SET_EXTERNAL_HOME`. Payload (little-endian):

| Field | Type | Notes |
|-------|------|-------|
| `lat` | int32 | ×1e7, WGS84 |
| `lon` | int32 | ×1e7, WGS84 |
| `alt` | int32 | cm, MSL |
| `donor_pdop` | uint16 | ×10 |
| `donor_sat_count` | uint8 | |
| `capture_timestamp` | uint32 | Unix seconds |
| `coord_id` | uint16 | hash of lat/lon |
| `crc32` | uint32 | over preceding fields |

**FC validation on receipt:**

1. CRC32 must match.
2. Lat/lon in valid ranges (not 0/0, not obviously invalid).
3. `donor_sat_count ≥ 10`, `donor_pdop ≤ 20` (PDOP 2.0).
4. `capture_timestamp` within the last N hours (default 24, was 8 in earlier draft).
5. FC disarmed.
6. `gps_rescue_allow_external_home` enabled.
7. Current state is NOT `MANUAL_HOME_VALIDATED` (don't allow mid-flight retarget
   even if disarmed-then-rearmed inside same powered session — actually moot
   because state resets only on power cycle, but spell it out).

If all pass, FC writes `GPS_home_llh`, sets `STATE(GPS_FIX_HOME)`, transitions to
`MANUAL_HOME_PROVISIONAL`, acks with the `coord_id` echoed back. If any check
fails, FC nacks with a specific error code and changes no state.

`MSP_GET_HOME` (read-only) is also added for verification: returns current
`GPS_home_llh`, current state enum, current `coord_id` (or 0 if no manual home).

## Capture flow (Lua-side)

The donor drone is the capture source and the validation flight subject. After
capture and validation, it's powered down for the session.

### Fresh capture requirements

- `numSat ≥ 10`
- `PDOP ≤ 2.0`
- 10+ consecutive readings at ≥1 Hz with all positions within 2m of each other
  (stationarity).
- `groundSpeed ≤ 0.5 m/s` for the entire capture window. Hard requirement.
- Altitude variance during capture < 2m.

If any check fails, capture is rejected; pilot must restart it.

### Validation flight (mandatory after fresh capture)

After a fresh capture, the donor must perform a validation flight before the
captured coord is released for transmit to receiving drones. The Lua script
enforces this gate.

The validation flight is:

1. Donor takes off with the captured coord loaded as its own manual home (donor
   uses the same external-home path as receivers — eats its own dog food).
2. Donor flies a brief pattern (recommended: out 100m, return) and lands.
3. Pilot confirms via radio prompt that the flight was nominal — home arrow
   tracked, no rescue triggered, no anomalies.
4. Only after this confirmation does the Lua script unlock the captured coord for
   transmit to other drones, and the saved slot is updated with the new coord and
   a fresh last-validated timestamp.

This isn't a code-level safety guarantee — it's an operational gate. The pilot
can lie to the script. But it forces a "did this actually work today?" sanity
check before any other drone uses the coord.

If the pilot reports a failed validation, the candidate coord is discarded and a
fresh capture is required. The saved slot is **not** modified on a failed
validation.

### Coord ID and pilot verification

On successful capture, the Lua script computes a 16-bit hash of `(lat, lon)` and
displays it on the radio as a 4-digit number. The same ID is transmitted in the
MSP payload. When the FC accepts the coord, it displays the ID on the receiving
drone's OSD. Pre-arm verification is "do these match" — pilot reads the OSD and
confirms the radio's displayed ID matches before arming.

### Saved location on the radio (single-slot)

After a successful capture-and-validation cycle in a session, the Lua script saves
the validated location to a single persistent slot on the radio's SD card. Only
the most recent successfully-validated coord is retained.

The saved entry stores:

- Lat / lon (int32 ×1e7)
- Altitude (int32 cm)
- Capture timestamp
- Donor PDOP and sat count at capture
- Validation flight pass timestamp (when this coord was last *validated*, not just
  captured)

### Loading the saved location

At session start, the pilot can choose between two paths:

1. Fresh capture and validation (full flow above).
2. Load saved location — bypasses both capture and validation, loads the saved
   coord directly into the script's working memory as a session-active coord ready
   to transmit.

Path 2 is only available when the saved slot exists.

### The warning

Loading the saved location requires an active confirmation. Script displays:

```
⚠ Load saved home — no validation flight will be performed
Lat: 40.7128456
Lon: -74.0059821
Last validated: 3 days ago
Confirm this is the correct field for today's session.
Per training procedure, a validation flight is required when changing fields.

Type last 4 digits of coord ID (####) to confirm    [Cancel]
```

Implementation requirements:

- **Typed coord-ID confirmation, not button-press or hold-to-confirm.** Pilot must
  read the displayed coord ID and type its last 4 digits via the radio's input
  method. Resistant to muscle-memory tap-through. (Updated from earlier
  hold-to-confirm draft per validation feedback.)
- Lat/lon displayed as full numeric coords, not abbreviated, not represented by a
  name.
- Last-validated timestamp displayed prominently with relative phrasing
  ("3 days ago", "yesterday", "this morning").
- "Validation flight required when changing fields" line is explicit text in the
  prompt, not buried in documentation.

### After loading

Once loaded, the saved coord is used identically to a freshly-captured-and-validated
coord for the rest of the session. The script does not periodically re-prompt or
re-warn — the load confirmation is the gate.

The script does **not** update the saved slot's last-validated timestamp on a load.
The timestamp only updates when a fresh validation flight passes. A coord loaded
ten times in a row without re-validation still shows its true age every time the
warning appears.

### Saving after a fresh validation

After a successful capture-and-validation in a session, the script silently
overwrites the saved slot with the new coord and sets the last-validated timestamp
to now. No prompt — saving is automatic on validation pass.

### Manual clear

Lua script provides explicit "Clear saved home" command. Operationally useful when
(a) finished training at one field and want to ensure the next session starts from
capture, or (b) the radio is being handed to a different pilot.

A cleared slot stays cleared until the next successful validation. The cleared
state forces path 1 (fresh capture and validation) on the next session.

## Altitude reference

Source drone's `gpsSol.llh.altCm` is in WGS84 ellipsoidal at the donor's current
position. The donor must be on the ground during capture (enforced by stationarity
check). Documented caveats:

- Capture must include altitude at ground level, not in flight.
- Receivers should treat manual-home altitude as approximate — typical GPS altitude
  error is 5–15m and propagates into rescue's descent target.
- If a receiver's baro is enabled and zeroed at takeoff, baro-derived altitude will
  dominate during descent and partially mask GPS altitude error.

## Transport reliability

- Lua transmits with up to N retries (default 3) until FC ack with matching
  `coord_id`.
- No ack after retries: script displays transmit failure; receiving drone is not
  marked configured.
- FC ignores duplicate sends with the same `coord_id` (idempotent).
- New `coord_id` while in `MANUAL_HOME_PROVISIONAL`: accept the most recent.
- New `coord_id` while in `MANUAL_HOME_VALIDATED`: rejected.

## IMU yaw convergence — explicit gate (added per validation feedback)

Without magnetometer, COG-derived yaw requires ~30–50m of forward flight at
≥1–2 m/s before it converges. A rescue triggered before yaw is converged will
fly the wrong direction for tens of seconds.

**Mitigation:** Rescue activation while in `MANUAL_HOME_VALIDATED` requires
`yaw_converged == true` (a flag the IMU/PosHold layer can already expose, or that
we add by tracking COG-stability over the last N samples). If a failsafe fires
before yaw is converged, the drone falls through to drop instead of rescue —
identical worst-case to hard rule #6.

This gate does NOT apply when a magnetometer is healthy (mag-derived yaw doesn't
need forward flight).

## Issue #13016 (`allow_arming_without_fix` interaction)

Betaflight issue #13016 documents a dangerous bug class where
`gps_rescue_allow_arming_without_fix` combined with GPS Rescue as failsafe action
and ELRS RX timing causes unintentional rescue activation at boot. Since
`gps_rescue_allow_external_home` requires the existing flag to be on, we inherit
this blast radius.

**Decision: ship-blocking.** Issue #13016 must be either fixed or its trigger
conditions reliably worked around before this feature ships. Inheriting an
unintentional-rescue-at-boot bug while explicitly requiring the feature flag that
triggers it is not acceptable.

## Blackbox logging

- Initial home source (`NONE`, `REAL_FIX`, `MANUAL`).
- State machine transitions with timestamps.
- Validation distance at promotion or rejection.
- Donor PDOP and sat count from MSP payload.
- `coord_id`.
- IMU yaw convergence state at any rescue trigger.

## OSD indicators

- `MANUAL_HOME_PROVISIONAL`: distinct icon (e.g., `HOME: EXT`) plus `coord_id`
  displayed.
- `MANUAL_HOME_VALIDATED`: standard home indicator.
- `MANUAL_HOME_REJECTED`: warning icon with `RESCUE OFF` text.
- Yaw not converged + rescue would otherwise be active: warning string.

## What we lose without a live ground anchor (residual risks)

1. Session-scoped session-active coord — within a session, the coord can't be
   changed without going through capture or load.
2. Validation flight on fresh captures.
3. Coord ID OSD verification.
4. FC's first-fix promotion check (the load-bearing in-flight check).
5. Typed coord-ID confirmation on saved-slot load.

The single-slot saved location feature allows loading a previously-validated coord
without re-validation. Wrong-field-from-yesterday case is mitigated by:

- Typed coord-ID confirmation displaying full lat/lon and age.
- Documented procedure requiring re-validation on field changes.
- The FC's first-fix promotion check.

**Fields located within `gps_rescue_max_home_distance` of each other are not
protected by the promotion check and rely entirely on procedure.** This is the
irreducible residual risk of this feature as designed.

The first-fix backstop (hard rule #6) prevents wrong-field flyaways for drones
that never acquire their own fix in flight — they drop on failsafe rather than fly
toward the wrong coord. Bad-but-recoverable; not a flyaway.

## Files to touch

| File | Change |
|------|--------|
| `src/main/io/gps.h` | new state enum `manualHomeState_e`; `extern manualHomeState_e gpsManualHomeState`; coord_id storage |
| `src/main/io/gps.c` | state machine implementation; `gpsManualHomeState` definition; promotion logic in `onGpsNewData()` (consecutive-fix counter, distance check); reject-on-arm of pending writes |
| `src/main/flight/gps_rescue.h` / `gps_rescue_multirotor.c` | new config (`max_home_distance`, `max_pdop`, `allow_external_home`); rescue gate on state==VALIDATED && yaw_converged |
| `src/main/fc/core.c` | extend `updateArmingStatus()` to factor in manual-home state for the GPS arming check (uses existing `allow_arming_without_fix` path, no new bypass) |
| `src/main/msp/msp.c` & `msp_protocol_v2_*.h` | implement `MSP2_SET_EXTERNAL_HOME` with full validation; `MSP_GET_HOME` |
| `src/main/osd/osd_elements.c` (or osd_warnings.c) | new strings: `HOME: EXT` w/ coord_id, `RESCUE OFF`, `YAW NOT READY` |
| `src/main/io/beeper.h` / `.c` | new entries for state transitions (manual loaded, promoted, rejected) |
| `src/main/blackbox/blackbox.c` | event types for state transitions |
| `src/main/sensors/compass.h` / IMU layer | expose `yawConverged` flag if not already exposed |

## TDD workflow

Existing gtest harnesses we'll extend:

- `arming_prevention_unittest.cc` — for arming-status interactions.
- `flight_failsafe_unittest.cc` — for rescue gate (state + yaw_converged).
- `osd_unittest.cc` — for new strings.
- `blackbox_unittest.cc` — for new event types.

New harnesses we'll create:

- `gps_manual_home_unittest.cc` — state machine transitions in isolation.
- `msp_external_home_unittest.cc` — MSP payload validation (CRC, ranges, age,
  donor quality, disarmed gate).

Per-milestone discipline: red, green, refactor; full `make test` before moving on;
each milestone shippable as its own PR.

Hardware-only behaviors (OSD pixels, beeper audio, blackbox bytes on flash) get
unit-test coverage at the **state/decision layer** (does the state flag flip, does
the beeper enum get queued, does the blackbox event-emit get called) rather than
the I/O layer.

## Milestones

Each milestone is shippable on its own. Order matters: M0 is a prerequisite outside
this feature; M1–M3 wire up the read/write/state primitives; M4 adds the
load-bearing promotion logic; M5 gates rescue; M6–M8 are polish and visibility;
M9 is the radio-side script.

### M0 — Prerequisite: Issue #13016 resolved or worked around
Must land before M5 ships. Not a code task in this feature, but ship-blocking per
hard rule #5 / explicit decision above. Owner: GPS Rescue maintainer.

### M1 — `MSP_GET_HOME` (read-only visibility)
**Goal:** Configurator/Lua can read current home + state without changing anything.
**Tests first** (new `msp_external_home_unittest.cc`):
- Returns current `GPS_home_llh`, state enum, `coord_id` byte-correct.
- Returns `coord_id=0` when state is `NO_HOME`.
**Code:** opcode + handler in `msp/msp.c`; expose state enum.
**Done when:** unit tests green; Configurator MSP shell round-trips.

### M2 — State machine skeleton
**Goal:** State enum, transitions, no behavior change yet.
**Tests first** (new `gps_manual_home_unittest.cc`):
- Boot → `NO_HOME`.
- Power cycle from any state → `NO_HOME`.
- `arming` event in any non-NO_HOME state preserves state.
**Code:** define `manualHomeState_e`; `gpsManualHomeState`; init at boot.

### M3 — `MSP2_SET_EXTERNAL_HOME` with full validation
**Goal:** Lua write transitions FC `NO_HOME → MANUAL_HOME_PROVISIONAL`.
**Tests first** (`msp_external_home_unittest.cc`):
- Bad CRC → nack, no state change.
- Lat=lon=0 → nack.
- Out-of-range lat/lon → nack.
- `donor_sat_count < 10` → nack.
- `donor_pdop > 20` → nack.
- `capture_timestamp` older than 24h → nack.
- FC armed → nack.
- `gps_rescue_allow_external_home` off → nack.
- Already in `MANUAL_HOME_VALIDATED` → nack.
- All checks pass → `GPS_home_llh` written, `STATE(GPS_FIX_HOME)` set, state →
  `MANUAL_HOME_PROVISIONAL`, ack with `coord_id`.
- Duplicate `coord_id` while in PROVISIONAL → idempotent ack.
- New `coord_id` while in PROVISIONAL → accept most recent.
**Code:** opcode, validation chain, state transition.

### M4 — First-fix promotion (load-bearing)
**Goal:** When FC GPS comes up in flight, promote to VALIDATED or REJECTED.
**Tests first** (`gps_manual_home_unittest.cc`):
- PROVISIONAL + 3 consecutive good fixes within `max_home_distance` → VALIDATED.
- PROVISIONAL + 3 consecutive good fixes beyond `max_home_distance` → REJECTED.
- PROVISIONAL + 2 good fixes then 1 bad (sat dip) → counter resets, no transition.
- PROVISIONAL + good fix but PDOP > `max_pdop` → no transition.
- VALIDATED → REJECTED transition does NOT occur (one-shot per arm).
- REJECTED → any state does NOT occur (one-shot).
- New CLI params: `max_home_distance` (default 1500), `max_pdop` (default 25).
**Code:** consecutive-fix counter in `onGpsNewData()`; transition logic; CLI
fields.

### M5 — Rescue gating (rescue-active iff state==VALIDATED && yaw_converged)
**Goal:** Rescue only fires when we're sure we know where home is AND can navigate
to it.
**Tests first** (`flight_failsafe_unittest.cc`):
- Failsafe + state=NO_HOME → drop.
- Failsafe + state=PROVISIONAL → drop (rescue armed but inert per rule #6).
- Failsafe + state=VALIDATED + `yaw_converged=true` → rescue fires.
- Failsafe + state=VALIDATED + `yaw_converged=false` → drop.
- Failsafe + state=REJECTED → drop.
- Failsafe + state=NORMAL_HOME → existing rescue fires (unchanged).
- New CLI param: `gps_rescue_allow_external_home` (default off, requires
  `gps_rescue_allow_arming_without_fix`).
**Code:** rescue init gate; `yaw_converged` plumbing if not already exposed;
CLI flag with cross-validation.

### M6 — Arming-status integration
**Goal:** `ARMING_DISABLED_GPS` clears when manual home is loaded AND
`gps_rescue_allow_arming_without_fix` is on AND
`gps_rescue_allow_external_home` is on. No new bypass code path — uses existing
`allow_arming_without_fix` semantics.
**Tests first** (`arming_prevention_unittest.cc`):
- All three flags+state aligned → `ARMING_DISABLED_GPS` cleared.
- Manual home loaded but `allow_arming_without_fix` off → arming refused
  (existing semantics, regression guard).
- Manual home loaded but `allow_external_home` off → arming refused.
**Code:** likely no changes to `updateArmingStatus()` — verify the existing
`allow_arming_without_fix` path already does the right thing once
`STATE(GPS_FIX_HOME)` is set by the MSP path. If it does, this milestone is
purely tests confirming the integration.

### M7 — OSD indicators
**Goal:** Pilot sees state on screen.
**Tests first** (`osd_unittest.cc`):
- State=PROVISIONAL → warnings include `HOME: EXT ####` (with coord_id).
- State=VALIDATED → standard home indicator, no manual warning.
- State=REJECTED → warnings include `RESCUE OFF`.
- VALIDATED + yaw not converged → warnings include `YAW NOT READY`.
**Code:** new warning strings; coord_id formatted into the OSD warning system.

### M8 — Beeper + blackbox markers
**Goal:** Audible + logged feedback for state transitions.
**Tests first:**
- Beeper enum queued on each transition (PROVISIONAL entry, VALIDATED, REJECTED).
- Blackbox event emitted on each transition with timestamp + relevant fields.
**Code:** new beeper entries; new blackbox event types.

### M9 — Lua reference script (separate repo)
**Goal:** Field-usable capture/validate/beam/save/load workflow.
**Manual test plan only:**
- Two FCs on the bench: capture from A (with simulated good fix), require
  validation flight on A, beam to B, verify B's OSD coord_id matches radio
  display, verify B's MSP_GET_HOME state==PROVISIONAL.
- Save slot: validation passes → slot updated. Reload slot → typed-confirm
  flow. Wrong code rejects. Correct code loads. Slot last-validated timestamp
  unchanged on load.
- Manual clear: slot empty afterward; next session forces fresh capture.
**Code:** ~150 lines Lua targeting EdgeTX; not in this repo.

## Detailed todo list

### M0 — Prerequisite
- [ ] Coordinate with Betaflight maintainers on issue #13016 status.
- [ ] If unfixed: define exact trigger conditions, document workaround, and gate
      M5 release on either fix landing or workaround being verified.

### M1 — `MSP_GET_HOME`
- [ ] Create `src/test/unit/msp_external_home_unittest.cc` skeleton.
- [ ] Failing test: empty state → returns NO_HOME, coord_id=0.
- [ ] Failing test: with home set externally (mocked), returns matching bytes.
- [ ] Define `manualHomeState_e` enum in `io/gps.h` (just the type, not yet
      driven anywhere).
- [ ] Add `MSP_GET_HOME` opcode in MSP V2 betaflight header.
- [ ] Implement handler in `mspProcessOutCommand()`.
- [ ] `make test` green.
- [ ] Bench: Configurator MSP shell round-trip.

### M2 — State machine skeleton
- [ ] Create `src/test/unit/gps_manual_home_unittest.cc` skeleton.
- [ ] Failing test: boot → NO_HOME.
- [ ] Failing test: simulated power cycle → NO_HOME.
- [ ] Define `gpsManualHomeState` global; init in `gpsInit()`.
- [ ] No transitions yet — just the enum and storage.
- [ ] `make test` green.

### M3 — `MSP2_SET_EXTERNAL_HOME`
- [ ] Failing test: bad CRC → nack.
- [ ] Failing test: lat=lon=0 → nack.
- [ ] Failing test: out-of-range coords → nack.
- [ ] Failing test: low sat count → nack.
- [ ] Failing test: high PDOP → nack.
- [ ] Failing test: stale timestamp (>24h) → nack.
- [ ] Failing test: armed → nack.
- [ ] Failing test: feature flag off → nack.
- [ ] Failing test: VALIDATED state → nack.
- [ ] Failing test: all pass → state=PROVISIONAL, GPS_home_llh set, ack with
      coord_id.
- [ ] Failing test: duplicate coord_id while PROVISIONAL → idempotent ack.
- [ ] Failing test: new coord_id while PROVISIONAL → updates.
- [ ] Add MSP V2 opcode.
- [ ] Implement validation chain in handler.
- [ ] Wire transition from NO_HOME to PROVISIONAL.
- [ ] Add `gps_rescue_allow_external_home` CLI parameter.
- [ ] `make test` green.

### M4 — First-fix promotion
- [ ] Add `gps_rescue_max_home_distance` (default 1500) and
      `gps_rescue_max_pdop` (default 25) CLI params.
- [ ] Failing test: 3 consecutive good fixes within distance → VALIDATED.
- [ ] Failing test: 3 consecutive good fixes beyond distance → REJECTED.
- [ ] Failing test: counter resets on bad sample.
- [ ] Failing test: PDOP gate works.
- [ ] Failing test: VALIDATED is one-shot (no degradation).
- [ ] Failing test: REJECTED is one-shot (no recovery).
- [ ] Implement consecutive-fix counter and gating in `onGpsNewData()`.
- [ ] Implement distance check and transitions.
- [ ] `make test` green.

### M5 — Rescue gating
- [ ] Confirm or implement `yaw_converged` exposure from IMU/PosHold layer.
- [ ] Failing test: NO_HOME + failsafe → drop.
- [ ] Failing test: PROVISIONAL + failsafe → drop.
- [ ] Failing test: VALIDATED + yaw_converged + failsafe → rescue.
- [ ] Failing test: VALIDATED + !yaw_converged + failsafe → drop.
- [ ] Failing test: REJECTED + failsafe → drop.
- [ ] Failing test: `allow_external_home` requires `allow_arming_without_fix`
      (CLI cross-validation rejects bad combo).
- [ ] Modify rescue init in `flight/gps_rescue_multirotor.c` to gate on state
      and yaw.
- [ ] CLI cross-validation between the two flags.
- [ ] `make test` green.
- [ ] M0 must be resolved before merging this milestone.

### M6 — Arming-status integration
- [ ] Failing test: state=PROVISIONAL + flags on → ARMING_DISABLED_GPS cleared.
- [ ] Failing test: state=PROVISIONAL but allow_arming_without_fix off → blocked.
- [ ] Failing test: state=PROVISIONAL but allow_external_home off → blocked.
- [ ] Verify `STATE(GPS_FIX_HOME)` set by M3 path → existing
      `allow_arming_without_fix` semantics already work; tests confirm.
- [ ] If existing path needs adjustment, do minimum required.
- [ ] `make test` green.

### M7 — OSD indicators
- [ ] Failing test: PROVISIONAL → "HOME: EXT ####" string present.
- [ ] Failing test: VALIDATED → standard home, no manual warning.
- [ ] Failing test: REJECTED → "RESCUE OFF" present.
- [ ] Failing test: VALIDATED + !yaw_converged → "YAW NOT READY" present.
- [ ] Add new strings to OSD warnings handler.
- [ ] Format coord_id into the warning text.
- [ ] `make test` green; bench-confirm OSD.

### M8 — Beeper + blackbox markers
- [ ] Failing test: PROVISIONAL entry queues beeper enum X.
- [ ] Failing test: VALIDATED entry queues beeper enum Y.
- [ ] Failing test: REJECTED entry queues beeper enum Z.
- [ ] Failing test: each transition emits blackbox event with type + relevant
      payload.
- [ ] Add beeper enum entries and sequences.
- [ ] Add blackbox event types.
- [ ] Hook into transition function.
- [ ] `make test` green.

### M9 — Lua reference script
- [ ] Build capture screen (gates: sat≥10, PDOP≤2.0, 10-sample stationarity,
      groundspeed ≤ 0.5 m/s, alt variance < 2m).
- [ ] Build validation-flight gate (post-capture confirm prompt).
- [ ] Build transmit screen (MSP send with retry, OSD coord_id readback prompt).
- [ ] Build single-slot save (auto on validation pass).
- [ ] Build load screen with typed coord-ID confirmation.
- [ ] Build "Clear saved home" command.
- [ ] Two-FC bench test pass.

## Open questions for review

1. `gps_rescue_max_home_distance` default of 1500m: confirm against actual session
   range data.
2. Stationarity thresholds during capture (2m position spread, 0.5 m/s groundspeed,
   2m altitude variance): conservative starting points; tune via field testing.
3. Capture-timestamp validity window: 24h proposed (was 8h in earlier draft).
   Should saved-slot loads have a hard expiration, or rely on procedure +
   typed-confirmation? Currently no hard expiration on saved slots.
4. `yaw_converged` exposure: confirm whether existing IMU/PosHold layer already
   tracks this, or whether M5 needs to add it.
5. Blackbox event-type IDs: assign without colliding with existing types.

## Out of scope for MVP

- Donor-as-live-anchor validation (donor stays offline after validation flight).
- Radio-mounted GPS for independent ground reference.
- Multiple saved location slots.
- Code-enforced revalidation when loading a saved slot.
- Multi-drone broadcast / mesh validation.
- ESP-NOW backpack integration.
- Position hold or any nav mode beyond standard Betaflight rescue.
- Persisting manual home across reboots on the FC.

## End-to-end verification (bench)

1. **Cold-no-fix arm with manual home**: bench FC indoors, no GPS antenna. Send
   valid `MSP2_SET_EXTERNAL_HOME`. State→PROVISIONAL, OSD shows `HOME: EXT
   ####`. With both flags on, arm switch accepted. Failsafe → drops (rule #6
   regression).
2. **Provisional → Validated**: same as 1, then connect antenna, wait for fix.
   State→VALIDATED, OSD switches to standard home indicator, beeper fires.
3. **Provisional → Rejected**: send manual home far from antenna's fix location.
   State→REJECTED, OSD shows `RESCUE OFF`, beeper distinct from validate beep.
4. **Yaw-not-converged drop**: VALIDATED state, induce failsafe before COG
   convergence (or with mag disabled, very early in flight). Drop, not rescue.
5. **Power-cycle clears**: any non-NO_HOME state → power cycle → NO_HOME, OSD
   plain.
6. **In-flight write rejected**: arm, send `MSP2_SET_EXTERNAL_HOME` mid-flight.
   Nack, no state change.
7. **VALIDATED write rejected**: in VALIDATED state, send a different
   `coord_id`. Nack.
8. **Bad payloads**: each individual validation rule (CRC, range, sats, PDOP,
   timestamp, flags) rejected with appropriate nack code.
9. **CLI cross-validation**: try to enable `allow_external_home` without
   `allow_arming_without_fix`. CLI rejects.
10. **Lua loop**: two FCs on the bench. Capture from A, validation-flight
    confirm, beam to B. Verify coord_id matches between radio and B's OSD,
    and B's MSP_GET_HOME returns matching coords + state=PROVISIONAL.
11. **Saved-slot reload**: validate-and-save in one session. Cycle radio. New
    session, load saved slot, type wrong coord_id → reject. Type correct →
    load. Slot's last-validated timestamp unchanged after load.
