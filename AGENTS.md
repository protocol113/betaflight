# AGENTS.md — protocol113/betaflight

A permanent private fork of Betaflight. It carries custom firmware features for
a drone training fleet, iterated fast and often, on top of an upstream that keeps
moving. Nothing here is ever sent upstream as a pull request.

Fork infrastructure (this file, `CLAUDE.md`, fork scripts) lives on `fork/base`.
Edit it there.

## How the operator uses this fork

- Features exist to support a training environment. Some are one-off
  experiments, some are kept for years, and nobody knows in advance which.
- Every feature must survive upstream moving underneath it. Upstream opens a new
  release line about twice a year and patches the current one continuously.
- The operator flashes one firmware image per fleet target. That image must
  contain exactly the set of features currently wanted: an abandoned experiment
  is removed without touching the others.

Those three needs produce one requirement, and the branch model exists to meet
it: **every feature is independently rebaseable, removable, and buildable
together with the rest.**

## Branch model (hard rule)

A **topic branch** holds one feature's commits and nothing else. The
**integration branch** combines topic branches into the firmware that gets
flashed; it is rebuilt, never edited.

| Branch | Role | How it is written |
|---|---|---|
| `fork/base` | Upstream sync point plus fork infrastructure. Tracks the current upstream release line. | Rebased onto upstream. Infrastructure commits only. |
| `fork/main` | Integration branch. The only branch that is built and flashed. | Rebuilt from `fork/base` plus merges of topic branches. Receives no direct commits. |
| `feat/<name>` | Topic: a feature intended to be kept. | Cut from `fork/base`. Commits for this topic only. |
| `exp/<name>` | Topic: an experiment. Deletable at any time without ceremony. | Same as `feat/`. Promote by renaming to `feat/` when it earns a place. |
| `fix/<name>` | Topic: a fix to upstream code that the fork carries. | Same as `feat/`. |

Invariants. Restore any of these before continuing other work:

1. `git log fork/base..<topic>` shows only that topic's commits.
2. `fork/main` is reproducible from `fork/base` plus a list of topic branches.
   Every commit reachable from `fork/main` is reachable from `fork/base` or
   from a topic branch, so discarding and rebuilding `fork/main` loses nothing.
3. Infrastructure changes land on `fork/base` and reach topic branches by
   rebase.
4. Upstream is read-only. The `upstream` remote's push URL is disabled and
   stays disabled. Everything is pushed to `origin`.

Why topic-on-base rather than one long feature chain: features chained onto one
another can be neither dropped nor updated separately. That is how this fork
ended up with the MGRS work stranded on the 2025.12 line and the paralyze work
split across two branches that were never joined.

## Which upstream to follow

`fork/base` tracks the **current upstream release maintenance branch** (today
`upstream/2026.6-maintenance`), not `upstream/master`. Maintenance branches
receive fixes for the version people fly; `master` is the unreleased next
version. Moving `fork/base` to a new release line (when `2026.12-maintenance`
opens, say) is a task the operator schedules; run the sync procedure with the
new branch as the target.

## Procedures

### Start a topic

1. `git fetch upstream`. If `fork/base` is behind its upstream branch, sync
   first.
2. `git switch -c <kind>/<name> fork/base`.
3. Commit atomically: one concern per commit, conventional-commit subject, body
   explains why, `Refs #N` naming the GitHub issue. Every change traces to an
   issue on `protocol113/betaflight`; open one when none exists.
4. `git push -u origin <kind>/<name>`.

Done when the branch builds for a fleet target, its unit tests pass, and
invariant 1 holds.

### Sync with upstream

1. `git fetch upstream`.
2. `git branch fork/base-prev fork/base` — a marker for where topics were cut.
3. `git rebase upstream/<release>-maintenance fork/base`.
4. For each topic branch being kept:
   `git rebase --onto fork/base fork/base-prev <topic>`.
   Conflicts here belong to exactly one feature; resolve them in that feature's
   terms and rerun that topic's tests before moving to the next.
5. `git branch -D fork/base-prev`.
6. Rebuild `fork/main` (below).
7. Push every rebased branch: `git push --force-with-lease origin <branch>`.

Done when `fork/main` builds for every fleet target and `make test` passes.

### Rebuild fork/main

1. `git switch -C fork/main fork/base` — recreates the branch from base; the
   previous `fork/main` is discarded (invariant 2 makes this safe).
2. `git merge --no-ff <topic>` for each topic on the include list. Default
   include list: every `feat/*` and `fix/*` branch on `origin`; `exp/*` only
   when the operator names it.
3. Build every fleet target. Run `make test`.
4. `git push --force-with-lease origin fork/main`.

Done when builds and tests are green. The include list is recorded by the merge
commits themselves: `git log --merges --first-parent fork/main`.

### Retire or promote an experiment

- Retire: `git branch -D exp/<name>`, `git push origin --delete exp/<name>`,
  rebuild `fork/main`.
- Promote: `git branch -m exp/<name> feat/<name>`, push the new name, delete
  the old remote branch.

### Build and flash

- `make <config-target>` (for example `make ACCIF435`). Output lands in `obj/`.
  `make help`, `make targets`, `make test_help` describe the rest.
- Fleet targets, confirm with the operator before changing: `ACCIF435`
  (AT32F435G).

## Betaflight code checklist

Gotchas the build does not confess until a target you did not compile fails.

1. **`mk/source.mk`** is an explicit source list. Add every new
   `src/main/**/*.c` there or FC target builds fail to link while unit tests
   pass.
2. **Feature guards.** GPS, OSD, GPS_RESCUE, MAG, WING and more are
   conditionally compiled. Mirror the `#ifdef` pattern of the area you are in.
   `CRAZYBEEF4SX1280` (STM32F411, no `USE_GPS`) and `STM32F4DISCOVERY`
   (`USE_WING`) are the CI canaries; build them before calling a change done.
3. **`USE_WING` parallel struct.** `gpsRescueConfig_t` is defined twice:
   `pg/gps_rescue_multirotor.{h,c}` and `pg/gps_rescue_wing.{h,c}`. A field
   added to one is added to the other, or every call site is guarded.
4. **CLI registration** takes three places: the PG struct field and default in
   `pg/<area>.{h,c}`, a `PARAM_NAME_<NAME>` macro in
   `src/main/fc/parameter_names.h`, and a `valueTable[]` entry in
   `src/main/cli/settings.c`. OSD elements additionally need their
   `osd_<element>_pos` entry in `valueTable[]` — commit `af26d2e06` is the fix
   for having missed it.
5. **MSP V2 opcodes** live in `src/main/msp/msp_protocol_v2_betaflight.h`.
   Take the next free number and document the payload in a comment beside it.
6. **Unit tests.** `src/test/Makefile` has a per-test `<name>_SRC :=` list; a
   new test needs its own entry. `src/test/unit/vtx_msp_unittest.cc` is a
   small clean template. Stale coverage state can segfault a rerun: `find
   obj/test/<test_name> -name '*.gcda' -delete`.

## Topics carried

- **MGRS OSD element** — `lib/main/mgrs/`, `src/main/osd/osd_elements.c`,
  guard `USE_GPS_MGRS`, CLI `osd_gps_mgrs_pos`. Branch `feat/mgrs-osd-2026.6`.
- **Post-failsafe quarantine** — after a genuine link-loss failsafe has
  disarmed the aircraft, count down and latch Paralyze unless the link properly
  recovers. Issues #4–#11, label `failsafe-quarantine`. Landed: #4 on
  `fix/msp-failsafe-config-bounds`, #6 on `feat/rc-mode-internal-latch` (the
  latch has no caller yet). #5 and #7 are the next work.

## Migration to this model (delete this section when complete)

The branch model above was adopted 2026-08-29. Outstanding cleanup:

- `feat/mgrs-osd-stable`, `chore/catchup-2025.12.6`, `feat/nurkkala-port`:
  stranded on the 2025.12 line with no MGRS commits. Delete once the operator
  confirms nothing unique is on them.
- `feat/failsafe-quarantine`: an empty placeholder (no commits past the MGRS
  tip). Delete.
- `feat/mgrs-osd-2026.6`, `fix/msp-failsafe-config-bounds`,
  `feat/rc-mode-internal-latch`: cut from `upstream/2026.6-maintenance`, not
  from `fork/base`. Rebase each onto `fork/base` (`git rebase fork/base
  <topic>`) and rename `feat/mgrs-osd-2026.6` to `feat/mgrs-osd`.
- Commit `af26d2e06` on the MGRS branch contains a self-referential symlink
  named `tools` at the repo root. Drop it during the rebase.
- Create `fork/main` for the first time via Rebuild.
