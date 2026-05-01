# Betaflight — session memory for Claude

Working notes for any future session on `protocol113/betaflight`.

## CI / PR workflow

- **GitHub PR webhook subscriptions only fire on FAILURE and on review activity.** They do NOT fire on success. After every `git push`, do **not** rely on the webhook to tell you when CI has finished. Wait ~3 minutes, then proactively call `mcp__github__pull_request_read get_check_runs` to confirm the run is green. Don't tell the user "watching for CI" — that implies passive notification, which doesn't happen for the success case.
- The user's view of GitHub may also lag — if the user says "I'm not seeing anything," re-poll the API and report the actual state.
- The `CI / Complete` umbrella check fails whenever any individual `CI / Build (...)` job fails. They share a root cause; investigating one is investigating both.
- The build matrix has fail-fast enabled. When one target fails, all the other targets show `cancelled` rather than their real conclusion. After fixing a failure, you don't actually know the others passed until the next clean run.
- Per the task brief: develop on the designated branch (`claude/...`), push with `-u origin <branch>`, and create the PR as a draft. The user's repo MCP scope is restricted to `protocol113/betaflight` only — do not attempt to call other repos.
- The PR's base branch should be `2025.12-maintenance` (the maintenance branch tracking the 2025.12.X releases), NOT `master` (which is the dev tip for the next release).
- After `git push`, always proactively check (don't wait for webhook): start a Bash with `run_in_background` and an `until` loop polling check_runs, or just sleep a couple minutes and poll.

## Adding code to Betaflight — checklist before committing

1. **`mk/source.mk`** — explicit source list, NOT a wildcard. Any new `src/main/**/*.c` must be added here or FC target builds (SITL, STM32 family, etc.) fail to link, even if unit tests pass.
2. **`#ifdef USE_GPS` (and friends)** — many feature areas of the codebase (GPS, OSD, GPS_RESCUE, MAG, WING) are conditionally compiled. Mirror the existing guard pattern of the area you're adding to. CRAZYBEEF4SX1280 (STM32F411) builds without `USE_GPS` and is a good canary for missed guards.
3. **`USE_WING` parallel struct** — `gpsRescueConfig_t` has TWO definitions: `pg/gps_rescue_multirotor.{h,c}` (under `#ifndef USE_WING`) and `pg/gps_rescue_wing.{h,c}` (under `#ifdef USE_WING`). Anything added to the multirotor struct must either also be added to the wing struct, OR be guarded with `#ifndef USE_WING` at every call site. STM32F4DISCOVERY is the `USE_WING` canary in CI.
4. **CLI registration** — adding a PG field is not enough for `set <name> = X` to work in the CLI. Three places must be touched:
   - PG struct field + default in `pg/<area>.{h,c}`
   - `PARAM_NAME_<NAME>` macro in `src/main/fc/parameter_names.h`
   - `valueTable[]` entry in `src/main/cli/settings.c`
   - Optional: matching `BLACKBOX_PRINT_HEADER_LINE` in `src/main/blackbox/blackbox.c` for parity with similar params.
5. **MSP V2 Betaflight opcodes** live in `src/main/msp/msp_protocol_v2_betaflight.h`. Last allocated as of writing is `0x300F` (MSP2_SET_EXTERNAL_HOME). Allocate the next number, document the payload in a comment.
6. **Tests** — `src/test/Makefile` has per-test `<name>_SRC := \` lists. New tests need their own entry there. The unit-test build needs `libblocksruntime-dev` and `libclang-rt-18-dev` on the host (CI has them; a fresh dev env may not).
7. **Coverage profile state** can corrupt across re-runs and produce segfaults at startup. Fix with `find obj/test/<test_name> -name '*.gcda' -delete` then re-run.
8. **Test pattern** — `src/test/unit/vtx_msp_unittest.cc` is a clean small example to copy. Use `extern "C" { ... }` to bring in firmware headers and stub out symbols the unit doesn't link.

## Project specifics for the Manual Home Override feature

- Branch: `claude/home-point-gps-flow-eEoJK`
- PR: protocol113/betaflight#2 (draft)
- Design doc: `docs/manual-home-override.md`
- Module: `src/main/io/gps_home.{h,c}`
- New MSP opcodes: `MSP2_GET_HOME = 0x300E`, `MSP2_SET_EXTERNAL_HOME = 0x300F`
- New CLI flag: `gps_rescue_allow_external_home` (off by default)
- Wire format chose CRC16-CCITT over CRC32 (smaller, sufficient for 21-byte payload, reuses `common/crc.c` infrastructure).
- Milestones M1 + M3 landed. M0 (issue #13016 prereq, ship-blocking for M5), M4 (first-fix promotion — load-bearing safety), M5–M8 still pending. M9 lives in a separate radio-side repo.
