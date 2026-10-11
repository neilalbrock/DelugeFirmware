# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Community firmware for the Synthstrom Deluge (sequencer/synth/sampler). Bare-metal C/C++ (C++23) with some assembly, no OS, targeting a Renesas RZ/A1L (Cortex-A9, 400MHz, 3MB on-chip SRAM + 64MB SDRAM). Hardware ships with either an OLED or a 7-segment display — every change must work on both.

## Commands

All tooling goes through DBT (`./dbt`, or `dbt.cmd` on Windows), which must be run from the repo root. On first run it downloads the full toolchain (arm-none-eabi GCC, clang-format/clang-tidy, CMake, Ninja, Python) into `toolchain/` — nothing is installed system-wide.

```bash
./dbt build release          # or debug / relwithdebinfo; no arg builds debug+release. Output in build/<Config>/
./dbt build -c debug         # clean first
./dbt configure -DENABLE_SYSEX_LOAD=YES   # pass CMake options (other example: -DENABLE_MATRIX_DEBUG=YES)
./dbt format                 # run clang-format exactly as CI does — use this rather than a system clang-format
./dbt test                   # configure (once) + build + run host unit tests in build/tests via ctest
./dbt test -n                # build tests only
./dbt loadfw release         # upload firmware to a device (see `./dbt loadfw -h`)
./dbt --help                 # list all tasks (scripts/tasks/task-*.py)
```

Tests are host-built, not on-target, and are a separate CMake project rooted at `tests/` (`./dbt test` uses the Ninja Multi-Config generator, so binaries land under `build/tests/<subdir>/Debug/`):
- `tests/unit/` — CppUTest, the main suite. Run one group/test with CppUTest flags: `build/tests/unit/Debug/UnitTests -g LFOTest` / `-n <testName>`, `-v` for verbose.
- `tests/spec/` — CppSpec specs (`*_spec.cpp`, auto-discovered; driver at `build/tests/spec/Debug/all_specs`).
- `tests/32bit_unit_tests/` — memory allocator / small-pointer tests; only built on Linux (need `-m32`).
- `tests/qemu/` — ARMv7 specs (fixed-point/NEON) cross-compiled and run under QEMU in CI.

Unit tests do **not** link the whole firmware. `tests/unit/CMakeLists.txt` explicitly lists both the test files and each firmware `.cpp` under test; anything else must be stubbed in `tests/unit/mocks/`. When adding a test, add the test file to `add_executable(UnitTests ...)` and the source(s) under test to `deluge_SOURCES`. `IN_UNIT_TESTS=1` is defined for test builds.

Pre-commit (`.pre-commit-config.yaml`) runs clang-format, ruff (Python), and actionlint.

## Architecture

Top-level `src/`: `main.c`/`resetprg.c` (boot), `RZA1/` (vendor HAL), `OSLikeStuff/` (task scheduler, timers, fault handler), `fatfs/`, `NE10/`, `definitions_cxx.hpp` (global constants, enums incl. `enum class Error`). Firmware proper is in `src/deluge/`.

**Execution model.** `deluge_main()` in `src/deluge/deluge.cpp` initialises hardware and then `startTaskManager()`. There are no threads: work runs as cooperative repeating tasks registered via `addRepeatingTask(fn, priority, minInterval, targetInterval, maxInterval, name, resource)` (see the block around `deluge.cpp:530`) — audio routine, USB MIDI, encoders, buttons/pads, playback, UI rendering, SD cluster loading, etc. Tasks must return quickly; long-running work blocks audio. Tasks declare resources (e.g. `RESOURCE_SD`, `RESOURCE_USB`) to avoid conflicting access.

**Audio.** `processing/engines/audio_engine.cpp` (`AudioEngine::routine`) renders audio in blocks: it iterates the song's `Output`s, which render `Voice`s through DSP in `dsp/` (oscillators, filters, reverb, delay, timestretch, granular, DX7 engine, etc.). Much DSP is fixed-point (`util/fixedpoint.h`) and some uses NEON. Performance matters: CPU-heavy changes (>3% sustained load) need a runtime or compile-time switch.

**Model.** `model/song/` — `Song` owns `Clip`s (`model/clip/`: `InstrumentClip`, `AudioClip`) and `Output`s (`model/output.h`; instruments in `model/instrument/`: synth/`SoundInstrument`, `Kit` of `Drum`s, MIDI, CV, plus `AudioOutput`). `InstrumentClip`s hold `NoteRow`s of `Note`s. `playback/playback_handler.cpp` drives timing/sync (internal clock, MIDI clock, ticks).

**ModelStack** (`model/model_stack.h` — read the big comment at the top). Instead of passing Song/Clip/NoteRow/ParamManager etc. as separate arguments, functions receive a stack-allocated `ModelStack` that is progressively extended (`ModelStackWithTimelineCounter` → `...WithNoteRow` → `...WithParamCollection` → `...WithAutoParam`, etc.). Older code predates it and doesn't use it everywhere.

**Params & modulation.** `modulation/params/` — `ParamManager` holds `ParamCollection`s (patched params, unpatched params, expression, MIDI); `AutoParam` carries automation. `modulation/patch/` implements the patch-cable mod matrix. LFOs, envelopes, arpeggiator and automation also live in `modulation/`. Undo/redo is via `ActionLogger` and `Consequence` objects in `model/action/` and `model/consequence/`.

**UI.** `gui/ui/ui.h` — a stack of `UI` objects (`openUI`, `changeRootUI`, `getCurrentUI`); button/pad/encoder input from `hid/` is dispatched to the current UI. Root views (session, arranger, instrument clip, keyboard, automation, performance, audio clip) are in `gui/views/`. `gui/ui/sound_editor.cpp` + `gui/menu_item/` implement the menu system; menu item instances are defined globally in `gui/ui/menus.cpp`. `hid/display/` abstracts OLED vs 7-segment (`display->haveOLED()`).

**Strings / l10n.** All user-visible strings go through `gui/l10n/`: add an id to `strings.h` (`enum class String`), text in `english.json`, and a ≤4-char abbreviation in `seven_segment.json` if it differs. `g_english.cpp` / `g_seven_segment.cpp` are generated by `generate.py` during the build — never edit them.

**Storage.** `storage/` — songs, presets (synths, kits) and settings are XML on the SD card (`Serializer`/`Deserializer`, `storage_manager.cpp`); there is also a JSON (de)serializer and `flash_storage.cpp` for persistent device settings. Sample audio is streamed from SD in clusters (`storage/cluster/`, `storage/audio/`).

**Memory.** No general heap — allocation goes through `GeneralMemoryAllocator` (`memory/`) with distinct regions (internal SRAM vs external SDRAM, small/large). Use `allocMaxSpeed`/`allocLowSpeed` (`memory/memory_allocator_interface.h`) or the provided allocators rather than raw `malloc`. "Stealable" memory (e.g. cached sample clusters) can be reclaimed under pressure. Compiled with `-fno-rtti`; avoid `dynamic_cast`/`typeid`.

## Conventions and requirements

- Match existing style (`.clang-format`: tabs, 120 cols, LLVM-based). Old code uses `goto` and single-return heavily; new C++ should prefer early returns and normal control flow.
- Changes to how users interact with the device need a toggle. Add a **runtime feature setting** (Community Features menu): enum in `model/settings/runtime_feature_settings.h` (before `MaxElement`) → l10n strings → `SetupOnOffSetting(...)` in `RuntimeFeatureSettings::init()` → `SettingToggle` in `gui/menu_item/runtime_feature/settings.cpp` → guard with `runtimeFeatureSettings.isOn(RuntimeFeatureSettingType::X)`. Changes that grow image size >5% need a `FEATURE_` preprocessor switch.
- Song/preset XML and flash config changes must stay upward-compatible (old files load or are upgraded) and ideally not break older firmware reading newer files.
- User-facing changes need an entry in the community features doc. Docs under `docs/` are mostly stubs now; the real sources live in the Astro site under `website/src/content/docs/` (e.g. `features/community_features.mdx`, `development/deluge/guidelines.md`).
- UI changes should follow `website/src/content/docs/development/deluge/ux_principles.md`.
- Project policy (`docs/CONTRIBUTING.md`): agents must not open PRs, comment on PRs, or write PR descriptions — leave those to the human contributor.
