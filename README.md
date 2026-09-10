# 4D Spring Reverb (CVA Labs) — VST3 Spring Reverb with a 4D Lattice Spring Model

A physical-modeling spring reverb where the spring itself is a **4D Lattice Spring Model (LSM)**:
a mass–spring lattice whose rest geometry is a **helix in 4 dimensions**
(3 spatial axes x/y/z + a coiling axis "w"). The full mathematical description is in
`DESIGN.md`.

## Download (Windows & macOS)

Ready-made builds (VST3 + Standalone) are published on the
[GitHub Releases](https://github.com/cva-labs/4dspringreverb/releases) page:

- **Windows**: `4DSpringReverb-<version>-Windows-VST3.zip`, `...-Windows-Standalone.zip`
- **macOS** (universal, Apple Silicon + Intel): `4DSpringReverb-<version>-macOS-VST3.zip`, `...-macOS-Standalone.zip`

Installation:
- **Windows VST3**: unzip and copy the whole `4D Spring Reverb.vst3` folder to
  `C:\Program Files\Common Files\VST3`.
- **Windows Standalone**: unzip `4D Spring Reverb.exe` anywhere and run it.
- **macOS VST3**: unzip and copy the `4D Spring Reverb.vst3` folder to
  `/Library/Audio/Plug-Ins/VST3`.
- **macOS Standalone**: unzip and move `4D Spring Reverb.app` to
  `/Applications`. The macOS binaries are unsigned/not notarized — on first launch,
  right-click → Open (or run `xattr -cr "/Applications/4D Spring Reverb.app"` if Gatekeeper
  blocks the launch).

## Building from source (Windows / Visual Studio)

Requirements: CMake ≥ 3.22, Visual Studio (C++ workload), git.
JUCE 8.0.6 is downloaded automatically via FetchContent the first time.

```powershell
cmake -S . -B build
cmake --build build --config Release --target Spring4DReverb_VST3 --parallel
```

Artifacts (the plugin shows up in hosts as **4D Spring Reverb** by **CVA Labs**):
- VST3: `build/Spring4DReverb_artefacts/Release/VST3/4D Spring Reverb.vst3`
- Standalone: `build/Spring4DReverb_artefacts/Release/Standalone/4D Spring Reverb.exe`

Install: copy the whole `4D Spring Reverb.vst3` folder to
`C:\Program Files\Common Files\VST3` (or whichever VST3 folder your host scans).

## Validation harness (console, no JUCE)

```powershell
cmake --build build --config Release --target SpringTest --parallel
.\build\Release\SpringTest.exe
```

It runs an impulse / noise burst / sine / stability sweep, writes
`spring4d_impulse.wav`, `spring4d_burst_L.wav`, `spring4d_burst_R.wav`,
`spring4d_sine.wav` and reports stability + CPU cost.

## Parameters

| Slider | Description |
|---|---|
| Drive | Gain + soft-clip (tanh) at the spring input |
| Tension | Axial stiffness — wave speed / brightness |
| Decay | Time to decay to −60 dB (0.4–12 s) |
| 4D Coupling | Coupling strength along the 4th w axis (coiling/twist) |
| Pickup Pos | Position of the 2nd pickup (right channel) along the spring |
| Brightness | Pickup low-pass (700 Hz–12 kHz) |
| Spring Size | Number of segments/rings (24–96) — also affects CPU |
| Mix | Dry/Wet |
| Output | Final gain (dB) |
| Anti-FB | Multiband spectral resonance suppressor — per band (8 octaves, 160 Hz–10 kHz) it detects when the energy runs away from its rolling reference or an absolute threshold and ducks it accordingly; keeps the level constant across frequencies; fully transparent at 0 |

Output is stereo: L = pickup near the free end, R = moving pickup.
Input is mono (or summed stereo).

## Architecture

- `Source/SpringLSM.h` — DSP engine (header-only, JUCE-independent)
- `Source/PluginProcessor.*` — JUCE AudioProcessor + AudioProcessorValueTreeState
- `Source/PluginEditor.*` — GUI (9 knobs + live visualization)
- `Source/SpringVisualizer.h` — real-time 4D spring visualization
  (helix projection, w→color, kinetic glow, pickup/transducer markers)
- `assets/background.png` — CVA LABS skin (1504×1046 landscape; the 9 knobs
  centered on the skin's gold dots, a visualization strip and the Anti-FB
  housing on screen; embedded in the binary via `juce_add_binary_data`;
  aspect ratio locked on resize)
- `Test/SpringTest.cpp` — console validation harness
- `CMakeLists.txt` — SpringTest + JUCE plugin targets

## Notes

- Changing **Spring Size** re-ties the spring (brief tail reset).
- CPU scales linearly with Spring Size; 48 (the default) is light.
- Code licensed under AGPLv3 — see the "License" section below.

## License (AGPLv3)

Copyright © 2026 CVA Labs

**4D Spring Reverb** is distributed under the **GNU Affero General Public License
v3.0** (AGPLv3). You may use, study, modify and redistribute it under the terms of
the AGPLv3. The full license text is in the [`LICENSE`](LICENSE) file and at
<https://www.gnu.org/licenses/agpl-3.0>.

- Source code: <https://github.com/cva-labs/4dspringreverb>
- If you distribute (or make available over a network) modified versions, the AGPLv3
  requires you to also provide the corresponding source code under the same license.
- JUCE 8 is used under GPLv3 (compatible with the AGPLv3); a closed-source product
  requires a commercial JUCE license.
