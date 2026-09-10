# 4D Spring Reverb (CVA Labs) — VST3 Spring Reverb με 4D Lattice Spring Model

Φυσικό μοντέλο spring reverb όπου το ελατήριο είναι ένα **4D Lattice Spring Model (LSM)**:
πλέγμα μαζών-ελατηρίων του οποίου η γεωμετρία ηρεμίας είναι **έλικα σε 4 διαστάσεις**
(3 χωρικοί άξονες x/y/z + άξονας περιέλιξης «w»). Η πλήρης μαθηματική περιγραφή είναι
στο `DESIGN.md`.

## Λήψη (Windows & macOS)

Έτοιμα builds (VST3 + Standalone) δημοσιεύονται στα
[GitHub Releases](https://github.com/cva-labs/4dspringreverb/releases):

- **Windows**: `4DSpringReverb-<έκδοση>-Windows-VST3.zip`, `...-Windows-Standalone.zip`
- **macOS** (universal, Apple Silicon + Intel): `4DSpringReverb-<έκδοση>-macOS-VST3.zip`, `...-macOS-Standalone.zip`

Εγκατάσταση:
- **Windows VST3**: αποσυμπιέστε και αντιγράψτε ολόκληρο τον φάκελο `4D Spring Reverb.vst3`
  στο `C:\Program Files\Common Files\VST3`.
- **Windows Standalone**: αποσυμπιέστε το `4D Spring Reverb.exe` οπουδήποτε και τρέξτε το.
- **macOS VST3**: αποσυμπιέστε και αντιγράψτε τον φάκελο `4D Spring Reverb.vst3` στο
  `/Library/Audio/Plug-Ins/VST3`.
- **macOS Standalone**: αποσυμπιέστε και μετακινήστε το `4D Spring Reverb.app` στο
  `/Applications`. Τα macOS binaries δεν είναι υπογεγραμμένα/notarized — την πρώτη φορά
  δεξί κλικ → Open (ή `xattr -cr "/Applications/4D Spring Reverb.app"` αν το Gatekeeper
  εμποδίσει το άνοιγμα).

## Build από πηγαίο κώδικα (Windows / Visual Studio)

Απαιτήσεις: CMake ≥ 3.22, Visual Studio (C++ workload), git.
Το JUCE 8.0.6 κατεβαίνει αυτόματα μέσω FetchContent την πρώτη φορά.

```powershell
cmake -S . -B build
cmake --build build --config Release --target Spring4DReverb_VST3 --parallel
```

Τέχνημα (το plugin εμφανίζεται στους hosts ως **4D Spring Reverb** από τη **CVA Labs**):
- VST3: `build/Spring4DReverb_artefacts/Release/VST3/4D Spring Reverb.vst3`
- Standalone: `build/Spring4DReverb_artefacts/Release/Standalone/4D Spring Reverb.exe`

Εγκατάσταση: αντιγράψτε ολόκληρο τον φάκελο `4D Spring Reverb.vst3` στο
`C:\Program Files\Common Files\VST3` (ή όποιον VST3 φάκελο σαράρει ο host).

## Έλεγχος ορθότητας (console, χωρίς JUCE)

```powershell
cmake --build build --config Release --target SpringTest --parallel
.\build\Release\SpringTest.exe
```

Τρέχει impulse / noise burst / sine / stability sweep, γράφει τα
`spring4d_impulse.wav`, `spring4d_burst_L.wav`, `spring4d_burst_R.wav`,
`spring4d_sine.wav` και τυπώνει σταθερότητα + κόστος CPU.

## Παράμετροι

| Slider | Περιγραφή |
|---|---|
| Drive | Gain + soft-clip (tanh) στην είσοδο του ελατηρίου |
| Tension | Αξονική ακαμψία — ταχύτητα κυμάτων / φωτεινότητα |
| Decay | Χρόνος αποσύνθεσης έως −60 dB (0.4–12 s) |
| 4D Coupling | Δύναμη σύζευξης στον 4ο άξονα w (περιέλιξη/στρέψη) |
| Pickup Pos | Θέση του 2ου pickup (δεξί κανάλι) κατά μήκος του ελατηρίου |
| Brightness | Low-pass των pickups (700 Hz–12 kHz) |
| Spring Size | Πλήθος τμημάτων/δακτυλίων (24–96) — αλλάζει και το CPU |
| Mix | Dry/Wet |
| Output | Τελικό gain (dB) |
| Anti-FB | Πολυζωνικός φασματικός καταστολέας συντονισμών — ανιχνεύει ανά ζώνη (8 οκτάβα, 160 Hz–10 kHz) πότε η ενέργεια ξεφεύγει από το rolling reference της ή το απόλυτο ορίο και την «πατάει» ανάλογα· σταθερή στάθμη σε όλες τις συχνότητες· στο 0 πλήρως διαφανές |

Έξοδος στερεοφωνική: L = pickup κοντά στο ελεύθερο άκρο, R = κινούμενο pickup.
Είσοδος mono (ή άθροισμα stereo).

## Αρχιτεκτονική

- `Source/SpringLSM.h` — DSP engine (header-only, ανεξάρτητο από JUCE)
- `Source/PluginProcessor.*` — JUCE AudioProcessor + AudioProcessorValueTreeState
- `Source/PluginEditor.*` — GUI (9 knobs + ζωντανή οπτικοποίηση)
- `Source/SpringVisualizer.h` — real-time 4D οπτικοποίηση του ελατηρίου
  (projection της έλικας, w→χρώμα, kinetic glow, pickup/transducer markers)
- `assets/background.png` — CVA LABS skin (1504×1046 landscape· τα 9 pots
  κεντραρισμένα στις χρυσές κουκίδες του skin, ζώνη οπτικοποίησης και θήκη
  Anti-FB στην οθόνη· ενσωματωμένο στο binary μέσω `juce_add_binary_data`·
  κεκλειδωμένο aspect ratio στο resize)
- `Test/SpringTest.cpp` — console validation harness
- `CMakeLists.txt` — SpringTest + JUCE plugin targets

## Σημειώσεις

- Η αλλαγή του **Spring Size** «ξαναδένει» το ελατήριο (σύντομο reset του tail).
- Το CPU κλιμακώνεται γραμμικά με το Spring Size· το 48 (default) είναι ελαφρύ.
- Κώδικας υπό AGPLv3 — βλ. ενότητα «Άδεια» παρακάτω.

## Άδεια (AGPLv3)

Copyright © 2026 CVA Labs

Το **4D Spring Reverb** διανέμεται υπό την άδεια **GNU Affero General Public License
v3.0** (AGPLv3). Μπορείτε να το χρησιμοποιήσετε, να το μελετήσετε, να το τροποποιήσετε
και να το αναδιανείμετε σύμφωνα με τους όρους της AGPLv3. Το πλήρες κείμενο της άδειας
βρίσκεται στο αρχείο [`LICENSE`](LICENSE) και στο
<https://www.gnu.org/licenses/agpl-3.0>.

- Πηγαίος κώδικας: <https://github.com/cva-labs/4dspringreverb>
- Αν διανείμετε (ή παραδώσετε μέσω δικτύου) τροποποιημένες εκδόσεις, η AGPLv3 απαιτεί
  να διαθέσετε και τον αντίστοιχο πηγαίο κώδικα με την ίδια άδεια.
- Η JUCE 8 χρησιμοποιείται υπό GPLv3 (συμβατή με την AGPLv3)· για κλειστό προϊόν
  απαιτείται εμπορική άδεια JUCE.
