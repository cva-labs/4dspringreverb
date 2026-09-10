//==============================================================================
//  4D Spring Reverb — console validation harness
//  Copyright (C) 2026 CVA Labs — https://github.com/cva-labs/4dspringreverb
//
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU Affero General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful, but
//  WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU Affero General Public License for more details.
//
//  You should have received a copy of the GNU Affero General Public License
//  along with this program. If not, see <https://www.gnu.org/licenses/>.
//==============================================================================

// SpringTest.cpp — JUCE-free validation harness for the 4D LSM spring engine.
// Renders impulse / noise-burst / sine through the engine, checks stability
// and decay, writes WAV files and reports CPU cost.

#include "../Source/SpringLSM.h"
#include "../Source/AntiFeedback.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#if defined(_M_X64) || defined(__x86_64__)
  #include <pmmintrin.h>
#endif

using spring4d::SpringLSM;

static void enableDenormalFTZ()
{
  #if defined(_M_X64) || defined(__x86_64__)
    _mm_setcsr (_mm_getcsr() | 0x8040u);      // FTZ | DAZ
  #endif
}

struct Stats { bool finite = true; float peak = 0.0f; double rms = 0.0; };

static Stats analyse (const std::vector<float>& x, int from = 0, int to = -1)
{
    if (to < 0 || to > (int) x.size()) to = (int) x.size();
    Stats s;
    double acc = 0.0;
    int count = 0;
    for (int i = from; i < to; ++i)
    {
        const float v = x[(size_t) i];
        if (! std::isfinite (v)) s.finite = false;
        s.peak = std::max (s.peak, std::abs (v));
        acc += (double) v * v;
        ++count;
    }
    s.rms = (count > 0) ? std::sqrt (acc / (double) count) : 0.0;
    return s;
}

static double goertzel (const std::vector<float>& x, int from, int to, double freq, double sr)
{
    const double k = 2.0 * 3.14159265358979 * freq / sr;
    const double c = 2.0 * std::cos (k);
    double s1 = 0.0, s2 = 0.0;
    for (int i = from; i < to; ++i)
    {
        const double s = (double) x[(size_t) i] + c * s1 - s2;
        s2 = s1;
        s1 = s;
    }
    const int n = to - from;
    return std::sqrt (std::max (0.0, s1 * s1 + s2 * s2 - c * s1 * s2)) / (n > 0 ? n : 1);
}

static void writeWav16 (const std::string& path, const std::vector<float>& x, double sr)
{
    std::ofstream f (path, std::ios::binary);
    if (! f) { std::printf ("  !! cannot write %s\n", path.c_str()); return; }

    const std::uint32_t n         = (std::uint32_t) x.size();
    const std::uint32_t dataBytes = n * 2u;

    auto wr4 = [&f] (std::uint32_t v)
    {
        f.put ((char) (v & 255u));          f.put ((char) ((v >> 8)  & 255u));
        f.put ((char) ((v >> 16) & 255u));  f.put ((char) ((v >> 24) & 255u));
    };
    auto wr2 = [&f] (std::uint16_t v)
    {
        f.put ((char) (v & 255u));          f.put ((char) ((v >> 8) & 255u));
    };

    f.write ("RIFF", 4);  wr4 (36u + dataBytes);  f.write ("WAVE", 4);
    f.write ("fmt ", 4);  wr4 (16u);  wr2 (1u);   wr2 (1u);
    wr4 ((std::uint32_t) sr);
    wr4 ((std::uint32_t) (sr * 2.0));
    wr2 (2u);  wr2 (16u);
    f.write ("data", 4);  wr4 (dataBytes);

    for (float v : x)
    {
        const float c = std::clamp (v, -1.0f, 1.0f);
        wr2 ((std::uint16_t) (std::int16_t) std::lround (c * 32767.0f));
    }
}

static void render (SpringLSM& eng, const std::vector<float>& in,
                    std::vector<float>& L, std::vector<float>& R)
{
    const int n = (int) in.size();
    L.resize ((size_t) n);
    R.resize ((size_t) n);
    for (int i = 0; i < n; ++i)
        eng.processBlock (&in[(size_t) i], &L[(size_t) i], &R[(size_t) i], 1);
}

int main()
{
    enableDenormalFTZ();

    constexpr double sr = 48000.0;
    int failures = 0;
    auto check = [&failures] (bool ok, const char* what)
    {
        std::printf ("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
        if (! ok) ++failures;
    };

    std::printf ("4D Spring Reverb engine test (48 segments, sr=%.0f)\n", sr);

    // ---- Test 1: impulse response + diagnostics -----------------------------
    {
        SpringLSM eng;
        eng.prepare (sr, 48);
        eng.setParameters (SpringLSM::Parameters {});

        const int n = (int) (6.0 * sr);
        std::vector<float> in ((size_t) n, 0.0f);
        in[0] = 1.0f;
        std::vector<float> L ((size_t) n), R ((size_t) n);
        for (int i = 0; i < n; ++i)
        {
            eng.processBlock (&in[(size_t) i], &L[(size_t) i], &R[(size_t) i], 1);
            if (i == (int) (0.05 * sr) || i == (int) (0.5 * sr) ||
                i == (int) (2.0 * sr)  || i == (int) (5.0 * sr))
            {
                const auto d = eng.diagnose();
                std::printf ("  diag t=%.2fs: |vx|=%.3e |vy|=%.3e |vz|=%.3e |vw|=%.3e  dx=%.2e dy=%.2e dz=%.2e dw=%.2e node=%d g=%.8f kAx=%.3e\n",
                             (double) i / sr, d.velX, d.velY, d.velZ, d.velW,
                             d.dispX, d.dispY, d.dispZ, d.dispW, d.maxVelNode,
                             d.gNode, d.kAx);
            }
        }

        // decay probe: render 1 s of silence and watch the lattice state fall
        const auto d6 = eng.diagnose();
        {
            std::vector<float> in2 ((size_t) sr, 0.0f);
            std::vector<float> L2 ((size_t) sr), R2 ((size_t) sr);
            render (eng, in2, L2, R2);
        }
        const auto d7 = eng.diagnose();
        const float m6 = std::max (std::max (d6.velX, d6.velY), std::max (d6.velZ, d6.velW));
        const float m7 = std::max (std::max (d7.velX, d7.velY), std::max (d7.velZ, d7.velW));
        std::printf ("  probe: gNode=%.8f dt=%.3e kAx=%.3e  max|v| 6s->7s: %.4e -> %.4e (expect ~x0.031)\n",
                     d7.gNode, d7.dt, d7.kAx, m6, m7);

        std::printf ("  tail spectrum (amp):");
        for (double f : { 100.0, 300.0, 700.0, 1200.0, 1600.0, 2200.0, 3000.0, 3800.0, 5500.0 })
            std::printf (" %.0f:%.5f", f, goertzel (L, (int) (4.5 * sr), n, f, sr));
        std::printf ("\n");

        // bisection: which spring class sustains the ~HF state?
        {
            const struct Cfg { int mask; const char* name; } cfgs[] =
            {
                { 0x3F, "full          " },
                { 0x3E, "no-winding(3) " },
                { 0x3D, "no-shear(4)   " },
                { 0x3B, "no-diag(2)    " },
                { 0x3D - 8, "no-sides(1)   " },
                { 0x1F, "no-axial(0)   " },
                { 0x20, "drive-only    " },
            };
            for (const auto& cf : cfgs)
            {
                SpringLSM eb;
                eb.prepare (sr, 48);
                SpringLSM::Parameters pb;
                pb.debugMask = cf.mask;
                eb.setParameters (pb);
                const int nb = (int) (2.0 * sr);
                std::vector<float> inb ((size_t) nb, 0.0f);
                inb[0] = 1.0f;
                std::vector<float> Lb ((size_t) nb), Rb ((size_t) nb);
                for (int i = 0; i < nb; ++i)
                    eb.processBlock (&inb[(size_t) i], &Lb[(size_t) i], &Rb[(size_t) i], 1);
                const auto db = eb.diagnose();
                const float mb = std::max (std::max (db.velX, db.velY), std::max (db.velZ, db.velW));
                int zcb = 0;
                for (int i = (int) (1.0 * sr) + 1; i < nb; ++i)
                    if ((Lb[(size_t) (i - 1)] < 0.0f) != (Lb[(size_t) i] < 0.0f)) ++zcb;
                std::printf ("  bisect %s: max|v|=%.4e  zcr(1-2s)=%.0f/s\n",
                             cf.name, mb, (double) zcb);
            }
        }

        const Stats full  = analyse (L);
        const Stats early = analyse (L, (int) (0.4 * sr), (int) (2.0 * sr));
        const Stats late  = analyse (L, (int) (4.5 * sr), n);

        int zc = 0;
        for (int i = (int) (4.5 * sr) + 1; i < n; ++i)
            if ((L[(size_t) (i - 1)] < 0.0f) != (L[(size_t) i] < 0.0f)) ++zc;
        const double zcr = (double) zc / 1.5;

        check (full.finite,                             "impulse: all samples finite");
        check (full.peak > 1.0e-4f && full.peak < 8.0f, "impulse: peak in sane range");
        check (late.rms < early.rms * 0.5,              "impulse: tail decays");

        std::printf ("  rms/0.75s:");
        for (int w = 0; w < 8; ++w)
            std::printf (" %.4f", analyse (L, (int) (w * 0.75 * sr), (int) ((w + 1) * 0.75 * sr)).rms);
        std::printf ("\n  tail zero-crossings/s: %.1f\n", zcr);

        writeWav16 ("spring4d_impulse.wav", L, sr);
        std::printf ("  impulse: peak=%.6f rms(0.4-2s)=%.6f rms(4.5-6s)=%.6f\n",
                     full.peak, early.rms, late.rms);
    }

    // ---- Test 2: noise burst (level reference + CPU) ------------------------
    {
        SpringLSM eng;
        eng.prepare (sr, 48);
        SpringLSM::Parameters p;
        p.drive = 0.5f;
        eng.setParameters (p);

        const int n = (int) (4.0 * sr);
        std::vector<float> in ((size_t) n, 0.0f);
        std::mt19937 rng (20260906u);
        std::uniform_real_distribution<float> dist (-1.0f, 1.0f);
        const int burst = (int) (0.25 * sr);
        for (int i = 0; i < burst; ++i)
            in[(size_t) i] = 0.7f * dist (rng);

        const auto t0 = std::chrono::steady_clock::now();
        std::vector<float> L, R;
        render (eng, in, L, R);
        const auto t1 = std::chrono::steady_clock::now();
        const double cpuTimesRealtime = std::chrono::duration<double> (t1 - t0).count() / 4.0;

        const Stats full  = analyse (L);
        const Stats early = analyse (L, (int) (0.3 * sr), (int) (1.5 * sr));
        const Stats late  = analyse (L, (int) (3.0 * sr), n);

        check (full.finite,          "burst: all samples finite");
        check (full.peak < 50.0f,    "burst: peak < 50 (stable)");
        check (late.rms < early.rms, "burst: tail decays");

        writeWav16 ("spring4d_burst_L.wav", L, sr);
        writeWav16 ("spring4d_burst_R.wav", R, sr);
        std::printf ("  burst: peak=%.6f rms(0.3-1.5s)=%.6f rms(3-4s)=%.6f cpu=%.2fx realtime\n",
                     full.peak, early.rms, late.rms, cpuTimesRealtime);
    }

    // ---- Test 3: steady sine -------------------------------------------------
    {
        SpringLSM eng;
        eng.prepare (sr, 48);
        eng.setParameters (SpringLSM::Parameters {});

        const int n = (int) (1.5 * sr);
        std::vector<float> in ((size_t) n);
        for (int i = 0; i < n; ++i)
            in[(size_t) i] = 0.5f * (float) std::sin (2.0 * 3.14159265358979 * 1000.0 * i / sr);

        std::vector<float> L, R;
        render (eng, in, L, R);

        const Stats full = analyse (L);
        check (full.finite,      "sine: all samples finite");
        check (full.rms > 1e-5,  "sine: audible output level");

        writeWav16 ("spring4d_sine.wav", L, sr);
        std::printf ("  sine: peak=%.6f rms=%.6f\n", full.peak, full.rms);
    }

    // ---- Test 4: parameter corner stability sweep ----------------------------
    {
        struct Case { float tension, helix, decay; };
        const Case cases[] =
        {
            { 0.0f, 0.0f,  0.4f },
            { 1.0f, 0.0f,  0.4f },
            { 0.0f, 1.0f, 12.0f },
            { 1.0f, 1.0f, 12.0f },
            { 0.5f, 1.0f,  0.4f },
        };

        bool ok = true;
        double worst = 0.0;
        std::mt19937 rng (7u);
        std::uniform_real_distribution<float> dist (-1.0f, 1.0f);

        for (const auto& c : cases)
        {
            SpringLSM eng;
            eng.prepare (sr, 96);                     // worst-case size
            SpringLSM::Parameters p;
            p.tension = c.tension;
            p.helix4d = c.helix;
            p.decay   = c.decay;
            p.drive   = 1.0f;
            eng.setParameters (p);

            const int n = (int) (2.0 * sr);
            std::vector<float> in ((size_t) n, 0.0f);
            for (int i = 0; i < (int) (0.2 * sr); ++i)
                in[(size_t) i] = dist (rng);

            std::vector<float> L, R;
            render (eng, in, L, R);

            const Stats s = analyse (L);
            worst = std::max (worst, (double) s.peak);
            if (! s.finite || s.peak > 100.0f)
                ok = false;
        }

        check (ok, "corner sweep (tension/helix/decay, 96 segs): stable");
        std::printf ("  sweep: worst peak=%.4f\n", worst);
    }

    // ---- Test 5: anti-feedback suppressor ------------------------------------
    {
        SpringLSM eng;
        eng.prepare (sr, 48);
        SpringLSM::Parameters p;
        p.drive = 0.5f;
        eng.setParameters (p);

        const int n = (int) (4.0 * sr);
        std::vector<float> in ((size_t) n, 0.0f);
        std::mt19937 rng (20260906u);
        std::uniform_real_distribution<float> dist (-1.0f, 1.0f);
        for (int i = 0; i < (int) (0.25 * sr); ++i)
            in[(size_t) i] = 0.7f * dist (rng);

        std::vector<float> L, R;
        render (eng, in, L, R);

        spring4d::AntiFeedback af;
        af.prepare (sr);

        std::vector<float> Lb = L, Rb = R;
        af.setAmount (0.0f);
        af.processBlock (Lb.data(), Rb.data(), n);
        float maxDiff = 0.0f;
        for (int i = 0; i < n; ++i)
            maxDiff = std::max (maxDiff, std::abs (Lb[(size_t) i] - L[(size_t) i]));
        check (maxDiff < 1.0e-3f, "anti-fb: bypass (amount=0) is transparent");

        std::vector<float> Ls = L, Rs = R;
        af.setAmount (1.0f);
        af.processBlock (Ls.data(), Rs.data(), n);
        const Stats s0 = analyse (L);
        const Stats s1 = analyse (Ls);
        check (s1.finite,         "anti-fb: all samples finite");
        check (s1.peak < s0.peak, "anti-fb: resonant peak reduced");
        check (s1.peak > 0.01f,   "anti-fb: signal still present");

        // hot corner: drive=1, 12 s decay, 96 segments -> resonant peaks > 0 dB
        SpringLSM eng2;
        eng2.prepare (sr, 96);
        SpringLSM::Parameters p2;
        p2.drive = 1.0f;
        p2.decay = 12.0f;
        eng2.setParameters (p2);
        const int n2 = (int) (2.0 * sr);
        std::vector<float> in2 ((size_t) n2, 0.0f);
        std::mt19937 rng2 (99u);
        for (int i = 0; i < (int) (0.2 * sr); ++i)
            in2[(size_t) i] = dist (rng2);
        std::vector<float> L2, R2;
        render (eng2, in2, L2, R2);
        const Stats raw = analyse (L2);
        af.reset();
        std::vector<float> L2f = L2, R2f = R2;
        af.processBlock (L2f.data(), R2f.data(), n2);
        const Stats damped = analyse (L2f);
        check (damped.finite && damped.peak < 4.0f,
               "anti-fb: hot corner peaks capped (< 4.0)");
        std::printf ("  anti-fb: burst peak %.4f -> %.4f, hot corner %.3f -> %.3f\n",
                     s0.peak, s1.peak, raw.peak, damped.peak);
    }

    std::printf (failures == 0 ? "\nALL TESTS PASSED\n" : "\n%d TEST(S) FAILED\n", failures);
    return (failures == 0) ? 0 : 1;
}
