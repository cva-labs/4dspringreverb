//==============================================================================
//  4D Spring Reverb — Anti-FB: multi-band spectral resonance suppressor
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

#pragma once

/*
    AntiFeedback.h — multiband spectral resonance suppressor (JUCE-free).

    8-way complementary octave crossover (split points 160 Hz .. 10.24 kHz).
    Per band: a fast peak envelope is compared against a slow rolling
    reference; a band whose energy exceeds ~1.5x its rolling reference
    (resonant buildup / feedback on whatever frequency the input excites) —
    or an absolute ceiling — is ducked with fast attack and musical release.

    The knob (amount) scales the whole suppression and it is FREQUENCY-LOCAL:
    only the band(s) that actually run away are attenuated, proportionally
    to how far they exceed, so the wet level stays even across frequencies.
    At amount = 0 the processor is transparent: the crossover tree is
    complementary (sum of bands == input).
*/

#include <algorithm>
#include <cmath>

namespace spring4d
{

class AntiFeedback
{
public:
    static constexpr int kNumBands = 8;
    static constexpr int kSplits   = kNumBands - 1;

    void prepare (double sampleRate)
    {
        sr_ = (float) sampleRate;
        for (int s = 0; s < kSplits; ++s)
        {
            const float fc = 160.0f * std::pow (2.0f, (float) s);
            splitA_[s] = 1.0f - std::exp (-6.2831853f * fc / sr_);
        }
        atkC_ = 1.0f - std::exp (-1.0f / (0.0015f * sr_));   // 1.5 ms
        relC_ = 1.0f - std::exp (-1.0f / (0.320f * sr_));    // 320 ms
        refC_ = 1.0f - std::exp (-1.0f / (0.900f * sr_));    // 900 ms reference
        reset();
    }

    void reset() noexcept
    {
        for (int s = 0; s < kSplits; ++s)
            lp1L_[s] = lp2L_[s] = lp1R_[s] = lp2R_[s] = 0.0f;
        for (int b = 0; b < kNumBands; ++b)
            env_[b] = ref_[b] = 0.0f, g_[b] = 1.0f;
    }

    void setAmount (float a) noexcept { amount_ = std::clamp (a, 0.0f, 1.0f); }
    float getAmount() const noexcept  { return amount_; }

    void processBlock (float* l, float* r, int numSamples) noexcept
    {
        for (int n = 0; n < numSamples; ++n)
            processSample (l[n], r[n]);
    }

private:
    void processSample (float& l, float& r) noexcept
    {
        float sigL = l, sigR = r;
        float outL = 0.0f, outR = 0.0f;

        for (int s = 0; s < kSplits; ++s)
        {
            lp1L_[s] += splitA_[s] * (sigL - lp1L_[s]);
            lp2L_[s] += splitA_[s] * (lp1L_[s] - lp2L_[s]);
            lp1R_[s] += splitA_[s] * (sigR - lp1R_[s]);
            lp2R_[s] += splitA_[s] * (lp1R_[s] - lp2R_[s]);

            const float bl = sigL - lp2L_[s];    // high part (complementary)
            const float br = sigR - lp2R_[s];

            outL += bl * applied (s);
            outR += br * applied (s);
            track (s, std::max (std::abs (bl), std::abs (br)));

            sigL = lp2L_[s];                     // continue with the low part
            sigR = lp2R_[s];
        }

        outL += sigL * applied (kNumBands - 1);
        outR += sigR * applied (kNumBands - 1);
        track (kNumBands - 1, std::max (std::abs (sigL), std::abs (sigR)));

        l = outL;
        r = outR;
    }

    float applied (int b) const noexcept
    {
        return 1.0f - amount_ * (1.0f - g_[b]);
    }

    void track (int b, float x) noexcept
    {
        float& env = env_[b];
        env += (x - env) * ((x > env) ? atkC_ : relC_);
        ref_[b] += (env - ref_[b]) * refC_;

        float target = 1.0f;
        const float over = ref_[b] * 1.5f + 1.0e-3f;          // resonance detector
        if (env > over)  target = over / env;                 // proportional duck
        if (env > 0.60f) target = std::min (target, 0.60f / env); // absolute ceiling
        target = std::max (target, 0.09f);                    // max cut ~ -21 dB

        float& g = g_[b];
        g += (target - g) * ((target < g) ? atkC_ : relC_);
    }

    float sr_ = 48000.0f;
    float splitA_[kSplits] {};
    float lp1L_[kSplits] {}, lp2L_[kSplits] {};
    float lp1R_[kSplits] {}, lp2R_[kSplits] {};
    float env_[kNumBands] {}, ref_[kNumBands] {}, g_[kNumBands] {};
    float atkC_ = 0.02f, relC_ = 2.0e-4f, refC_ = 5.0e-5f;
    float amount_ = 0.0f;
};

} // namespace spring4d
