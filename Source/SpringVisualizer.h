//==============================================================================
//  4D Spring Reverb — real-time 4D spring visualisation
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
    SpringVisualizer.h — real-time view of the 4D lattice spring.

    Rest helix from the engine's public geometry + the ACTUAL node
    displacements (snapshot from the audio thread), exaggerated by an
    auto-scaled factor. Projection: slow 3D orbit of (x,y,z); the 4th
    dimension (w) is encoded as colour (magenta <-> cyan) plus "ghost" dots.

    On screen: Tension -> wire colour, 4D Coupling -> winding ghosts,
    Pickup Pos -> "B" marker, Brightness -> marker colour, Drive ->
    transducer glow, Decay -> how long the kinetic glow lingers.
*/

#include <atomic>
#include <cmath>
#include <vector>

#include <juce_gui_basics/juce_gui_basics.h>
#include "SpringLSM.h"
#include "PluginProcessor.h"

class SpringVisualizer : public juce::Component,
                         private juce::Timer
{
public:
    explicit SpringVisualizer (Spring4DReverbAudioProcessor& p)
        : processor (p)
    {
        tensionP = p.apvts.getRawParameterValue ("tension");
        helixP   = p.apvts.getRawParameterValue ("helix");
        pickupP  = p.apvts.getRawParameterValue ("pickup");
        brightP  = p.apvts.getRawParameterValue ("brightness");

        setOpaque (false);
        startTimerHz (30);
    }

    ~SpringVisualizer() override { stopTimer(); }

    void paint (juce::Graphics& g) override
    {
        using spring4d::SpringLSM;

        auto b = getLocalBounds().toFloat();

        juce::ColourGradient bg (juce::Colour (0xff241119), b.getCentreX(), b.getY(),
                                 juce::Colour (0xff150a0f), b.getCentreX(), b.getBottom(), false);
        g.setGradientFill (bg);
        g.fillRoundedRectangle (b, 8.0f);
        g.setColour (juce::Colour (0xff463039));
        g.drawRoundedRectangle (b.reduced (0.5f), 8.0f, 1.0f);

        const float tensionPn = tensionP != nullptr ? tensionP->load() : 0.5f;
        const float helixAmt  = helixP  != nullptr ? helixP->load()  : 0.5f;
        const float bright    = brightP != nullptr ? brightP->load() : 0.6f;
        const float pickupPn  = pickupP != nullptr ? pickupP->load() : 0.35f;

        const int nSeg = frame.numRings > 1 ? frame.numRings : 48;
        const int npr  = SpringLSM::kNodesPerRing;
        const int numNodes = juce::jmin (frame.numNodes, nSeg * npr);

        const float cosA = std::cos (viewAngle);
        const float sinA = std::sin (viewAngle);
        const float pitch = 0.45f;

        // Worst-case projected extents (incl. clamped displacements), so the
        // WHOLE spring stays inside the panel at every orbit angle.
        constexpr float axClamp = 0.10f;   // max drawn axial displacement (model units)
        constexpr float trClamp = 0.16f;   // max drawn transverse/w displacement
        constexpr float zMax    = 0.32f;   // max |Z| incl. cross-section offsets
        const float halfW = (0.5f + axClamp) * std::abs (cosA)
                          + zMax * std::abs (sinA) + 0.03f;
        const float halfH = (SpringLSM::kRadius + 0.5f * SpringLSM::kWireQ + trClamp)
                          + ((0.5f + axClamp) * std::abs (sinA)
                          +  zMax * std::abs (cosA)) * pitch;

        const float midX = b.getCentreX();
        const float sX = (b.getWidth() * 0.5f - 30.0f) / juce::jmax (0.45f, halfW);
        const float sT = juce::jmin (sX * 0.40f, b.getHeight() * 0.46f / halfH);
        const float cy = b.getY() + b.getHeight() * 0.52f;

        std::vector<juce::Point<float>> pts;
        std::vector<float> wN, dep;
        pts.resize ((size_t) numNodes);
        wN.resize   ((size_t) numNodes);
        dep.resize  ((size_t) numNodes);

        float maxKin = 1.0e-12f;
        for (int i = 0; i < numNodes; ++i)
            maxKin = juce::jmax (maxKin, frame.kin[i]);

        for (int x = 0; x < nSeg; ++x)
        {
            const float t  = (nSeg > 1) ? (float) x / (float) (nSeg - 1) : 0.0f;
            const float th = juce::MathConstants<float>::twoPi
                               * (float) SpringLSM::kCoils * t;
            const float rx = SpringLSM::kLength * t;
            const float ry = SpringLSM::kRadius * std::cos (th);
            const float rz = SpringLSM::kRadius * std::sin (th);
            const float rw = SpringLSM::kRadiusW * std::sin (2.0f * th);

            for (int n = 0; n < npr; ++n)
            {
                const int i = x * npr + n;
                if (i >= numNodes) break;

                const float oy = (((n & 1) != 0) ? 0.5f : -0.5f) * SpringLSM::kWireQ;
                const float oz = (((n & 2) != 0) ? 0.5f : -0.5f) * SpringLSM::kWireQ;

                const float dx = juce::jlimit (-axClamp, axClamp, scaleAx * frame.disp[i]);
                const float dy = juce::jlimit (-trClamp, trClamp, scaleTr * frame.disp[SpringLSM::kMaxNodes + i]);
                const float dz = juce::jlimit (-trClamp, trClamp, scaleTr * frame.disp[2 * SpringLSM::kMaxNodes + i]);
                const float dw = juce::jlimit (-trClamp, trClamp, scaleTr * frame.disp[3 * SpringLSM::kMaxNodes + i]);

                const float X = rx + dx;
                const float Y = ry + oy + dy;
                const float Z = rz + oz + dz;
                const float W = rw + dw;

                const float Xc = X - 0.5f * SpringLSM::kLength;  // orbit about the
                const float Xr = Xc * cosA + Z * sinA;           // vertical axis
                const float Zr = -Xc * sinA + Z * cosA;          // through the centre

                pts[(size_t) i] = { midX + Xr * sX, cy - Y * sT + Zr * sT * pitch };
                dep[(size_t) i] = Zr;
                wN[(size_t) i]  = juce::jlimit (0.0f, 1.0f,
                                     0.5f + W / (2.0f * SpringLSM::kRadiusW + 1.0e-6f));
            }
        }

        auto wColour = [] (float wn, float alpha)
        {
            return juce::Colour (0xffff4fd8)
                     .interpolatedWith (juce::Colour (0xff37e6ff), wn)
                     .withAlpha (juce::jlimit (0.0f, 1.0f, alpha));
        };
        auto depthAlpha = [&] (float z)
        {
            return 0.40f + 0.35f * juce::jlimit (-1.0f, 1.0f, z / SpringLSM::kRadius);
        };

        // axial strands (one per node index n), coloured by the local w value
        for (int n = 0; n < npr; ++n)
        {
            for (int x = 0; x + 1 < nSeg; ++x)
            {
                const int i = x * npr + n;
                const int j = (x + 1) * npr + n;
                if (i >= numNodes || j >= numNodes) break;

                const juce::Colour wc = wColour (0.5f * (wN[(size_t) i] + wN[(size_t) j]),
                                                 depthAlpha (dep[(size_t) i]));
                g.setColour (wc.interpolatedWith (juce::Colour (0xffffa63d)
                                                      .withAlpha (wc.getAlpha() / 255.0f),
                                                  0.55f * tensionPn));
                g.drawLine ({ pts[(size_t) i], pts[(size_t) j] }, 1.6f);
            }
        }

        // ring quads (the wire cross-section)
        juce::Path ringPath;
        for (int x = 0; x < nSeg; ++x)
        {
            const int b4 = x * npr;
            if (b4 + 3 >= numNodes) break;

            ringPath.startNewSubPath (pts[(size_t) (b4 + 0)]);
            ringPath.lineTo          (pts[(size_t) (b4 + 1)]);
            ringPath.lineTo          (pts[(size_t) (b4 + 3)]);
            ringPath.lineTo          (pts[(size_t) (b4 + 2)]);
            ringPath.closeSubPath();
        }
        g.setColour (juce::Colour (0xffd9c9a8).withAlpha (0.35f));
        g.strokePath (ringPath, juce::PathStrokeType (1.0f));

        // winding ghosts: offset dots showing each node's w displacement
        if (helixAmt > 0.02f)
        {
            for (int i = 0; i < numNodes; ++i)
            {
                const float wDev = juce::jlimit (-trClamp, trClamp,
                                                     scaleTr * frame.disp[3 * SpringLSM::kMaxNodes + i]);
                if (std::abs (wDev) < 1.0e-5f) continue;

                g.setColour (wColour (wN[(size_t) i], 0.15f + 0.55f * helixAmt));
                g.fillEllipse (pts[(size_t) i].x - 1.5f,
                               pts[(size_t) i].y - wDev * sT * 0.8f - 1.5f, 3.0f, 3.0f);
            }
        }

        // nodes with kinetic glow (fades with the Decay setting)
        for (int i = 0; i < numNodes; ++i)
        {
            const float k = juce::jlimit (0.0f, 1.0f, frame.kin[i] / maxKin);
            const float r = 1.4f + 2.2f * k;
            g.setColour (juce::Colour (0xfff3efe4).withAlpha (0.30f + 0.65f * k));
            g.fillEllipse (pts[(size_t) i].x - r, pts[(size_t) i].y - r, 2.0f * r, 2.0f * r);
        }

        // input transducer (drive mass): pulses with the input level (Drive knob)
        if (frame.numNodes > 0)
        {
            const int di = frame.numNodes - 1;   // drive mass is the last node
            const float dX = -0.10f + juce::jlimit (-axClamp, axClamp, scaleAx * frame.disp[di]);
            const float dY = SpringLSM::kRadius
                           + juce::jlimit (-trClamp, trClamp, scaleTr * frame.disp[SpringLSM::kMaxNodes + di]);
            const float dZ = juce::jlimit (-trClamp, trClamp, scaleTr * frame.disp[2 * SpringLSM::kMaxNodes + di]);

            const float dXc = dX - 0.5f * SpringLSM::kLength;
            const float dXr = dXc * cosA + dZ * sinA;
            const float dZr = -dXc * sinA + dZ * cosA;
            const auto p = juce::Point<float> (midX + dXr * sX, cy - dY * sT + dZr * sT * pitch);

            const float e = juce::jlimit (0.0f, 1.0f, inputEnv * 1.4f);
            const float rad = 4.0f + 2.0f * e;
            g.setColour (juce::Colour (0xffff7a1a).withAlpha (0.45f + 0.5f * e));
            g.fillEllipse (p.x - rad, p.y - rad, 2.0f * rad, 2.0f * rad);
        }

        // pickups: A fixed near the far end, B movable (Pickup Pos)
        auto drawPickup = [&] (int ring, juce::Colour col, const juce::String& label)
        {
            const int b4 = ring * npr;
            if (ring < 0 || b4 + npr > numNodes) return;

            juce::Point<float> c (0.0f, 0.0f);
            for (int n = 0; n < npr; ++n) c += pts[(size_t) (b4 + n)];
            c /= (float) npr;

            g.setColour (col);
            g.drawEllipse (c.x - 7.0f, c.y - 7.0f, 14.0f, 14.0f, 1.6f);
            g.setFont (juce::FontOptions (11.0f));
            g.drawText (label, (int) c.x - 14, (int) c.y + 9, 28, 14,
                        juce::Justification::centred);
        };

        const juce::Colour pickCol = juce::Colour (0xffffd23d)
                .interpolatedWith (juce::Colour (0xffffffff), bright);

        drawPickup (nSeg - 2, pickCol, "A");
        int rb = frame.ringB;
        if (rb <= 0 || rb >= nSeg)
            rb = (int) std::lround ((0.45f + 0.50f * pickupPn) * (float) (nSeg - 1));
        drawPickup (juce::jlimit (1, nSeg - 2, rb), pickCol, "B");

        // w legend
        const float lx = b.getRight() - 62.0f, ly = b.getY() + 10.0f;
        g.setColour (wColour (0.0f, 0.95f)); g.fillEllipse (lx,         ly, 7.0f, 7.0f);
        g.setColour (wColour (1.0f, 0.95f)); g.fillEllipse (lx + 28.0f, ly, 7.0f, 7.0f);
        g.setColour (juce::Colour (0xffd9c9a8));
        g.setFont (juce::FontOptions (10.0f));
        g.drawText ("w-", lx + 8.0f,  ly - 1.0f, 20.0f, 9.0f, juce::Justification::centredLeft);
        g.drawText ("w+", lx + 36.0f, ly - 1.0f, 20.0f, 9.0f, juce::Justification::centredLeft);

        // caption. NB: juce::String(const char*) assumes ASCII, so the
        // non-ASCII middot/multiplication sign must be handed over
        // explicitly as UTF-8 via CharPointer_UTF8.
        const juce::String caption (juce::CharPointer_UTF8 (
            "x,y,z: 3D orbit \xc2\xb7 w: color \xc2\xb7 displacement \xc3\x97"));
        g.setFont (juce::FontOptions (10.5f));
        g.setColour (juce::Colour (0xffd9c9a8));
        g.drawText (caption + juce::String (scaleTr, 0),
                    b.withTrimmedLeft (12.0f).removeFromBottom (16.0f),
                    juce::Justification::centredLeft);
    }

private:
    void timerCallback() override
    {
        processor.copyVisualFrame (frame);
        inputEnv = juce::jmax (frame.inputPeak, inputEnv * 0.86f);

        // auto-scale the exaggeration (separate axial / transverse gains) so
        // the vibration is visible but always stays inside the panel
        float maxAx = 1.0e-9f, maxTr = 1.0e-9f;
        const int n = juce::jmin (frame.numNodes, (int) spring4d::SpringLSM::kMaxNodes);
        for (int i = 0; i < n; ++i)
        {
            maxAx = juce::jmax (maxAx, std::abs (frame.disp[i]));
            maxTr = juce::jmax (maxTr, std::abs (frame.disp[  spring4d::SpringLSM::kMaxNodes + i]));
            maxTr = juce::jmax (maxTr, std::abs (frame.disp[2 * spring4d::SpringLSM::kMaxNodes + i]));
            maxTr = juce::jmax (maxTr, std::abs (frame.disp[3 * spring4d::SpringLSM::kMaxNodes + i]));
        }

        const float targetAx = juce::jlimit (4.0f, 1200.0f, 0.070f / maxAx);
        const float targetTr = juce::jlimit (8.0f, 2200.0f, 0.110f / maxTr);
        scaleAx += (targetAx - scaleAx) * 0.12f;
        scaleTr += (targetTr - scaleTr) * 0.12f;

        viewAngle += 0.010f;
        repaint();
    }

    Spring4DReverbAudioProcessor& processor;
    spring4d::SpringLSM::VisualFrame frame;

    std::atomic<float>* tensionP = nullptr;
    std::atomic<float>* helixP   = nullptr;
    std::atomic<float>* pickupP  = nullptr;
    std::atomic<float>* brightP  = nullptr;

    float inputEnv = 0.0f, scaleAx = 40.0f, scaleTr = 120.0f, viewAngle = 0.3f;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (SpringVisualizer)
};
