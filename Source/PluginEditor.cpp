//==============================================================================
//  4D Spring Reverb — plugin editor (9 knobs + live 4D visualisation)
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

#include "PluginEditor.h"
#include "BinaryData.h"

namespace
{
    // Geometry measured on assets/background.png (1504 x 1046), as fractions of
    // the artwork. The nine gold dots form a 3x3 grid: every pot is centred on
    // its dot. The cream window up top holds the visualiser plus a right-hand
    // pocket for the Anti-FB pot.
    constexpr float colX[3] = { 0.2199f, 0.4996f, 0.7793f };   // dot columns
    constexpr float rowY[3] = { 0.5873f, 0.7312f, 0.8738f };   // dot rows
    constexpr float scrX0   = 0.0618f, scrX1 = 0.9368f;        // cream screen
    constexpr float scrY0   = 0.1023f, scrY1 = 0.4885f;
    constexpr float visX1   = 0.8180f;                         // visualiser right edge
    constexpr float afCx    = 0.8930f, afCy = 0.2954f;         // Anti-FB pot centre

    // "v1.0" badge drawn right of the engraved title (fractions of the artwork)
    constexpr const char* versionText = "v1.0";
    constexpr float verX  = 0.6051f;    // left edge, just right of the title
    constexpr float verCy = 0.0516f;    // vertical centre of the title row

    void setupKnob (juce::Slider& s, juce::Label& l, const juce::String& text)
    {
        s.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
        s.setTextBoxStyle (juce::Slider::NoTextBox, false, 0, 0);
        s.setColour (juce::Slider::rotarySliderFillColourId,    juce::Colour (0xfff2e4bc));
        s.setColour (juce::Slider::rotarySliderOutlineColourId, juce::Colour (0x00000000));
        s.setColour (juce::Slider::thumbColourId,               juce::Colour (0xffffdfa0));
        s.setColour (juce::Slider::textBoxTextColourId,         juce::Colour (0xff33241a));
        s.setColour (juce::Slider::textBoxHighlightColourId,    juce::Colour (0xfff2e4bc));
        s.setColour (juce::Slider::textBoxOutlineColourId,      juce::Colour (0x00000000));

        l.setText (text, juce::dontSendNotification);       // engraved labels
        l.setJustificationType (juce::Justification::centred); // painted in paint()
        l.setInterceptsMouseClicks (false, false);
    }
}

//==============================================================================
Spring4DReverbAudioProcessorEditor::Spring4DReverbAudioProcessorEditor (Spring4DReverbAudioProcessor& p)
    : AudioProcessorEditor (p), processorRef_ (p), visual_ (p)
{
    bgImage_ = juce::ImageCache::getFromMemory (BinaryData::background_png,
                                                BinaryData::background_pngSize);

    setupKnob (drive_.slider,      drive_.label,      "Drive");
    setupKnob (tension_.slider,    tension_.label,    "Tension");
    setupKnob (decay_.slider,      decay_.label,      "Decay");
    setupKnob (helix_.slider,      helix_.label,      "4D Coupling");
    setupKnob (pickup_.slider,     pickup_.label,     "Pickup Pos");
    setupKnob (brightness_.slider, brightness_.label, "Brightness");
    setupKnob (size_.slider,       size_.label,       "Spring Size");
    setupKnob (mix_.slider,        mix_.label,        "Mix");
    setupKnob (output_.slider,     output_.label,     "Output");
    setupKnob (antifb_.slider,     antifb_.label,     "Anti-FB");

    addAndMakeVisible (visual_);

    for (auto* k : { &drive_, &tension_, &decay_, &helix_, &pickup_,
                     &brightness_, &size_, &mix_, &output_, &antifb_ })
        addAndMakeVisible (k->slider);

    auto& apvts = processorRef_.apvts;

    driveAtt_      = std::make_unique<Att> (apvts, "drive",      drive_.slider);
    tensionAtt_    = std::make_unique<Att> (apvts, "tension",    tension_.slider);
    decayAtt_      = std::make_unique<Att> (apvts, "decay",      decay_.slider);
    helixAtt_      = std::make_unique<Att> (apvts, "helix",      helix_.slider);
    pickupAtt_     = std::make_unique<Att> (apvts, "pickup",     pickup_.slider);
    brightnessAtt_ = std::make_unique<Att> (apvts, "brightness", brightness_.slider);
    sizeAtt_       = std::make_unique<Att> (apvts, "size",       size_.slider);
    mixAtt_        = std::make_unique<Att> (apvts, "mix",        mix_.slider);
    outputAtt_     = std::make_unique<Att> (apvts, "output",     output_.slider);
    antifbAtt_     = std::make_unique<Att> (apvts, "antifb",     antifb_.slider);

    setResizable (true, true);                      // host-resizable + corner grip
    setResizeLimits (720, 501, 1504, 1046);
    getConstrainer()->setFixedAspectRatio (          // keep the artwork undistorted
        bgImage_.isValid() ? (double) bgImage_.getWidth() / (double) bgImage_.getHeight()
                           : 1.4379);
    setSize (1150, 800);
}

//==============================================================================
void Spring4DReverbAudioProcessorEditor::paint (juce::Graphics& g)
{
    const float W = (float) getWidth(), H = (float) getHeight();

    g.fillAll (juce::Colour (0xff201016));
    if (bgImage_.isValid())
        g.drawImage (bgImage_, getLocalBounds().toFloat(),
                     juce::RectanglePlacement (juce::RectanglePlacement::fillDestination));

    // the nine pots sit exactly on the artwork's gold dots
    const float r    = H * 0.0575f;     // pot radius
    const float seat = r * 1.06f;       // recessed seat radius
    const char* texts[9] = { "DRIVE", "TENSION", "DECAY",
                             "4D COUPLING", "PICKUP POS", "BRIGHTNESS",
                             "SPRING SIZE", "MIX", "OUTPUT" };

    for (int i = 0; i < 9; ++i)
    {
        const float cx = W * colX[i % 3];
        const float cy = H * rowY[i / 3];

        g.setColour (juce::Colour (0x33000000));            // machined seat
        g.fillEllipse (cx - seat, cy - seat, 2.0f * seat, 2.0f * seat);
        g.setColour (juce::Colour (0x3df2e4bc));
        g.drawEllipse (cx - seat, cy - seat, 2.0f * seat, 2.0f * seat, 1.1f);

        g.setColour (juce::Colour (0xd9f2e4bc));            // engraved label
        g.setFont (juce::FontOptions (H * 0.0145f));
        g.drawText (texts[i], cx - W * 0.115f, cy + seat + H * 0.002f,
                    W * 0.23f, H * 0.018f, juce::Justification::centred);
    }

    // Anti-FB pocket on the cream screen, right side
    const float acx = W * afCx, acy = H * afCy;
    g.setColour (juce::Colour (0xff241119));
    g.fillEllipse (acx - seat, acy - seat, 2.0f * seat, 2.0f * seat);
    g.setColour (juce::Colour (0x66f2e4bc));
    g.drawEllipse (acx - seat, acy - seat, 2.0f * seat, 2.0f * seat, 1.1f);
    g.setColour (juce::Colour (0xff33241a));                // dark text on cream
    g.setFont (juce::FontOptions (H * 0.0145f));
    g.drawText ("ANTI-FB", acx - W * 0.06f, acy + seat + H * 0.004f,
                W * 0.12f, H * 0.018f, juce::Justification::centred);

    // version badge, right of the engraved title
    g.setColour (juce::Colour (0xd9f2e4bc));
    g.setFont (juce::FontOptions (H * 0.032f));
    g.drawText (versionText, W * verX, H * (verCy - 0.020f),
                W * 0.10f, H * 0.040f, juce::Justification::centredLeft);
}

void Spring4DReverbAudioProcessorEditor::resized()
{
    const float W = (float) getWidth(), H = (float) getHeight();
    const int   r = (int) (H * 0.0575f);

    // visualiser inside the artwork's screen; right pocket kept for Anti-FB
    const int vx = (int) (W * scrX0) + 3;
    const int vy = (int) (H * scrY0) + 3;
    const int vw = (int) (W * visX1) - vx;
    const int vh = (int) (H * scrY1) - 3 - vy;
    visual_.setBounds (vx, vy, juce::jmax (10, vw), juce::jmax (10, vh));

    antifb_.slider.setBounds ((int) (W * afCx) - r, (int) (H * afCy) - r,
                              2 * r, 2 * r);

    // the nine knobs land exactly on the artwork's gold dots
    Knob* knobs[] = { &drive_, &tension_, &decay_, &helix_, &pickup_,
                      &brightness_, &size_, &mix_, &output_ };

    for (int i = 0; i < 9; ++i)
        knobs[i]->slider.setBounds ((int) (W * colX[i % 3]) - r,
                                    (int) (H * rowY[i / 3]) - r, 2 * r, 2 * r);
}
