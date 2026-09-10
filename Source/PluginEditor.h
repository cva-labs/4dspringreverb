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

#pragma once

#include <juce_audio_utils/juce_audio_utils.h>
#include "PluginProcessor.h"
#include "SpringVisualizer.h"

//==============================================================================
class Spring4DReverbAudioProcessorEditor : public juce::AudioProcessorEditor
{
public:
    explicit Spring4DReverbAudioProcessorEditor (Spring4DReverbAudioProcessor& p);

    void paint (juce::Graphics& g) override;
    void resized() override;

private:
    using Att = juce::AudioProcessorValueTreeState::SliderAttachment;

    struct Knob
    {
        juce::Slider slider;
        juce::Label  label;
    };

    Spring4DReverbAudioProcessor& processorRef_;
    SpringVisualizer visual_;
    juce::Image bgImage_;

    Knob drive_, tension_, decay_, helix_, pickup_, brightness_, size_, mix_, output_, antifb_;

    std::unique_ptr<Att> driveAtt_, tensionAtt_, decayAtt_, helixAtt_, pickupAtt_,
                         brightnessAtt_, sizeAtt_, mixAtt_, outputAtt_, antifbAtt_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Spring4DReverbAudioProcessorEditor)
};
