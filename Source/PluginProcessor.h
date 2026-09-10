//==============================================================================
//  4D Spring Reverb — AudioProcessor + AudioProcessorValueTreeState
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

#include <juce_audio_processors/juce_audio_processors.h>
#include "SpringLSM.h"
#include "AntiFeedback.h"

//==============================================================================
class Spring4DReverbAudioProcessor : public juce::AudioProcessor
{
public:
    Spring4DReverbAudioProcessor();
    ~Spring4DReverbAudioProcessor() override = default;

    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override {}
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override                        { return true; }
    const juce::String getName() const override            { return JucePlugin_Name; }
    bool acceptsMidi() const override                      { return false; }
    bool producesMidi() const override                     { return false; }
    bool isMidiEffect() const override                     { return false; }
    double getTailLengthSeconds() const override;

    int getNumPrograms() override                            { return 1; }
    int getCurrentProgram() override                         { return 0; }
    void setCurrentProgram (int) override                    {}
    const juce::String getProgramName (int) override         { return "Default"; }
    void changeProgramName (int, const juce::String&) override {}

    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    juce::AudioProcessorValueTreeState apvts;

    // Thread-safe snapshot of the lattice for the GUI visualiser.
    bool copyVisualFrame (spring4d::SpringLSM::VisualFrame& out) const
    {
        return engine_.copyVisualFrame (out);
    }

private:
    static juce::AudioProcessorValueTreeState::ParameterLayout createLayout();

    spring4d::SpringLSM engine_;
    spring4d::AntiFeedback antifb_;
    juce::AudioBuffer<float> wetBuf_;
    int lastSegments_ = -1;

    std::atomic<float>* driveParam_   = nullptr;
    std::atomic<float>* tensionParam_ = nullptr;
    std::atomic<float>* decayParam_   = nullptr;
    std::atomic<float>* helixParam_   = nullptr;
    std::atomic<float>* pickupParam_  = nullptr;
    std::atomic<float>* brightParam_  = nullptr;
    std::atomic<float>* sizeParam_    = nullptr;
    std::atomic<float>* mixParam_     = nullptr;
    std::atomic<float>* outParam_     = nullptr;
    std::atomic<float>* antifbParam_  = nullptr;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Spring4DReverbAudioProcessor)
};
