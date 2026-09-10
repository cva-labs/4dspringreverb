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

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
Spring4DReverbAudioProcessor::Spring4DReverbAudioProcessor()
    : AudioProcessor (BusesProperties()
                          .withInput  ("Input",  juce::AudioChannelSet::mono(),   true)
                          .withOutput ("Output", juce::AudioChannelSet::stereo(), true)),
      apvts (*this, nullptr, "PARAMS", createLayout())
{
    driveParam_   = apvts.getRawParameterValue ("drive");
    tensionParam_ = apvts.getRawParameterValue ("tension");
    decayParam_   = apvts.getRawParameterValue ("decay");
    helixParam_   = apvts.getRawParameterValue ("helix");
    pickupParam_  = apvts.getRawParameterValue ("pickup");
    brightParam_  = apvts.getRawParameterValue ("brightness");
    sizeParam_    = apvts.getRawParameterValue ("size");
    mixParam_     = apvts.getRawParameterValue ("mix");
    outParam_     = apvts.getRawParameterValue ("output");
    antifbParam_  = apvts.getRawParameterValue ("antifb");
}

//==============================================================================
juce::AudioProcessorValueTreeState::ParameterLayout
Spring4DReverbAudioProcessor::createLayout()
{
    juce::AudioProcessorValueTreeState::ParameterLayout layout;

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "drive", 1 }, "Drive",
        juce::NormalisableRange<float> (0.0f, 1.0f, 0.001f), 0.5f));

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "tension", 1 }, "Tension",
        juce::NormalisableRange<float> (0.0f, 1.0f, 0.001f), 0.5f));

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "decay", 1 }, "Decay",
        juce::NormalisableRange<float> (0.4f, 12.0f, 0.01f, 0.35f), 2.0f));

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "helix", 1 }, "4D Coupling",
        juce::NormalisableRange<float> (0.0f, 1.0f, 0.001f), 0.5f));

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "pickup", 1 }, "Pickup Pos",
        juce::NormalisableRange<float> (0.0f, 1.0f, 0.001f), 0.35f));

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "brightness", 1 }, "Brightness",
        juce::NormalisableRange<float> (0.0f, 1.0f, 0.001f), 0.6f));

    layout.add (std::make_unique<juce::AudioParameterInt> (
        juce::ParameterID { "size", 1 }, "Spring Size", 24, 96, 48));

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "mix", 1 }, "Mix",
        juce::NormalisableRange<float> (0.0f, 1.0f, 0.001f), 0.5f));

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "output", 1 }, "Output",
        juce::NormalisableRange<float> (-24.0f, 6.0f, 0.1f), 0.0f));

    layout.add (std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { "antifb", 1 }, "Anti-Feedback",
        juce::NormalisableRange<float> (0.0f, 1.0f, 0.001f), 0.7f));

    return layout;
}

//==============================================================================
void Spring4DReverbAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    juce::ignoreUnused (samplesPerBlock);
    lastSegments_ = (sizeParam_ != nullptr)
                        ? (int) sizeParam_->load (std::memory_order_relaxed) : 48;
    engine_.prepare (sampleRate, lastSegments_);
    antifb_.prepare (sampleRate);
    wetBuf_.setSize (2, samplesPerBlock > 0 ? samplesPerBlock : 1);
}

bool Spring4DReverbAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
    const auto& in  = layouts.getMainInputChannelSet();
    const auto& out = layouts.getMainOutputChannelSet();

    if (out != juce::AudioChannelSet::stereo())
        return false;

    return in == juce::AudioChannelSet::mono() || in == juce::AudioChannelSet::stereo();
}

void Spring4DReverbAudioProcessor::processBlock (juce::AudioBuffer<float>& buffer,
                                                 juce::MidiBuffer&)
{
    juce::ScopedNoDenormals noDenormals;
    const int numSamples = buffer.getNumSamples();
    if (numSamples <= 0 || buffer.getNumChannels() == 0)
        return;

    spring4d::SpringLSM::Parameters p;
    p.drive      = driveParam_->load   (std::memory_order_relaxed);
    p.tension    = tensionParam_->load (std::memory_order_relaxed);
    p.decay      = decayParam_->load   (std::memory_order_relaxed);
    p.helix4d    = helixParam_->load   (std::memory_order_relaxed);
    p.pickupB    = pickupParam_->load  (std::memory_order_relaxed);
    p.brightness = brightParam_->load  (std::memory_order_relaxed);
    p.segments   = (int) sizeParam_->load (std::memory_order_relaxed);
    engine_.setParameters (p);

    if (p.segments != lastSegments_)           // re-string the spring
    {
        engine_.prepare (getSampleRate(), p.segments);
        lastSegments_ = p.segments;
    }

    const int numCh = buffer.getNumChannels();
    float* outL = buffer.getWritePointer (0);
    float* outR = (numCh > 1) ? buffer.getWritePointer (1) : nullptr;

    wetBuf_.setSize (2, numSamples, false, false, true);
    float* wL = wetBuf_.getWritePointer (0);
    float* wR = wetBuf_.getWritePointer (1);

    // pass 1: render the wet stereo tail (dry stays in the buffer channels)
    float wetL = 0.0f, wetR = 0.0f;
    for (int n = 0; n < numSamples; ++n)
    {
        float dry = outL[n];
        if (numCh > 1)
            dry = 0.5f * (dry + outR[n]);

        engine_.processBlock (&dry, &wetL, &wetR, 1);
        wL[n] = wetL;
        wR[n] = wetR;
    }

    // pass 2: multiband anti-feedback on the wet path only
    antifb_.setAmount (antifbParam_->load (std::memory_order_relaxed));
    antifb_.processBlock (wL, wR, numSamples);

    // pass 3: mix + output gain
    const float mix     = mixParam_->load (std::memory_order_relaxed);
    const float outGain = juce::Decibels::decibelsToGain (outParam_->load (std::memory_order_relaxed));
    const float dryGain = 1.0f - mix;

    for (int n = 0; n < numSamples; ++n)
    {
        outL[n] = (outL[n] * dryGain + wL[n] * mix) * outGain;
        if (outR != nullptr)
            outR[n] = (outR[n] * dryGain + wR[n] * mix) * outGain;
    }
}

//==============================================================================
double Spring4DReverbAudioProcessor::getTailLengthSeconds() const
{
    return (decayParam_ != nullptr) ? decayParam_->load() + 1.0 : 3.0;
}

juce::AudioProcessorEditor* Spring4DReverbAudioProcessor::createEditor()
{
    return new Spring4DReverbAudioProcessorEditor (*this);
}

//==============================================================================
void Spring4DReverbAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    juce::MemoryOutputStream mos (destData, false);
    apvts.state.writeToStream (mos);
}

void Spring4DReverbAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    auto tree = juce::ValueTree::readFromData (data, (std::size_t) juce::jmax (0, sizeInBytes));
    if (tree.isValid())
        apvts.replaceState (tree);
}

//==============================================================================
// JUCE plugin entry point: creates new instances of the processor.
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new Spring4DReverbAudioProcessor();
}

