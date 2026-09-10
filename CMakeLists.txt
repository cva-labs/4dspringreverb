cmake_minimum_required(VERSION 3.22)

project(Spring4DReverb VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ----------------------------------------------------------------------------
# JUCE-free console validation harness for the DSP engine
# ----------------------------------------------------------------------------
add_executable(SpringTest Test/SpringTest.cpp)
target_include_directories(SpringTest PRIVATE Source)
if (MSVC)
    target_compile_options(SpringTest PRIVATE /O2 /fp:fast /EHsc)
endif()

# ----------------------------------------------------------------------------
# JUCE 8 (downloaded automatically via FetchContent)
# ----------------------------------------------------------------------------
include(FetchContent)
FetchContent_Declare(JUCE
    GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
    GIT_TAG        8.0.6
    GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(JUCE)

juce_add_plugin(Spring4DReverb
    COMPANY_NAME             "CVA Labs"
    PLUGIN_MANUFACTURER_CODE CvaL
    PLUGIN_CODE              S4rv
    BUNDLE_ID                com.cvalabs.4dspringreverb
    VERSION                  1.0.0
    FORMATS                  VST3 Standalone
    PRODUCT_NAME             "4D Spring Reverb"
    DESCRIPTION              "Spring reverb with a 4D Lattice Spring Model"
    VST3_CATEGORIES          Fx
    IS_SYNTH                 FALSE
    NEEDS_MIDI_INPUT         FALSE
    NEEDS_MIDI_OUTPUT        FALSE
    IS_MIDI_EFFECT           FALSE
    COPY_PLUGIN_AFTER_BUILD  FALSE)

juce_add_binary_data(Spring4DAssets SOURCES assets/background.png)

target_sources(Spring4DReverb PRIVATE
    Source/PluginProcessor.cpp
    Source/PluginEditor.cpp)

target_compile_definitions(Spring4DReverb PUBLIC
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0)

target_link_libraries(Spring4DReverb
    PRIVATE
        juce::juce_audio_utils
        Spring4DAssets
    PUBLIC
        juce::juce_recommended_config_flags)

if (MSVC)
    target_compile_options(Spring4DReverb PRIVATE /fp:fast)
endif()
