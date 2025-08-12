cmake_minimum_required(VERSION 3.30)

project(StructGRIS
    VERSION 1.0.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_INCLUDE_CURRENT_DIR 1)

# 1. Look for a system JUCE if any
if(NOT TARGET juce::juce_core)
  find_package(JUCE QUIET)
endif()

# 2. If not found, use bundled JUCE
if(NOT TARGET juce::juce_core)
  add_subdirectory(submodules/JUCE)
endif()

add_library(StructGRIS STATIC
    Containers/sg_AtomicUpdater.hpp
    Containers/sg_LogBuffer.cpp
    Containers/sg_LogBuffer.hpp
    Containers/sg_OwnedMap.hpp
    Containers/sg_StaticMap.hpp
    Containers/sg_StaticVector.hpp
    Containers/sg_StrongArray.hpp
    Containers/sg_TaggedAudioBuffer.hpp
    Containers/sg_ThreadSafeBuffer.hpp

    Data/StrongTypes/sg_CartesianVector.cpp
    Data/StrongTypes/sg_Dbfs.hpp
    Data/StrongTypes/sg_Degrees.hpp
    Data/StrongTypes/sg_Hz.hpp
    Data/StrongTypes/sg_Meters.hpp
    Data/StrongTypes/sg_OutputPatch.hpp
    Data/StrongTypes/sg_Radians.hpp
    Data/StrongTypes/sg_StrongFloat.hpp
    Data/StrongTypes/sg_StrongIndex.hpp
    Data/StrongTypes/sg_CartesianVector.hpp
    Data/StrongTypes/sg_SourceIndex.hpp

    Data/Quaternion.cpp
    Data/Quaternion.hpp
    Data/sg_AudioStructs.cpp
    Data/sg_AudioStructs.hpp
    Data/sg_CommandId.hpp
    Data/sg_constants.cpp
    Data/sg_constants.hpp
    Data/sg_LegacyLbapPosition.cpp
    Data/sg_LegacyLbapPosition.hpp
    Data/sg_LegacySpatFileFormat.cpp
    Data/sg_LegacySpatFileFormat.hpp
    Data/sg_LogicStrucs.cpp
    Data/sg_LogicStrucs.hpp
    Data/sg_Macros.hpp
    Data/sg_Narrow.hpp
    Data/sg_PolarVector.cpp
    Data/sg_PolarVector.hpp
    Data/sg_Position.cpp
    Data/sg_Position.hpp
    Data/sg_SpatMode.cpp
    Data/sg_SpatMode.hpp
    Data/sg_Triplet.hpp

    Utilities/ValueTreeUtilities.cpp
    Utilities/ValueTreeUtilities.hpp
)

target_include_directories(StructGRIS PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR} # for generated JuceHeader.h
)

target_link_libraries(StructGRIS
  PUBLIC
    juce::juce_core
    juce::juce_events
    juce::juce_audio_basics
    juce::juce_graphics
    juce::juce_dsp
    juce::juce_gui_basics
)

# Generate JuceHeader.h inside StructGRIS
file(GENERATE
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/JuceHeader.h
    CONTENT
[=[
#pragma once

#include <juce_core/juce_core.h>
#include <juce_events/juce_events.h>
#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_graphics/juce_graphics.h>
#include <juce_dsp/juce_dsp.h>
#include <juce_gui_basics/juce_gui_basics.h>

#if ! JUCE_DONT_DECLARE_PROJECTINFO
namespace ProjectInfo
{
    const char* const  projectName    = "StructGRIS";
    const char* const  companyName    = "Gris / SAT";
    const char* const  versionString  = "1.0.0";
    const int          versionNumber  = 0x100000;
}
#endif
]=]
)
