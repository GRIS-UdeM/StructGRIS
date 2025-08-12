cmake_minimum_required(VERSION 3.30)

project(StructGRIS
    VERSION 1.0.0
    LANGUAGES CXX
)

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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source>
)

target_compile_features(StructGRIS PUBLIC cxx_std_20)
