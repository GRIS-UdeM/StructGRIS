# submodules/StructGRIS/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

project(StructGRIS
    VERSION 1.0.0
    LANGUAGES CXX
)

add_library(StructGRIS STATIC
    Source/ValueTreeUtilities.cpp
    Source/ValueTreeUtilities.hpp
    Source/Quaternion.cpp
    Source/Quaternion.hpp
    Source/StrongTypes/sg_Radians.hpp
    Source/StrongTypes/sg_StrongFloat.hpp
)

target_include_directories(StructGRIS PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source>
)

target_compile_features(StructGRIS PUBLIC cxx_std_20)
