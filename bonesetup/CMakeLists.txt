#//-----------------------------------------------------------------------------
#//	MATHLIB.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(bonesetup_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "bonesetup_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

include_directories(../public)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "bone_setup.cpp")
target_sources(${OUTLIBNAME} PRIVATE "bone_ik.cpp")
target_sources(${OUTLIBNAME} PRIVATE "bone_utils.cpp")
target_sources(${OUTLIBNAME} PRIVATE "bone_decode.cpp")
target_sources(${OUTLIBNAME} PRIVATE "bone_constraints.cpp")