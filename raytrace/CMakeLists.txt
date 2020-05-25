#//-----------------------------------------------------------------------------
#//	RAYTRACE.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(raytrace_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "raytrace_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

include_directories(${SRCDIR}/utils/common)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -Wno-narrowing -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "raytrace.cpp")
target_sources(${OUTLIBNAME} PRIVATE "trace2.cpp")
target_sources(${OUTLIBNAME} PRIVATE "trace3.cpp")