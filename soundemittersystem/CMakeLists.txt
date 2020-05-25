#//-----------------------------------------------------------------------------
#//	SOUNDEMITTERSYSTEM.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(soundemittersystem_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "soundemittersystem_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

include_directories(${SRCDIR}/game/shared)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-DSOUNDEMITTERSYSTEM_EXPORTS -D_WINDOWS -DPROTECTED_THINGS_ENABLE)

target_sources(${OUTBINNAME} PRIVATE "soundemittersystembase.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/SoundParametersInternal.cpp")

target_link_libraries(${OUTBINNAME} tier2_client vstdlib_client)