#//-----------------------------------------------------------------------------
#//	SCENEFILECACHE.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(scenefilecache_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "scenefilecache_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

include_directories(${SRCDIR}/game/shared)
add_definitions(-D_WINDOWS -DPROTECTED_THINGS_ENABLE)

target_sources(${OUTBINNAME} PRIVATE "SceneFileCache.cpp")