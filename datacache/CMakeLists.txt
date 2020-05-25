#//-----------------------------------------------------------------------------
#//	DATACACHE.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(datacache_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "datacache_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

add_definitions(-DMDLCACHE_DLL_EXPORT)

if( LINUXALL )
    target_compile_options(${OUTBINNAME} PRIVATE -shared -fPIC)
endif()

target_sources(${OUTBINNAME} PRIVATE "datacache.cpp")
target_sources(${OUTBINNAME} PRIVATE "mdlcache.cpp")
target_sources(${OUTBINNAME} PRIVATE "mdlcombine.cpp")
target_sources(${OUTBINNAME} PRIVATE "vtfcombine.cpp")
target_sources(${OUTBINNAME} PRIVATE "resourceaccesscontrol.cpp")
target_sources(${OUTBINNAME} PRIVATE "precachesystem.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/studio.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/studio_virtualmodel.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/common/studiobyteswap.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/utils/common/datalinker.cpp")

target_link_libraries(${OUTBINNAME} tier2_client tier3_client mathlib_client vstdlib_client)