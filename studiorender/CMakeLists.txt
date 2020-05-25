#//-----------------------------------------------------------------------------
#//	STUDIORENDER.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(studiorender_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "studiorender_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

add_definitions(-DSTUDIORENDER_EXPORTS -DPROTECTED_THINGS_ENABLE)

target_sources(${OUTBINNAME} PRIVATE "studiorender.cpp")
target_sources(${OUTBINNAME} PRIVATE "studiorendercontext.cpp")
target_sources(${OUTBINNAME} PRIVATE "flexrenderdata.cpp")
target_sources(${OUTBINNAME} PRIVATE "r_studio.cpp")
target_sources(${OUTBINNAME} PRIVATE "r_studiodecal.cpp")
target_sources(${OUTBINNAME} PRIVATE "r_studiodraw.cpp")
target_sources(${OUTBINNAME} PRIVATE "r_studiodraw_computeflexedvertex.cpp")
target_sources(${OUTBINNAME} PRIVATE "r_studioflex.cpp")
target_sources(${OUTBINNAME} PRIVATE "r_studiosubd.cpp")
target_sources(${OUTBINNAME} PRIVATE "r_studiosubd_patches.cpp")
target_sources(${OUTBINNAME} PRIVATE "r_studiogettriangles.cpp")
target_sources(${OUTBINNAME} PRIVATE "r_studiolight.cpp")

target_link_libraries(${OUTBINNAME} vstdlib_client bitmap_client mathlib_client tier2_client tier3_client)