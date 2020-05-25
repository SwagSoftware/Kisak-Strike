#//-----------------------------------------------------------------------------
#//	localize.dll project script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(localize_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "localize_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

target_sources(${OUTBINNAME} PRIVATE "localize.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/registry.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/UnicodeFileHelpers.cpp")

target_link_libraries(${OUTBINNAME} tier2_client vstdlib_client)