#//-----------------------------------------------------------------------------
#//	FGDLIB.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(fgdlib_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "fgdlib_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

include_directories(${SRCDIR}/utils/common)

target_sources(${OUTLIBNAME} PRIVATE "gamedata.cpp")
target_sources(${OUTLIBNAME} PRIVATE "gdclass.cpp")
target_sources(${OUTLIBNAME} PRIVATE "gdvar.cpp")
target_sources(${OUTLIBNAME} PRIVATE "inputoutput.cpp")
target_sources(${OUTLIBNAME} PRIVATE "wckeyvalues.cpp")