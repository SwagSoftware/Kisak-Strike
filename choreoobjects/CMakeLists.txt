#//-----------------------------------------------------------------------------
#//	MATHLIB.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(choreoobjects_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "choreoobjects_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

include_directories(../game/shared)
include_directories(../utils/common)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "${SRCDIR}/game/shared/choreoactor.cpp")
target_sources(${OUTLIBNAME} PRIVATE "${SRCDIR}/game/shared/choreochannel.cpp")
target_sources(${OUTLIBNAME} PRIVATE "${SRCDIR}/game/shared/choreoevent.cpp")
target_sources(${OUTLIBNAME} PRIVATE "${SRCDIR}/game/shared/choreoscene.cpp")
target_sources(${OUTLIBNAME} PRIVATE "${SRCDIR}/game/shared/sceneimage.cpp")