#//-----------------------------------------------------------------------------
#//	RESOURCEFILE.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(resourcefile_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "resourcefile_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -Wno-narrowing -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "resourcedictionary.cpp")
target_sources(${OUTLIBNAME} PRIVATE "resourcefile.cpp")
target_sources(${OUTLIBNAME} PRIVATE "resourceintrospection.cpp")
target_sources(${OUTLIBNAME} PRIVATE "resourcestream.cpp")