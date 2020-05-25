#//-----------------------------------------------------------------------------
#//	DMXLOADER.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(dmxloader_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "dmxloader_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

add_definitions(-DDMXLOADER_LIB)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "dmxattribute.cpp")
target_sources(${OUTLIBNAME} PRIVATE "dmxelement.cpp")
target_sources(${OUTLIBNAME} PRIVATE "dmxloader.cpp")
target_sources(${OUTLIBNAME} PRIVATE "dmxloadertext.cpp")
target_sources(${OUTLIBNAME} PRIVATE "dmxserializationdictionary.cpp")

target_sources(${OUTLIBNAME} PRIVATE "utlsoacontainer_serialization.cpp")
