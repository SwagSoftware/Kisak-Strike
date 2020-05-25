#//-----------------------------------------------------------------------------
#//	RESOURCEFILE.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(responserules_runtime_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "responserules_runtime_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

include_directories(${SRCDIR}/public)

target_sources(${OUTLIBNAME} PRIVATE "rrrlib.cpp")
target_sources(${OUTLIBNAME} PRIVATE "response_types.cpp")
target_sources(${OUTLIBNAME} PRIVATE "response_types_internal.cpp")
target_sources(${OUTLIBNAME} PRIVATE "response_system.cpp")
target_sources(${OUTLIBNAME} PRIVATE "rr_speechconcept.cpp")
target_sources(${OUTLIBNAME} PRIVATE "rr_response.cpp")
target_sources(${OUTLIBNAME} PRIVATE "criteriaset.cpp")