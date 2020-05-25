cmake_minimum_required(VERSION 3.17)
project(tier3_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "tier3_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -Wno-narrowing -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "tier3.cpp")
target_sources(${OUTLIBNAME} PRIVATE "mdlutils.cpp")
target_sources(${OUTLIBNAME} PRIVATE "choreoutils.cpp")
target_sources(${OUTLIBNAME} PRIVATE "scenetokenprocessor.cpp")
target_sources(${OUTLIBNAME} PRIVATE "studiohdrstub.cpp")