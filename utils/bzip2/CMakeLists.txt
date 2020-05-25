#//-----------------------------------------------------------------------------
#//	BZIP2.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(bzip2_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "bzip2_client")
set(OUTLIBDIR "${SRCDIR}/lib/common/linux64") #this one goes in common

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "blocksort.c")
target_sources(${OUTLIBNAME} PRIVATE "bzip2.c")
target_sources(${OUTLIBNAME} PRIVATE "bzlib.c")
target_sources(${OUTLIBNAME} PRIVATE "compress.c")
target_sources(${OUTLIBNAME} PRIVATE "crctable.c")
target_sources(${OUTLIBNAME} PRIVATE "decompress.c")
target_sources(${OUTLIBNAME} PRIVATE "huffman.c")
target_sources(${OUTLIBNAME} PRIVATE "randtable.c")