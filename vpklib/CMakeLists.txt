#//-----------------------------------------------------------------------------
#//	VPKLIB.VPC
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(vpklib_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "vpklib_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)

target_sources(${OUTLIBNAME} PRIVATE "packedstore.cpp")
