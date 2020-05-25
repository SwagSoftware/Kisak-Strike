#//-----------------------------------------------------------------------------
#//	DMSERIALIZERS.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(dmserializers_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "dmserializers_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

add_definitions(-DDMSERIALIZERS_LIB)

target_compile_options(${OUTLIBNAME} PRIVATE -fpermissive)

target_sources(${OUTLIBNAME} PRIVATE "dmebaseimporter.cpp")
target_sources(${OUTLIBNAME} PRIVATE "dmserializers.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importactbusy.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importcommentary.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importkeyvaluebase.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importmks.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importsfmv1.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importsfmv2.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importsfmv3.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importsfmv4.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importsfmv5.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importsfmv6.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importsfmv7.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importsfmv8.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importsfmv9.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importtex.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importvmf.cpp")
target_sources(${OUTLIBNAME} PRIVATE "importvmt.cpp")