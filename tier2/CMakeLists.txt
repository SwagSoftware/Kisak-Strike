#//-----------------------------------------------------------------------------
#//	TIER2.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(tier2_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "tier2_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

include_directories(${SRCDIR}/public/tier2)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "beamsegdraw.cpp")
target_sources(${OUTLIBNAME} PRIVATE "defaultfilesystem.cpp")
target_sources(${OUTLIBNAME} PRIVATE "fileutils.cpp")
target_sources(${OUTLIBNAME} PRIVATE "keybindings.cpp")
target_sources(${OUTLIBNAME} PRIVATE "${SRCDIR}/public/map_utils.cpp")
target_sources(${OUTLIBNAME} PRIVATE "${SRCDIR}/public/materialsystem/MaterialSystemUtil.cpp")
target_sources(${OUTLIBNAME} PRIVATE "meshutils.cpp")
target_sources(${OUTLIBNAME} PRIVATE "p4helpers.cpp")
target_sources(${OUTLIBNAME} PRIVATE "renderutils.cpp")
target_sources(${OUTLIBNAME} PRIVATE "riff.cpp")
target_sources(${OUTLIBNAME} PRIVATE "soundutils.cpp")
target_sources(${OUTLIBNAME} PRIVATE "tier2.cpp")
target_sources(${OUTLIBNAME} PRIVATE "tier2_logging.cpp")
target_sources(${OUTLIBNAME} PRIVATE "tokenreader.cpp")
target_sources(${OUTLIBNAME} PRIVATE "util_init.cpp")
target_sources(${OUTLIBNAME} PRIVATE "utlstreambuffer.cpp")
target_sources(${OUTLIBNAME} PRIVATE "vconfig.cpp")
target_sources(${OUTLIBNAME} PRIVATE "schemaclassbinding.cpp")
target_sources(${OUTLIBNAME} PRIVATE "socketcreator.cpp")
target_sources(${OUTLIBNAME} PRIVATE "resourceprecacher.cpp")
target_sources(${OUTLIBNAME} PRIVATE "interval.cpp")