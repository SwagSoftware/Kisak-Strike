#//-----------------------------------------------------------------------------
#//	FILESYSTEM_STDIO.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(filesystem_stdio_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "filesystem_stdio_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

add_definitions(-DFILESYSTEM_STDIO_EXPORTS -DDONT_PROTECT_FILEIO_FUNCTIONS -DPROTECTED_THINGS_ENABLE)

target_sources(${OUTBINNAME} PRIVATE "basefilesystem.cpp")
target_sources(${OUTBINNAME} PRIVATE "basefilesystemasync.cpp")
target_sources(${OUTBINNAME} PRIVATE "filetracker.cpp")
target_sources(${OUTBINNAME} PRIVATE "filesystemasync.cpp")		#[!$PS3]
target_sources(${OUTBINNAME} PRIVATE "filesystem_stdio.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/kevvaluescompiler.cpp") #[!$PS3]
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/zip_utils.cpp")	#[!$PS3]
target_sources(${OUTBINNAME} PRIVATE "QueuedLoader.cpp")
if( POSIX )
    target_sources(${OUTBINNAME} PRIVATE "linux_support.cpp")
endif()

target_link_libraries(${OUTBINNAME} tier2_client vpklib_client vstdlib_client)