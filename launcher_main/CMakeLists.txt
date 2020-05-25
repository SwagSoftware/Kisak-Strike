#//-----------------------------------------------------------------------------
#//	LAUNCHER_MAIN.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(launcher_main)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINDIR "${SRCDIR}/../game")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)

if( WIN32 )
    set(OUTBINNAME "csgo")
elseif( WIN64 )
    set(OUTBINNAME "csgo_win64")
elseif( OSX32 )
    set(OUTBINNAME "csgo_osx")
elseif( OSX64 )
    set(OUTBINNAME "csgo_osx64")
elseif( LINUX32 )
    set(OUTBINNAME "csgo_linux")
elseif( LINUX64 )
    set(OUTBINNAME "csgo_linux64")
endif()

set(NOSTINKYLINKIES "1")
set(NOSKELETONBASE "1")

include(${CMAKE_MODULE_PATH}/source_exe_base.cmake)

target_sources(${OUTBINNAME} PRIVATE main.cpp)

target_link_libraries(${OUTBINNAME} dl) #link for dlopen.