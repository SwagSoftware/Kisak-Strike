#//-----------------------------------------------------------------------------
#//	LAUNCHER.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
#dont build this on dedicated build
if( DEDICATED )
    return()
endif()
project(launcher_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "launcher_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

set(NOSTINKYLINKIES "1") #link this project carefully ourselves

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
if( LINUXALL )
    # We run from the ./game dir, but want to look in the ./game/bin directory when loading libraries.
    # To dump rpath/runpath of a library, use "chrpath --list blah.so" or "objdump -x blah.so" or "readelf -d bin/launcher.so"
    #		$GCC_ExtraLinkerFlags		"-Wl,--enable-new-dtags -Wl,-z,origin -Wl,-rpath,'$$ORIGIN' -L/usr/lib32 -L/usr/lib" [$LINUXALL]
    #target_compile_options(${OUTBINNAME} PRIVATE "-Wl,--enable-new-dtags -Wl,-z,origin -Wl,-rpath,'$ORIGIN' -L/usr/lib32 -L/usr/lib")
    # This should be done before the target gets added.

    #lwss- disabled this so that libs in ${PUBLIC} will link. doesn't really matter with an open-source proj
    #set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    #set(CMAKE_INSTALL_RPATH "$ORIGIN")
endif()
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

add_definitions(-DLAUNCHERONLY -DALLOW_TEXT_MODE=1)
if( USE_SCALEFORM )
    add_definitions(-DINCLUDE_SCALEFORM)
elseif( USE_ROCKETUI )
    add_definitions(-DINCLUDE_ROCKETUI)
endif()

if( USE_KISAK_PHYSICS )
    add_definitions(-DUSE_KISAK_PHYSICS)
elseif( USE_BULLET_PHYSICS )
    add_definitions(-DUSE_BULLET_PHYSICS)
endif()

if( LINUXALL )
    target_link_libraries(${OUTBINNAME} SDL2)
endif()

target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/filesystem_init.cpp")
target_sources(${OUTBINNAME} PRIVATE "launcher.cpp")
target_sources(${OUTBINNAME} PRIVATE "reslistgenerator.cpp")

target_link_libraries(${OUTBINNAME} appframework_client tier1_client tier2_client tier3_client interfaces_client )
target_link_libraries(${OUTBINNAME} ${LIBPUBLIC}/libsteam_api.so) # Link to proprietary steamapi
if( NOT IS_LIB_PROJECT AND GL )
    target_link_libraries(${OUTBINNAME} libtogl_client) # Link to precompiled togl
endif()
