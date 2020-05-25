#//-----------------------------------------------------------------------------
#//	LAUNCHER.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(launcher_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "launcher_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

set(NOSTINKYLINKIES "1") #link this project carefully ourselves

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

add_definitions(-DLAUNCHERONLY -DINCLUDE_SCALEFORM -DALLOW_TEXT_MODE=1)

if( LINUXALL )
    target_link_libraries(${OUTBINNAME} SDL2)
    #		// We run from the ./game dir, but want to look in the ./game/bin directory when loading libraries.
    #		//   To dump rpath/runpath of a library, use "chrpath --list blah.so" or "objdump -x blah.so" or "readelf -d bin/launcher.so"
    #		$GCC_ExtraLinkerFlags		"-Wl,--enable-new-dtags -Wl,-z,origin -Wl,-rpath,'$$ORIGIN' -L/usr/lib32 -L/usr/lib" [$LINUXALL]
    target_compile_options(${OUTBINNAME} PRIVATE "-Wl,--enable-new-dtags -Wl,-z,origin -Wl,-rpath,'$ORIGIN' -L/usr/lib32 -L/usr/lib")
endif()

target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/filesystem_init.cpp")
target_sources(${OUTBINNAME} PRIVATE "launcher.cpp")
target_sources(${OUTBINNAME} PRIVATE "reslistgenerator.cpp")

target_link_libraries(${OUTBINNAME} appframework_client tier1_client tier2_client tier3_client interfaces_client )
target_link_libraries(${OUTBINNAME} ${LIBPUBLIC}/libsteam_api.so) # Link to proprietary steamapi
if( NOT IS_LIB_PROJECT AND GL )
    target_link_libraries(${OUTBINNAME} ${LIBPUBLIC}/libtogl_client.so) # Link to precompiled togl
endif()
