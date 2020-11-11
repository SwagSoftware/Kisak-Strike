#//-----------------------------------------------------------------------------
#//	DEDICATED_MAIN.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(dedicated_main)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINDIR "${SRCDIR}/../game")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)

set(NOSTINKYLINKIES "1")
set(NOSKELETONBASE "1")

if( WIN32 )
    set(OUTBINNAME "srcds")
elseif( WIN64 )
    set(OUTBINNAME "srcds_x64")
elseif( OSX32 OR OSX64 )
    set(OUTBINNAME "srcds_osx")
elseif( LINUXALL )
    set(OUTBINNAME "srcds_linux")
endif()

#	$Linker
#	{
#		$AdditionalDependencies				"$BASE Advapi32.lib" [$WINDOWS]
#	}

include(${CMAKE_MODULE_PATH}/source_exe_base.cmake)

target_sources(${OUTBINNAME} PRIVATE main.cpp)
target_sources(${OUTBINNAME} PRIVATE ${SRCDIR}/common/SteamAppStartup.cpp)

if( LINUXALL )
    #        $Lib     "$SRCDIR/thirdparty/telemetry/lib/libtelemetryx86.link" [$LINUX]
    target_link_libraries(${OUTBINNAME} dl) #link for dlopen

    #valve snuck these in the linux makefiles instead of VPC
    target_link_libraries(${OUTBINNAME} libtier0_client libvstdlib_client)
endif()
