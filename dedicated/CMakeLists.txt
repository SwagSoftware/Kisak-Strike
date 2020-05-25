#//-----------------------------------------------------------------------------
#//	DEDICATED.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(dedicated_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "dedicated_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

include_directories(${SRCDIR}/engine)
add_definitions(-DDEDICATED -DLAUNCHERONLY)

#linker
if( WINDOWS )
    #$AdditionalDependencies				"$BASE wsock32.lib odbc32.lib odbccp32.lib winmm.lib" [$WINDOWS]
endif()
if( OSXALL )
    #$SystemLibraries                     "iconv" [$OSXALL]
endif()

#Source
if( WINDOWS )
    target_sources(${OUTBINNAME} PRIVATE "dedicated.rc")
endif()
if( POSIX )
    target_sources(${OUTBINNAME} PRIVATE "sys_linux.cpp")
endif()
target_sources(${OUTBINNAME} PRIVATE "filesystem.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/filesystem_init.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/common/netapi.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/common/SteamAppStartup.cpp")
target_sources(${OUTBINNAME} PRIVATE "sys_common.cpp")
target_sources(${OUTBINNAME} PRIVATE "sys_ded.cpp")
target_sources(${OUTBINNAME} PRIVATE "sys_subproc.cpp")
if( WINDOWS )
    target_sources(${OUTBINNAME} PRIVATE "sys_windows.cpp" [$WINDOWS])
endif()

## Console ##
target_sources(${OUTBINNAME} PRIVATE "console/conproc.cpp")
target_sources(${OUTBINNAME} PRIVATE "console/textconsole.cpp")
if( POSIX )
    target_sources(${OUTBINNAME} PRIVATE "console/TextConsoleUnix.cpp")
endif()
if( WINDOWS )
    target_sources(${OUTBINNAME} PRIVATE "console/TextConsoleWin32.cpp")
endif()

## VGUI ##
if( WINDOWS OR POSIX )
    target_sources(${OUTBINNAME} PRIVATE "vgui/CreateMultiplayerGameServerPage.cpp")
    target_sources(${OUTBINNAME} PRIVATE "vgui/MainPanel.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/vgui_controls/vgui_controls.cpp")
    target_sources(${OUTBINNAME} PRIVATE "vgui/vguihelpers.cpp")
endif()

## Filesystem ##
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/filesystem/filetracker.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/filesystem/basefilesystem.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/filesystem/basefilesystemasync.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/filesystem/filesystem_stdio.cpp")
if( NOT POSIX )
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/filesystem/filesystem_steam.cpp")
endif()
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/filesystem/filesystemasync.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/zip_utils.cpp")
if( POSIX )
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/filesystem/linux_support.cpp")
endif()

#SKIPPING - NEED THESE FIRST
#$Lib	appframework
#$Lib	dmxloader
#$ImplibExternal	steam_api							[ ( $WIN32 || $POSIX || $PS3 ) && !$NO_STEAM ]
#$ImplibExternal	steam_api64							[ $WIN64 && !$NO_STEAM ]
#$Lib	mathlib
#$Lib 	tier2
#$Lib	tier3
#$Lib 	vpklib
#$Lib	vgui_controls [$WINDOWS||$OSXALL]

if( (NOT DEFINED NO_STEAM) OR (${NO_STEAM})  )
    #grug
else()
    message(FATAL_ERROR "CMake steam_api integration is disabled.")
endif()

target_link_libraries(${OUTBINNAME} appframework_client dmxloader_client mathlib_client tier2_client tier3_client vstdlib_client vpklib_client)

if( WINDOWS OR OSXALL )
    target_link_libraries(${OUTBINNAME} vgui_controls_client)
endif()