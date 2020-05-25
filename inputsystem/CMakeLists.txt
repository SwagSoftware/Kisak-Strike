#//-----------------------------------------------------------------------------
#//	INPUTSYSTEM.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(inputsystem_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "inputsystem_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

add_definitions(-DNO_STRING_T -DVERSION_SAFE_STEAM_API_INTERFACES -DPROTECTED_THINGS_ENABLE)

if( LINUXALL )
    target_link_libraries(${OUTBINNAME} SDL2)
endif()

target_sources(${OUTBINNAME} PRIVATE "inputstacksystem.cpp")
target_sources(${OUTBINNAME} PRIVATE "inputsystem.cpp")
target_sources(${OUTBINNAME} PRIVATE "inputsystem.h")
if( NOT POSIX )
    #$File	"joystick.cpp"			[!$POSIX]
endif()
if( OSXALL )
    #$File	"joystick_osx.cpp"		[$OSXALL]
endif()
if( LINUXALL )
    target_sources(${OUTBINNAME} PRIVATE "joystick_linux.cpp")
endif()
target_sources(${OUTBINNAME} PRIVATE "steamcontroller.cpp")
target_sources(${OUTBINNAME} PRIVATE "key_translation.cpp")
target_sources(${OUTBINNAME} PRIVATE "key_translation.h")
if( NOT POSIX )
    #$File	"xcontroller.cpp"		[!$POSIX]
endif()
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/common/platforminputdevice.cpp")

target_link_libraries(${OUTBINNAME} tier2_client mathlib_client)
#Requires evil proprietary link to libsteam_api
target_link_libraries(${OUTBINNAME} ${LIBPUBLIC}/libsteam_api.so)