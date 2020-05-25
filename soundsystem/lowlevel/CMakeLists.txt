#//-----------------------------------------------------------------------------
#//	SOUNDSYSTEM_LOWLEVEL.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(soundsystem_lowlevel)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "soundsystem_lowlevel")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

#windows
#$Include "$SRCDIR\vpc_scripts\dxsdk_macros.vpc"
#$Configuration
#{
#	$Compiler
#	{
#		$AdditionalIncludeDirectories	"$BASE;$SRCDIR\thirdparty\dxsdk\include;$DXSDKINCLUDE"					[$WINDOWS]
#	}
#}

#$File	"device_dsound.cpp" [$WINDOWS]
#$File	"device_xaudio2.cpp" [$WINDOWS]
if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

if( POSIX )
    target_sources(${OUTLIBNAME} PRIVATE "device_sdl.cpp")
endif()
target_sources(${OUTLIBNAME} PRIVATE "device_null.cpp")
target_sources(${OUTLIBNAME} PRIVATE "mix.cpp")
target_sources(${OUTLIBNAME} PRIVATE "simple_filter.cpp")
if( WINDOWS )
    #$File	"windows_audio.cpp" [$WINDOWS]
endif()