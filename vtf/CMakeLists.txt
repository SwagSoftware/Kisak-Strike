#//-----------------------------------------------------------------------------
#//	VTF.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(vtf_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "vtf_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

if( WINDOWS )
    #		$AdditionalIncludeDirectories	"$BASE;$SRCDIR\dx9sdk\include"			[$WINDOWS]
    #		$PreprocessorDefinitions		"$BASE;fopen=dont_use_fopen"			[$WINDOWS]
endif()

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "convert_x360.cpp")
if( WINDOWS )
    #$File	"s3tc_decode.cpp" 	[$WINDOWS]
endif()
target_sources(${OUTLIBNAME} PRIVATE "vtf.cpp")
#$File	"vtf_x360.cpp"		[$X360]
#$File	"vtf_ps3.cpp"		[$PS3]

if( WINDOWS )
    #	$Folder "Link Libraries" [$X360]
    #	{
    #		$File	"$SRCDIR\lib\public\bitmap_360.lib"
    #		$File	"$SRCDIR\lib\public\mathlib_360.lib"
    #		$File	"$SRCDIR\lib\public\tier2_360.lib"
    #	}
endif()