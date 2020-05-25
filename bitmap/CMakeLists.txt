#//-----------------------------------------------------------------------------
#//	BITMAP.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(bitmap_libraries)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
include(${CMAKE_MODULE_PATH}/detect_platform.cmake)

#build byteswap_client
set(OUTLIBNAME "bitmap_byteswap_client")

include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

target_sources(${OUTLIBNAME} PRIVATE ImageByteSwap.cpp)

#build bitmap_client
set(OUTLIBNAME "bitmap_client")

include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

if( X360 OR PS3 )
    #$File	"ImageByteSwap.cpp"					[$X360 || $PS3]
endif()
target_sources(${OUTLIBNAME} PRIVATE "bitmap.cpp")
target_sources(${OUTLIBNAME} PRIVATE "colorconversion.cpp")
target_sources(${OUTLIBNAME} PRIVATE "floatbitmap.cpp")
target_sources(${OUTLIBNAME} PRIVATE "floatbitmap2.cpp")
target_sources(${OUTLIBNAME} PRIVATE "floatbitmap3.cpp")
if( WINDOWS )
    #$File	"floatbitmap4.cpp"					[$WINDOWS]
endif()
target_sources(${OUTLIBNAME} PRIVATE "floatbitmap_bilateralfilter.cpp")
target_sources(${OUTLIBNAME} PRIVATE "floatcubemap.cpp")
target_sources(${OUTLIBNAME} PRIVATE "imageformat.cpp")
target_sources(${OUTLIBNAME} PRIVATE "psd.cpp")
target_sources(${OUTLIBNAME} PRIVATE "psheet.cpp")
target_sources(${OUTLIBNAME} PRIVATE "resample.cpp")
target_sources(${OUTLIBNAME} PRIVATE "tgaloader.cpp")
target_sources(${OUTLIBNAME} PRIVATE "texturepacker.cpp")
target_sources(${OUTLIBNAME} PRIVATE "tgawriter.cpp")

if( WINDOWS )
    #$Folder "Link Libraries" [$WINDOWS]
	#{
	#	$Lib	"$SRCDIR\lib\public\nvtc"			[!$WIN64]
	#	$Lib	"$SRCDIR\lib\public\nvtc64"			[$WIN64]
	#	$Lib	"bitmap_byteswap"
	#}
endif()