#//-----------------------------------------------------------------------------
#//	VGUI_SURFACELIB.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(vgui_surfacelib_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "vgui_surfacelib_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

if( LINUXALL )
    include_directories(/usr/include/freetype2)
    target_compile_options(${OUTLIBNAME} PRIVATE "-fpermissive")
    target_compile_options(${OUTLIBNAME} PRIVATE "-fPIC")
endif()

add_definitions(-DALLOW_TEXT_MODE=1)

target_sources(${OUTLIBNAME} PRIVATE "BitmapFont.cpp")
target_sources(${OUTLIBNAME} PRIVATE "FontAmalgam.cpp")
target_sources(${OUTLIBNAME} PRIVATE "fontmanager.cpp")
target_sources(${OUTLIBNAME} PRIVATE "FontEffects.cpp")
target_sources(${OUTLIBNAME} PRIVATE "FontEffects.h")
target_sources(${OUTLIBNAME} PRIVATE "fonttexturecache.cpp")
target_sources(${OUTLIBNAME} PRIVATE "texturedictionary.cpp")
#$File	"Win32Font.cpp"			[$WINDOWS]
#$File	"Win32Font_x360.cpp"	[$X360]
#$File	"ps3font.cpp"			[$PS3]
#$File	"osxfont.cpp"			[$OSXALL]
if( LINUXALL )
    target_sources(${OUTLIBNAME} PRIVATE "linuxfont.cpp")
endif()
