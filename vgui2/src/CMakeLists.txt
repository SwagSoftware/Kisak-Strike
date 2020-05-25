#//-----------------------------------------------------------------------------
#//	VGUI_DLL.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(vgui2_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "vgui2_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

if( LINUXALL )
    include_directories(/usr/include/freetype2)
endif()
include_directories(${SRCDIR}/thirdparty)
#$PreprocessorDefinitions		"$BASE;DONT_PROTECT_FILEIO_FUNCTIONS"

if( LINUXALL )
    target_link_libraries(${OUTBINNAME} SDL2)
endif()

target_sources(${OUTBINNAME} PRIVATE "Bitmap.cpp")
target_sources(${OUTBINNAME} PRIVATE "Border.cpp")
target_sources(${OUTBINNAME} PRIVATE "ScalableImageBorder.cpp")
target_sources(${OUTBINNAME} PRIVATE "ImageBorder.cpp")
target_sources(${OUTBINNAME} PRIVATE "fileimage.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/filesystem_helpers.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/filesystem_init.cpp")
if( WINDOWS )
    #$File	"htmlwindow.cpp"				[$WINDOWS]
endif()
if( WINDOWS OR X360 OR PS3 OR LINUXALL )
    target_sources(${OUTBINNAME} PRIVATE "InputWin32.cpp")
endif()
if( OSXALL )
    #$File	"inputosx.cpp" 					[$OSXALL]
endif()
target_sources(${OUTBINNAME} PRIVATE "MemoryBitmap.cpp")
target_sources(${OUTBINNAME} PRIVATE "Memorybitmap.h")
target_sources(${OUTBINNAME} PRIVATE "MessageListener.cpp")
target_sources(${OUTBINNAME} PRIVATE "Scheme.cpp")
if( WINDOWS )
    #$File	"Surface.cpp"					[$WINDOWS]
endif()
if( WINDOWS OR X360 OR PS3 )
    #$File	"System.cpp"					[$WINDOWS||$X360||$PS3]
endif()
if( POSIX )
    target_sources(${OUTBINNAME} PRIVATE "system_posix.cpp" )
endif()
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/UnicodeFileHelpers.cpp")
target_sources(${OUTBINNAME} PRIVATE "vgui.cpp")
target_sources(${OUTBINNAME} PRIVATE "vgui_internal.cpp")
target_sources(${OUTBINNAME} PRIVATE "vgui_key_translation.cpp")
target_sources(${OUTBINNAME} PRIVATE "VPanel.cpp")
target_sources(${OUTBINNAME} PRIVATE "VPanelWrapper.cpp")
target_sources(${OUTBINNAME} PRIVATE "keyrepeat.cpp")

target_link_libraries(${OUTBINNAME} vgui_surfacelib_client tier2_client tier3_client vstdlib_client)