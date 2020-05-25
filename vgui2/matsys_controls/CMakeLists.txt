#//-----------------------------------------------------------------------------
#//	MATSYS_CONTROLS.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(matsys_controls_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "matsys_controls_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "assetpicker.cpp")
target_sources(${OUTLIBNAME} PRIVATE "baseassetpicker.cpp")
target_sources(${OUTLIBNAME} PRIVATE "colorpickerpanel.cpp")
target_sources(${OUTLIBNAME} PRIVATE "curveeditorpanel.cpp")
target_sources(${OUTLIBNAME} PRIVATE "gamefiletreeview.cpp")
target_sources(${OUTLIBNAME} PRIVATE "manipulator.cpp")
target_sources(${OUTLIBNAME} PRIVATE "matsyscontrols.cpp")
target_sources(${OUTLIBNAME} PRIVATE "mdlpanel.cpp")
target_sources(${OUTLIBNAME} PRIVATE "mdlpicker.cpp")
target_sources(${OUTLIBNAME} PRIVATE "mdlsequencepicker.cpp")
target_sources(${OUTLIBNAME} PRIVATE "particlepicker.cpp")
target_sources(${OUTLIBNAME} PRIVATE "picker.cpp")
target_sources(${OUTLIBNAME} PRIVATE "potterywheelpanel.cpp")
target_sources(${OUTLIBNAME} PRIVATE "proceduraltexturepanel.cpp")
target_sources(${OUTLIBNAME} PRIVATE "QCGenerator.cpp")
target_sources(${OUTLIBNAME} PRIVATE "sequencepicker.cpp")
target_sources(${OUTLIBNAME} PRIVATE "sheetsequencepanel.cpp")
target_sources(${OUTLIBNAME} PRIVATE "tgapicker.cpp")
target_sources(${OUTLIBNAME} PRIVATE "tgapreviewpanel.cpp")
target_sources(${OUTLIBNAME} PRIVATE "vmtpicker.cpp")
target_sources(${OUTLIBNAME} PRIVATE "vmtpreviewpanel.cpp")
target_sources(${OUTLIBNAME} PRIVATE "vtfpicker.cpp")
target_sources(${OUTLIBNAME} PRIVATE "vtfpreviewpanel.cpp")
target_sources(${OUTLIBNAME} PRIVATE "vmtpanel.cpp")

if( WINDOWS )
    #		$Lib	bitmap
endif()