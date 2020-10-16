#//-----------------------------------------------------------------------------
#//	SCALEFORMUI.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

#DUMMY VERSION FOR CLION - WILL NOT BUILD! MISSING SCALEFORM SDK.
project(scaleformui_client)
cmake_minimum_required(VERSION 3.16)

set(SRCDIR "${CMAKE_CURRENT_SOURCE_DIR}/../")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINDIR ${SRCDIR}/../game/bin)
set(OUTBINNAME "scaleform_dummy")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/movieslot.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/scaleformuicursorimpl.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/scaleformuihighlevelimpl.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/scaleformuiimage.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/scaleformuiinitimpl.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/scaleformuiintegration.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/scaleformuimovieimpl.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/scaleformuimovieslotimpl.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/scaleformuiparamsimpl.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/scaleformuirendererimpl.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/scaleformuitextobjimpl.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/scaleformuitranslationimpl.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/scaleformuivalueimpl.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/sfuiavatarimage.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/sfuichromehtmlimage.cpp)
target_sources(${OUTBINNAME} PRIVATE ./scaleformuiimpl/sfuiinventoryimage.cpp)

#Don't actually build during `make all`
set_target_properties(${OUTBINNAME} PROPERTIES EXCLUDE_FROM_ALL TRUE)