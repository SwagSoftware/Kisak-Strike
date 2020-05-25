#//-----------------------------------------------------------------------------
#//	APPFRAMEWORK.cmake
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(appframework_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "appframework_client")
set(AF_SRCDIR "${SRCDIR}/appframework")
set(AF_INCDIR "${SRCDIR}/public/appframework")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

if( NOT IS_LIB_PROJECT AND GL )
    message(FATAL_ERROR "can't build appframework as dll")
    #		$AdditionalProjectDependencies	"$BASE;togl"	[!$IS_LIB_PROJECT && $GL]
endif()

if( (WINDOWS AND GL) OR LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
    add_definitions(-DVERSION_SAFE_STEAM_API_INTERFACES -DDX_TO_GL_ABSTRACTION)
endif()
if( CSTRIKE_TRUNK_BUILD OR CSTRIKE_STAGING_BUILD )
    add_definitions(-DALLOW_TEXT_MODE=1)
endif()

#Sources
include_directories(${SRCDIR}/thirdparty/SDL2)
target_sources(${OUTLIBNAME} PRIVATE "${AF_SRCDIR}/AppSystemGroup.cpp")
target_sources(${OUTLIBNAME} PRIVATE "${SRCDIR}/public/filesystem_init.cpp")
if( WINDOWS )
endif()
if( POSIX )
    target_sources(${OUTLIBNAME} PRIVATE "${AF_SRCDIR}/posixapp.cpp")
endif()
if( SDL )
    target_sources(${OUTLIBNAME} PRIVATE "${AF_SRCDIR}/sdlmgr.cpp")
endif()
if( OSXALL )
    if( SDL )
        target_sources(${OUTLIBNAME} PRIVATE "${AF_SRCDIR}/glmrendererinfo_osx.mm")
    else()
        target_sources(${OUTLIBNAME} PRIVATE "${AF_SRCDIR}/cocoamgr.mm")
    endif()
endif()

#$Folder	"Link Libraries"
#{
#    $ImpLib togl [!$IS_LIB_PROJECT && $GL]
#    $ImpLib SDL2 [!$IS_LIB_PROJECT && $SDL]
#}