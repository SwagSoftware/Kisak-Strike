#//-----------------------------------------------------------------------------
#//	SHADERLIB.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)
project(shaderlib_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(SHADERLIBDIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "shaderlib_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fpic -fno-semantic-interposition)
endif()

include_directories(../)
add_definitions(-DFAST_MATERIALVAR_ACCESS)

target_sources(${OUTLIBNAME} PRIVATE "BaseShader.cpp")
target_sources(${OUTLIBNAME} PRIVATE "ShaderDLL.cpp")
target_sources(${OUTLIBNAME} PRIVATE "shaderlib_cvar.cpp")