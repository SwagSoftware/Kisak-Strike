#//-----------------------------------------------------------------------------
#//	MESHUTILS.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(meshutils_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "meshutils_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

include_directories(${SRCDIR}/public/meshutils)

target_sources(${OUTLIBNAME} PRIVATE "atlas.cpp")
target_sources(${OUTLIBNAME} PRIVATE "clipmesh.cpp")
target_sources(${OUTLIBNAME} PRIVATE "convexhull.cpp")
target_sources(${OUTLIBNAME} PRIVATE "mesh.cpp")
target_sources(${OUTLIBNAME} PRIVATE "simplify.cpp")
target_sources(${OUTLIBNAME} PRIVATE "vertexops.cpp")
target_sources(${OUTLIBNAME} PRIVATE "uvparam.cpp")
target_sources(${OUTLIBNAME} PRIVATE "${SRCDIR}/public/interpolatortypes.cpp")
