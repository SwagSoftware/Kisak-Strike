#//-----------------------------------------------------------------------------
#//	quickhull.vpc
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(quickhull_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "quickhull_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

include_directories(${SRCDIR}/public)
include_directories(${SRCDIR}/common)
include_directories(${SRCDIR}/public/quickhull)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

# the ones here without code can be removed.
target_sources(${OUTLIBNAME} PRIVATE "qhTypes.h")
target_sources(${OUTLIBNAME} PRIVATE "qhMath.h")
target_sources(${OUTLIBNAME} PRIVATE "qhMath.inl")
target_sources(${OUTLIBNAME} PRIVATE "qhMath.cpp")
target_sources(${OUTLIBNAME} PRIVATE "qhMemory.h")
target_sources(${OUTLIBNAME} PRIVATE "qhMemory.inl")
target_sources(${OUTLIBNAME} PRIVATE "qhMemory.cpp")
target_sources(${OUTLIBNAME} PRIVATE "qhArray.h")
target_sources(${OUTLIBNAME} PRIVATE "qhArray.inl")
target_sources(${OUTLIBNAME} PRIVATE "qhList.h")
target_sources(${OUTLIBNAME} PRIVATE "qhList.inl")
target_sources(${OUTLIBNAME} PRIVATE "qhHalfEdge.h")
target_sources(${OUTLIBNAME} PRIVATE "qhHalfEdge.cpp")
target_sources(${OUTLIBNAME} PRIVATE "qhConvex.h")
target_sources(${OUTLIBNAME} PRIVATE "qhConvex.inl")
target_sources(${OUTLIBNAME} PRIVATE "qhConvex.cpp")
target_sources(${OUTLIBNAME} PRIVATE "qhMass.h")
target_sources(${OUTLIBNAME} PRIVATE "qhMass.cpp")