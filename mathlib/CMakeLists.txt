#//-----------------------------------------------------------------------------
#//	MATHLIB.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(mathlib_libraries)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
include(${CMAKE_MODULE_PATH}/detect_platform.cmake)

## Build the regular mathlib ##
set(OUTLIBNAME "mathlib_client")
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)
include_directories(../public/mathlib)
target_compile_definitions(${OUTLIBNAME} PRIVATE -DMATHLIB_LIB)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -Wno-narrowing -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "expressioncalculator.cpp")
target_sources(${OUTLIBNAME} PRIVATE "color_conversion.cpp")
target_sources(${OUTLIBNAME} PRIVATE "cholesky.cpp")
target_sources(${OUTLIBNAME} PRIVATE "halton.cpp")
target_sources(${OUTLIBNAME} PRIVATE "lightdesc.cpp")
target_sources(${OUTLIBNAME} PRIVATE "mathlib_base.cpp")
target_sources(${OUTLIBNAME} PRIVATE "powsse.cpp")
target_sources(${OUTLIBNAME} PRIVATE "sparse_convolution_noise.cpp")
target_sources(${OUTLIBNAME} PRIVATE "sseconst.cpp")
target_sources(${OUTLIBNAME} PRIVATE "sse.cpp")
target_sources(${OUTLIBNAME} PRIVATE "ssenoise.cpp")
target_sources(${OUTLIBNAME} PRIVATE "anorms.cpp")
target_sources(${OUTLIBNAME} PRIVATE "bumpvects.cpp")
target_sources(${OUTLIBNAME} PRIVATE "IceKey.cpp")
target_sources(${OUTLIBNAME} PRIVATE "kdop.cpp")
target_sources(${OUTLIBNAME} PRIVATE "imagequant.cpp")
target_sources(${OUTLIBNAME} PRIVATE "spherical.cpp")
target_sources(${OUTLIBNAME} PRIVATE "polyhedron.cpp")
target_sources(${OUTLIBNAME} PRIVATE "quantize.cpp")
target_sources(${OUTLIBNAME} PRIVATE "randsse.cpp")
target_sources(${OUTLIBNAME} PRIVATE "simdvectormatrix.cpp")
target_sources(${OUTLIBNAME} PRIVATE "vmatrix.cpp")
target_sources(${OUTLIBNAME} PRIVATE "almostequal.cpp")
target_sources(${OUTLIBNAME} PRIVATE "simplex.cpp")
target_sources(${OUTLIBNAME} PRIVATE "eigen.cpp")
if( NOT OSX32 )
    target_sources(${OUTLIBNAME} PRIVATE "box_buoyancy.cpp") #[!$OSX32] // doesn't compile in debug under GCC 4.2.X)
endif()
target_sources(${OUTLIBNAME} PRIVATE "camera.cpp")
target_sources(${OUTLIBNAME} PRIVATE "planefit.cpp")
target_sources(${OUTLIBNAME} PRIVATE "polygon.cpp")
target_sources(${OUTLIBNAME} PRIVATE "volumeculler.cpp")
target_sources(${OUTLIBNAME} PRIVATE "transform.cpp")
target_sources(${OUTLIBNAME} PRIVATE "sphere.cpp")
target_sources(${OUTLIBNAME} PRIVATE "capsule.cpp")



## Build the extended mathlib ##
set(OUTLIBNAME "mathlib_extended_client")
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

target_compile_definitions(${OUTLIBNAME} PRIVATE -DMATHLIB_EXTENDED_LIB -DSOURCE1)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "disjoint_set_forest.cpp")
target_sources(${OUTLIBNAME} PRIVATE "dynamictree.cpp")
target_sources(${OUTLIBNAME} PRIVATE "eigen.cpp")
target_sources(${OUTLIBNAME} PRIVATE "simdvectormatrix.cpp")
target_sources(${OUTLIBNAME} PRIVATE "femodel.cpp")
target_sources(${OUTLIBNAME} PRIVATE "femodelbuilder.cpp")
target_sources(${OUTLIBNAME} PRIVATE "feagglomerator.cpp")
target_sources(${OUTLIBNAME} PRIVATE "svd.cpp")
target_sources(${OUTLIBNAME} PRIVATE "transform.cpp")
target_sources(${OUTLIBNAME} PRIVATE "femodeldesc.cpp")
target_sources(${OUTLIBNAME} PRIVATE "softbody.cpp")
target_sources(${OUTLIBNAME} PRIVATE "softbodyenvironment.cpp")