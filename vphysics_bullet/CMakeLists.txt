#Custom Bullet Physics for Source Engine
#Originally made for gmod from: https://github.com/DrChat/Gmod-vphysics
#Forked by: https://github.com/dyanikoglu/source-sdk-bullet-physics and upgraded
#Adapted for use in the Kisak-Strike project.
cmake_minimum_required(VERSION 3.16)
project(bulletvphysics_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "bulletvphysics_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

include_directories(${SRCDIR}/public)
include_directories(${SRCDIR}/public/vphysics_bullet)
include_directories(${SRCDIR}/public/tier0)
include_directories(${SRCDIR}/public/tier1)
include_directories(${SRCDIR}/thirdparty/bullet3/src)


if( LINUXALL )
    target_compile_options(${OUTBINNAME} PRIVATE -fpic -fno-semantic-interposition)
endif()

#Source files
target_sources(${OUTBINNAME} PRIVATE "DebugDrawer.cpp")
target_sources(${OUTBINNAME} PRIVATE "DebugDrawer.cpp")
target_sources(${OUTBINNAME} PRIVATE "miscmath.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_Collision.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_CollisionSet.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_Constraint.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_DragController.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_Environment.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_FluidController.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_FrictionSnapshot.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_KeyParser.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_MotionController.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_Object.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_ObjectPairHash.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_PlayerController.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_ShadowController.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_SoftBody.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_SurfaceProps.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_VehicleAirboat.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_VehicleController.cpp")
target_sources(${OUTBINNAME} PRIVATE "Physics_VehicleControllerCustom.cpp")
target_sources(${OUTBINNAME} PRIVATE "StdAfx.cpp")

#Linker
target_link_libraries(${OUTBINNAME} libtier0_client tier1_client tier2_client libvstdlib_client mathlib_client )

#Bullet Links
target_link_libraries(${OUTBINNAME} BulletDynamics BulletSoftBody BulletCollision LinearMath)