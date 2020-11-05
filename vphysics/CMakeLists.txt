#//-----------------------------------------------------------------------------
#//	VPHYSICS.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(kisakvphysics_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "kisakvphysics_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

#ivp = Ipion Virtual Physics. A physics company from Germany that went defunct, somehow valve is using this
include_directories(${SRCDIR}/ivp/ivp_intern)
include_directories(${SRCDIR}/ivp/ivp_collision)
include_directories(${SRCDIR}/ivp/ivp_physics)
include_directories(${SRCDIR}/ivp/ivp_surface_manager)
include_directories(${SRCDIR}/ivp/ivp_utility)
include_directories(${SRCDIR}/ivp/ivp_controller)
include_directories(${SRCDIR}/ivp/ivp_compact_builder)
include_directories(${SRCDIR}/ivp/havana/havok)
include_directories(${SRCDIR}/ivp/havana)

add_definitions(-DVPHYSICS_EXPORTS -DHAVANA_CONSTRAINTS -DHAVOK_MOPP)

if( LINUXALL )
    target_compile_options(${OUTBINNAME} PRIVATE -fpic -fno-semantic-interposition)
endif()

#Source files
target_sources(${OUTBINNAME} PRIVATE "convert.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/filesystem_helpers.cpp")
target_sources(${OUTBINNAME} PRIVATE "ledgewriter.cpp")
target_sources(${OUTBINNAME} PRIVATE "main.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_airboat.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_collide.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_constraint.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_controller_raycast_vehicle.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_environment.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_fluid.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_friction.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_material.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_motioncontroller.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_object.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_shadow.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_spring.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_vehicle.cpp")
target_sources(${OUTBINNAME} PRIVATE "physics_virtualmesh.cpp")
target_sources(${OUTBINNAME} PRIVATE "trace.cpp")
target_sources(${OUTBINNAME} PRIVATE "vcollide_parse.cpp")
target_sources(${OUTBINNAME} PRIVATE "vphysics_saverestore.cpp")

#Linker
#		$Lib "$LIBCOMMON/havana_constraints"
#		$Lib "$LIBCOMMON/hk_base"
#		$Lib "$LIBCOMMON/hk_math"
#		$Lib "$LIBCOMMON/ivp_compactbuilder"
#		$Lib "$LIBCOMMON/ivp_physics"
target_link_libraries(${OUTBINNAME} havana_constraints hk_base hk_math ivp_compactbuilder ivp_physics)
target_link_libraries(${OUTBINNAME} mathlib_client tier2_client libvstdlib_client)
