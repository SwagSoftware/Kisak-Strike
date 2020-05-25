#//--------------------------------------------------------------------------------
#//	PARTICLES.VPC
#//
#//  full particles.lib for the client.
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(particles_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "particles_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC)
endif()

target_sources(${OUTLIBNAME} PRIVATE "builtin_constraints.cpp")
target_sources(${OUTLIBNAME} PRIVATE "builtin_initializers.cpp")
target_sources(${OUTLIBNAME} PRIVATE "builtin_particle_emitters.cpp")
target_sources(${OUTLIBNAME} PRIVATE "builtin_particle_forces.cpp")
target_sources(${OUTLIBNAME} PRIVATE "addbuiltin_ops.cpp")
target_sources(${OUTLIBNAME} PRIVATE "builtin_particle_ops.cpp")
target_sources(${OUTLIBNAME} PRIVATE "builtin_particle_render_ops.cpp")
target_sources(${OUTLIBNAME} PRIVATE "particle_snapshot.cpp")
target_sources(${OUTLIBNAME} PRIVATE "particle_sort.cpp")
target_sources(${OUTLIBNAME} PRIVATE "particles.cpp")

#	$Folder "Link Libraries" [$WINDOWS||$X360]
#	{
#		$Lib "bitmap"
#	}