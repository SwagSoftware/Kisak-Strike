#//-----------------------------------------------------------------------------
#//	TOGL.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(libtogl_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "libtogl_client") #also the target name
set(OUTBINDIR ${SRCDIR}/../game/bin)

set(TOGL_SRCDIR "${SRCDIR}/togl")
set(TOGL_INCDIR "${SRCDIR}/public/togl")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

include_directories("../")
include_directories("../public/togl")

if(OSXALL)
    add_definitions(-DTOGL_DLL_EXPORT)
else()
    add_definitions(-DTOGL_DLL_EXPORT -DPROTECTED_THINGS_ENABLE -Dstrncpy=use_Q_strncpy_instead -D_snprintf=use_Q_snprintf_instead)
endif()

if(POSIX)
    target_compile_options(${OUTBINNAME} PRIVATE -fpic -fno-semantic-interposition -Wno-narrowing)
endif()

#//-----------------------------------------------------------------------------
#// Sources & Headers
#//-----------------------------------------------------------------------------
if( GL )
    #source
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/dx9asmtogl2.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/dxabstract.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/glentrypoints.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/glmgr.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/glmgrbasics.cpp")
    if( OSXALL )
        #source
        target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/glmgrcocoa.mm")
        target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/intelglmallocworkaround.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/mach_override.c")
    endif()
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/cglmtex.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/cglmfbo.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/cglmprogram.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/cglmbuffer.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/cglmquery.cpp")


endif()


#//-----------------------------------------------------------------------------
#// Link
#//-----------------------------------------------------------------------------

target_link_libraries(${OUTBINNAME} tier2_client libvstdlib_client mathlib_client )
