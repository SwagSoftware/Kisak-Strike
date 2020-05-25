#//-----------------------------------------------------------------------------
#//	TOGL.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(togl)
set(SRCDIR "..")
list(APPEND CMAKE_MODULE_PATH "${SRCDIR}/cmake")

set(OUTBINDIR "${SRCDIR}../game/bin")
set(OUTBINNAME "togl") #also the target name
set(TOGL_SRCDIR "${SRCDIR}/togl")
set(TOGL_INCDIR "${SRCDIR}/public/togl")

include("${CMAKE_MODULE_PATH}/source_dll_base.cmake")

include_directories("../")
include_directories("../public/togl")
if(OSXALL)
    add_definitions(-DTOGL_DLL_EXPORT)
else()
    add_definitions(-DTOGL_DLL_EXPORT -DPROTECTED_THINGS_ENABLE -Dstrncpy=use_Q_strncpy_instead -D_snprintf=use_Q_snprintf_instead)
endif()


#//-----------------------------------------------------------------------------
#// Sources & Headers
#//-----------------------------------------------------------------------------
if( GL )
    #headers
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/dx9asmtogl2.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/glmgr_flush.inl")

    #public headers
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/togl/rendermechanism.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/dxabstract.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/dxabstract_types.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/glbase.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/glentrypoints.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/glmgr.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/glmdebug.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/glmgrbasics.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/glmgrext.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/glmdisplay.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/glmdisplaydb.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/glfuncs.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/cglmtex.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/cglmfbo.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/cglmprogram.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/cglmbuffer.h")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_INCDIR}/cglmquery.h")
    #source
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/dx9asmtogl2.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/dxabstract.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/glentrypoints.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/glmgr.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/glmgrbasics.cpp")
    if( OSXALL )
        #headers
        target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/intelglmallocworkaround.h")
        target_sources(${OUTBINNAME} PRIVATE "${TOGL_SRCDIR}/mach_override.h")

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

target_link_libraries(${OUTBINNAME} "${LIBPUBLIC}/libtogl_client.so")
target_link_libraries(${OUTBINNAME} "${LIBPUBLIC}/tier2.a")
target_link_libraries(${OUTBINNAME} "${LIBPUBLIC}/mathlib_client.a")
