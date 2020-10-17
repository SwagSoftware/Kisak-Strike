#//-----------------------------------------------------------------------------
#//	SHADERDX8.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(shaderapidx9_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "shaderapidx9_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

if( WINDOWS )
    if( GL )
    else()
        include_directories(${SRCDIR}/dx9sdk/include)
        add_definitions(-DUSE_ACTUAL_DX)
    endif()
    include_directories(${SRCDIR}/thirdparty/nvidia/nvapi)
    target_link_libraries(${OUTBINNAME} PRIVATE ws2_32.lib)
endif()

include_directories(../)
add_definitions(-DSHADERAPIDX9 -DSHADER_DLL_EXPORT -DPROTECTED_THINGS_ENABLE -Dstrncpy=use_Q_strncpy_instead -D_snprintf=use_Q_snprintf_instead)

if( GL )
    add_definitions(-DGL_GLEXT_PROTOTYPES -DDX_TO_GL_ABSTRACTION)
endif()

#$SystemFrameworks				"Carbon;OpenGL;Quartz;Cocoa;IOKit"
#$SystemLibraries				"iconv" [$OSXALL]
#$AdditionalDependencies			"$BASE libgcm_sys_stub.a libsysutil_stub.a libresc_stub.a libspurs_stub.a" [$PS3]


#Source files
target_sources(${OUTBINNAME} PRIVATE "colorformatdx8.cpp")
target_sources(${OUTBINNAME} PRIVATE "d3d_async.cpp")
if( NOT OSXALL )
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/filesystem_helpers.cpp")
endif()
target_sources(${OUTBINNAME} PRIVATE "hardwareconfig.cpp")
target_sources(${OUTBINNAME} PRIVATE "meshbase.cpp")
target_sources(${OUTBINNAME} PRIVATE "meshdx8.cpp")
if( WINDOWS AND NOT GL )
    target_sources(${OUTBINNAME} PRIVATE "recording.cpp")
    target_sources(${OUTBINNAME} PRIVATE "wmi.cpp")
endif()
target_sources(${OUTBINNAME} PRIVATE "shaderapidx8.cpp")
target_sources(${OUTBINNAME} PRIVATE "shaderdevicebase.cpp")
target_sources(${OUTBINNAME} PRIVATE "shaderapibase.cpp")
target_sources(${OUTBINNAME} PRIVATE "shaderdevicedx8.cpp")
target_sources(${OUTBINNAME} PRIVATE "shadershadowdx8.cpp")
target_sources(${OUTBINNAME} PRIVATE "texturedx8.cpp")
target_sources(${OUTBINNAME} PRIVATE "TransitionTable.cpp")
target_sources(${OUTBINNAME} PRIVATE "cvballoctracker.cpp")
target_sources(${OUTBINNAME} PRIVATE "vertexdecl.cpp")
target_sources(${OUTBINNAME} PRIVATE "vertexshaderdx8.cpp")
#$File	"$SHADERAPIDX9DIR\textureheap.cpp"							[$X360]
#$File	"$SRCDIR\common\ps3\vjobutils.cpp"							[$PS3]
#$File	"$SRCDIR\common\ps3\spu_job_shared.cpp"						[$PS3]
if( NOT WINDOWS )
    target_sources(${OUTBINNAME} PRIVATE "winutils.cpp")
endif()



#Linker
if( WINDOWS OR POSIX )
    target_link_libraries(${OUTBINNAME} videocfg_client)
endif()
if( GL )
    target_link_libraries(${OUTBINNAME} libtogl_client)
endif()

target_link_libraries(${OUTBINNAME} bitmap_client tier2_client vtf_client bzip2_client )

if( WINDOWS )
    if( NOT GL )
    #$File	"$SRCDIR\dx9sdk\lib$PLATSUBDIR\d3d9.lib" [$WINDOWS && !$GL]
    #$File	"$SRCDIR\dx9sdk\lib$PLATSUBDIR\d3dx9.lib" [$WINDOWS && !$GL]
    endif()
    #$File	"$SRCDIR\thirdparty\nvidia\nvapi\lib$PLATSUBDIR\nvapi.lib" [$WINDOWS]
endif()

