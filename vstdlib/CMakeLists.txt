cmake_minimum_required(VERSION 3.17)
project(vstdlib_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "vstdlib_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

add_definitions(-DVSTDLIB_DLL_EXPORT)

if( LINUXALL )
    target_compile_options(${OUTBINNAME} PRIVATE -U_FORTIFY_SOURCE -fno-stack-protector -fPIC)
endif()

target_sources(${OUTBINNAME} PRIVATE "concommandhash.h")
if( NOT X360 )
    target_sources(${OUTBINNAME} PRIVATE "coroutine.cpp")
endif()
target_sources(${OUTBINNAME} PRIVATE "cvar.cpp")
target_sources(${OUTBINNAME} PRIVATE "jobthread.cpp")
target_sources(${OUTBINNAME} PRIVATE "keyvaluessystem.cpp")
if( WINDOWS )
    #$File	"processutils.cpp"			[$WINDOWS]
endif()
target_sources(${OUTBINNAME} PRIVATE "random.cpp")
target_sources(${OUTBINNAME} PRIVATE "vcover.cpp")
target_sources(${OUTBINNAME} PRIVATE "eventsystem.cpp")
if( PS3 )
    #$File	"prx.cpp"					[$PS3]
    #$File	"prxexport.cpp"		[$PS3]
endif()
target_sources(${OUTBINNAME} PRIVATE "vstrtools.cpp")
target_sources(${OUTBINNAME} PRIVATE "vstdlib_strtools.cpp")