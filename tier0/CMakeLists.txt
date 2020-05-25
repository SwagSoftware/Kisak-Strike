#//-----------------------------------------------------------------------------
#//	TIER0.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.17)
project(libtier0_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTBINNAME "libtier0_client")
set(OUTBINDIR ${SRCDIR}/../game/bin)

set(NOSTINKYLINKIES "1")
set(NOSKELETONBASE "1")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

if( WINDOWS OR X360 )
    add_definitions(-DTIER0_DLL_EXPORT -DCROSS_PLATFORM_VERSION=1 -DTHREAD_MUTEX_TRACING_ENABLED)
elseif( POSIX OR PS3 )
    add_definitions(-DTIER0_DLL_EXPORT -DCROSS_PLATFORM_VERSION=1 -DPOSIX)
endif()
if( NOFPO )
    add_definitions(-DTIER0_FPO_DISABLED)
endif()

if( LINUXALL )
    target_compile_options(${OUTBINNAME} PRIVATE -shared -fPIC)
endif()

target_sources(${OUTBINNAME} PRIVATE "../unitlib/unitlib.cpp")
target_sources(${OUTBINNAME} PRIVATE "assert_dialog.cpp")
target_sources(${OUTBINNAME} PRIVATE "commandline.cpp")
target_sources(${OUTBINNAME} PRIVATE "cpu.cpp")
target_sources(${OUTBINNAME} PRIVATE "cpumonitoring.cpp")
target_sources(${OUTBINNAME} PRIVATE "cputopology.cpp")
if( POSIX )
    target_sources(${OUTBINNAME} PRIVATE "cpu_posix.cpp")
endif()
target_sources(${OUTBINNAME} PRIVATE "dbg.cpp")
target_sources(${OUTBINNAME} PRIVATE "fasttimer.cpp")
target_sources(${OUTBINNAME} PRIVATE "logging.cpp")
target_sources(${OUTBINNAME} PRIVATE "mem.cpp")
target_sources(${OUTBINNAME} PRIVATE "mem_helpers.cpp")
target_sources(${OUTBINNAME} PRIVATE "memdbg.cpp")
target_sources(${OUTBINNAME} PRIVATE "memvirt.cpp")
target_sources(${OUTBINNAME} PRIVATE "memstd.cpp")
target_sources(${OUTBINNAME} PRIVATE "memvalidate.cpp")
target_sources(${OUTBINNAME} PRIVATE "dynfunction.cpp")
target_sources(${OUTBINNAME} PRIVATE "minidump.cpp")
target_sources(${OUTBINNAME} PRIVATE "tier0_strtools.cpp")
target_sources(${OUTBINNAME} PRIVATE "miniprofiler.cpp")
target_sources(${OUTBINNAME} PRIVATE "perfstats.cpp")
if( POSIX )
    target_sources(${OUTBINNAME} PRIVATE "platform_posix.cpp")
endif()
target_sources(${OUTBINNAME} PRIVATE "platform_independent.cpp")
target_sources(${OUTBINNAME} PRIVATE "platwindow.cpp")
if( POSIX OR PS3 )
    target_sources(${OUTBINNAME} PRIVATE "pme_posix.cpp")
endif()
target_sources(${OUTBINNAME} PRIVATE "progressbar.cpp")
target_sources(${OUTBINNAME} PRIVATE "security.cpp")
target_sources(${OUTBINNAME} PRIVATE "stackstats.cpp")
target_sources(${OUTBINNAME} PRIVATE "stacktools.cpp")
target_sources(${OUTBINNAME} PRIVATE "systeminformation.cpp")
target_sources(${OUTBINNAME} PRIVATE "threadtools.cpp")
target_sources(${OUTBINNAME} PRIVATE "tslist.cpp")
target_sources(${OUTBINNAME} PRIVATE "vatoms.cpp")
target_sources(${OUTBINNAME} PRIVATE "vprof.cpp")
target_sources(${OUTBINNAME} PRIVATE "vtuneinterface.cpp")
target_sources(${OUTBINNAME} PRIVATE "win32consoleio.cpp")
if( LINUXALL )
    #target_sources(${OUTBINNAME} PRIVATE "../tier1/pathmatch.cpp" )
endif()


#Linker
#-$ImpLib "$LIBPUBLIC\tier0"
#-$Lib	 "$LIBPUBLIC\tier1"
#-$implib "$LIBPUBLIC\vstdlib"
#-$Lib	 "$LIBPUBLIC\interfaces"
#
#$LibExternal	 "$SRCDIR\thirdparty\telemetry\lib\telemetry32.link" [$WIN32 && !$RAD_TELEMETRY_DISABLED]
#$LibExternal	 "$SRCDIR\thirdparty\telemetry\lib\telemetry64.link" [$WIN64 && !$RAD_TELEMETRY_DISABLED]
#$LibExternal	 "$SRCDIR/thirdparty/telemetry/lib/libtelemetryx86.link" [$LINUX32 && !$RAD_TELEMETRY_DISABLED]
#$LibExternal	 "$SRCDIR/thirdparty/telemetry/lib/libtelemetryx64.link" [$LINUX64 && !$RAD_TELEMETRY_DISABLED]

target_link_libraries(${OUTBINNAME} dl pthread)