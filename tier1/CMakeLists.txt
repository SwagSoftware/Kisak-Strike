#//-----------------------------------------------------------------------------
#//	TIER1.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.17)
project(tier1_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "tier1_client")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

add_definitions(-DTIER1_STATIC_LIB)

if( WINDOWS )
    #		$AdditionalDependencies			"$BASE Rpcrt4.lib"
endif()

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE "-fPIC")
endif()

#Sources
target_sources(${OUTLIBNAME} PRIVATE "appinstance.cpp")
target_sources(${OUTLIBNAME} PRIVATE "bitbuf.cpp")
target_sources(${OUTLIBNAME} PRIVATE "newbitbuf.cpp")
target_sources(${OUTLIBNAME} PRIVATE "byteswap.cpp")
target_sources(${OUTLIBNAME} PRIVATE "characterset.cpp")
target_sources(${OUTLIBNAME} PRIVATE "checksum_crc.cpp")
target_sources(${OUTLIBNAME} PRIVATE "checksum_md5.cpp")
target_sources(${OUTLIBNAME} PRIVATE "checksum_sha1.cpp")
target_sources(${OUTLIBNAME} PRIVATE "circularbuffer.cpp")
target_sources(${OUTLIBNAME} PRIVATE "commandbuffer.cpp")
target_sources(${OUTLIBNAME} PRIVATE "convar.cpp")
target_sources(${OUTLIBNAME} PRIVATE "datamanager.cpp")
target_sources(${OUTLIBNAME} PRIVATE "diff.cpp")
target_sources(${OUTLIBNAME} PRIVATE "exprevaluator.cpp")
target_sources(${OUTLIBNAME} PRIVATE "generichash.cpp")
target_sources(${OUTLIBNAME} PRIVATE "interface.cpp")
target_sources(${OUTLIBNAME} PRIVATE "keyvalues.cpp")
target_sources(${OUTLIBNAME} PRIVATE "keyvaluesjson.cpp")
target_sources(${OUTLIBNAME} PRIVATE "kvpacker.cpp")
target_sources(${OUTLIBNAME} PRIVATE "lzmaDecoder.cpp")
target_sources(${OUTLIBNAME} PRIVATE "lzss.cpp")
target_sources(${OUTLIBNAME} PRIVATE "mempool.cpp")
target_sources(${OUTLIBNAME} PRIVATE "memstack.cpp")
target_sources(${OUTLIBNAME} PRIVATE "NetAdr.cpp")
target_sources(${OUTLIBNAME} PRIVATE "splitstring.cpp")

if( WINDOWS OR X360 )
    target_sources(${OUTLIBNAME} PRIVATE "processor_detect.cpp")
    #					$EnableC++Exceptions	"Yes (/EHsc)" [!$X360]
endif()

if( POSIX )
    target_sources(${OUTLIBNAME} PRIVATE "processor_detect_linux.cpp")
endif()
if( LINUXALL OR PS3 )
    target_sources(${OUTLIBNAME} PRIVATE "qsort_s.cpp")
endif()

target_sources(${OUTLIBNAME} PRIVATE "rangecheckedvar.cpp")
target_sources(${OUTLIBNAME} PRIVATE "stringpool.cpp")
target_sources(${OUTLIBNAME} PRIVATE "strtools.cpp")
target_sources(${OUTLIBNAME} PRIVATE "strtools_unicode.cpp")
target_sources(${OUTLIBNAME} PRIVATE "tier1.cpp")
target_sources(${OUTLIBNAME} PRIVATE "tier1_logging.cpp")
target_sources(${OUTLIBNAME} PRIVATE "timeutils.cpp")
target_sources(${OUTLIBNAME} PRIVATE "uniqueid.cpp")
target_sources(${OUTLIBNAME} PRIVATE "utlbuffer.cpp")
target_sources(${OUTLIBNAME} PRIVATE "utlbufferutil.cpp")
target_sources(${OUTLIBNAME} PRIVATE "utlsoacontainer.cpp")
target_sources(${OUTLIBNAME} PRIVATE "utlstring.cpp")
target_sources(${OUTLIBNAME} PRIVATE "utlstringtoken.cpp")
target_sources(${OUTLIBNAME} PRIVATE "utlsymbol.cpp")
target_sources(${OUTLIBNAME} PRIVATE "miniprofiler_hash.cpp")
target_sources(${OUTLIBNAME} PRIVATE "sparsematrix.cpp")
target_sources(${OUTLIBNAME} PRIVATE "memoverride_dummy.cpp")

target_sources(${OUTLIBNAME} PRIVATE "pathmatch.cpp")