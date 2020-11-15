#//-----------------------------------------------------------------------------
#//	GCSDK.VPC
#//
#//	Project Script for the Game Coordinator SDK
#//-----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(kisak_gcsdk_client)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(OUTLIBNAME "kisak_gcsdk_client")
set(GENERATED_PROTO_DIR "${SRCDIR}/gcsdk/generated_proto")

include(${CMAKE_MODULE_PATH}/detect_platform.cmake)
include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)
include(${CMAKE_MODULE_PATH}/protobuf_builder.cmake)

TargetBuildAndAddProto( ${OUTLIBNAME} ${SRCDIR}/gcsdk/steammessages.proto ${GENERATED_PROTO_DIR} )
TargetBuildAndAddProto( ${OUTLIBNAME} ${SRCDIR}/gcsdk/gcsdk_gcmessages.proto ${GENERATED_PROTO_DIR} )
TargetBuildAndAddProto( ${OUTLIBNAME} ${SRCDIR}/gcsdk/gcsystemmsgs.proto ${GENERATED_PROTO_DIR} )

include_directories(${SRCDIR})
include_directories(${SRCDIR}/gcsdk/steamextra)
include_directories(${SRCDIR}/public)
include_directories(${SRCDIR}/public/gcsdk)
include_directories(${SRCDIR}/thirdparty/JSON_parser)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE -fPIC -fno-semantic-interposition)
endif()

target_sources(${OUTLIBNAME} PRIVATE "gcclient.cpp")
target_sources(${OUTLIBNAME} PRIVATE "gcconstants.cpp")
target_sources(${OUTLIBNAME} PRIVATE "gclogger.cpp")
target_sources(${OUTLIBNAME} PRIVATE "gcmsg.cpp")
target_sources(${OUTLIBNAME} PRIVATE "job.cpp")
target_sources(${OUTLIBNAME} PRIVATE "jobmgr.cpp")
target_sources(${OUTLIBNAME} PRIVATE "jobtime.cpp")
target_sources(${OUTLIBNAME} PRIVATE "messagelist.cpp")
target_sources(${OUTLIBNAME} PRIVATE "msgprotobuf.cpp")
target_sources(${OUTLIBNAME} PRIVATE "netpacket.cpp")
target_sources(${OUTLIBNAME} PRIVATE "netpacketpool.cpp")
target_sources(${OUTLIBNAME} PRIVATE "protobufsharedobject.cpp")
target_sources(${OUTLIBNAME} PRIVATE "sharedobject.cpp")
target_sources(${OUTLIBNAME} PRIVATE "sharedobjectcache.cpp")
target_sources(${OUTLIBNAME} PRIVATE "gcclient_sharedobjectcache.cpp")
target_sources(${OUTLIBNAME} PRIVATE "steamextra/misc.cpp")
target_sources(${OUTLIBNAME} PRIVATE "steamextra/rtime.cpp")
target_sources(${OUTLIBNAME} PRIVATE "steamextra/steamid.cpp")

target_sources(${OUTLIBNAME} PRIVATE "steamextra/tier1/hashglobals.cpp")
target_sources(${OUTLIBNAME} PRIVATE "steamextra/tier1/tsmempool.cpp")
target_sources(${OUTLIBNAME} PRIVATE "steamextra/tier1/tsmultimempool.cpp")
target_sources(${OUTLIBNAME} PRIVATE "workthreadpool.cpp")
target_sources(${OUTLIBNAME} PRIVATE "webapi_response.cpp")
target_sources(${OUTLIBNAME} PRIVATE "${SRCDIR}/thirdparty/JSON_parser/JSON_parser.c")