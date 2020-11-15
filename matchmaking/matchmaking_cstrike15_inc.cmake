set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
include(${CMAKE_MODULE_PATH}/common_functions.cmake)

MacroRequired(SRCDIR)
MacroRequired(OUTBINNAME)

set(GAMENAME "csgo")
set(GAMESUBDIR "cstrike15")
set(GENERATED_PROTO_DIR "generated_proto")

set(OUTBINDIR "${SRCDIR}/../game/${GAMENAME}/bin")

include(${CMAKE_MODULE_PATH}/source_dll_base.cmake)

if( LINUXALL )
    target_compile_options(${OUTBINNAME} PRIVATE -fpic -fno-semantic-interposition)
endif()

#####matchmaking_inc.vpc#####
target_include_directories(${OUTBINNAME} PRIVATE ${SRCDIR}/gcsdk/steamextra)
target_include_directories(${OUTBINNAME} PRIVATE ${SRCDIR}/common)
target_include_directories(${OUTBINNAME} PRIVATE ${SRCDIR}/thirdparty/protobuf-2.5.0/src)
target_include_directories(${OUTBINNAME} PRIVATE ${SRCDIR}/common/xlast_${GAMENAME})
target_compile_definitions(${OUTBINNAME} PRIVATE -DNO_STRING_T -DVECTOR -DVERSION_SAFE_STEAM_API_INTERFACES -DPROTECTED_THINGS_ENABLE -DNO_STEAM_GAMECOORDINATOR)


#[!$X360 && !$PS3]
target_link_libraries(${OUTBINNAME} tier3_client libvstdlib_client)
#############################

#$Folder "cstrike15"
#{
    target_sources(${OUTBINNAME} PRIVATE "${GAMESUBDIR}/mm_title.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${GAMESUBDIR}/mm_title_gamesettingsmgr.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${GAMESUBDIR}/mm_title_main.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${GAMESUBDIR}/mm_title_richpresence.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${GAMESUBDIR}/mm_title_titledata.cpp")
#}

#$Folder "Shared"
#{
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/gametypes.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/common/platforminputdevice.cpp")
#}
