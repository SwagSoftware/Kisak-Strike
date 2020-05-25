set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
include(${CMAKE_MODULE_PATH}/common_functions.cmake)

MacroRequired(SRCDIR)
MacroRequired(OUTLIBNAME)

include(${CMAKE_MODULE_PATH}/source_lib_base.cmake)

target_compile_definitions(${OUTLIBNAME} PRIVATE -DNO_STRING_T -DVECTOR -DVERSION_SAFE_STEAM_API_INTERFACES -DPROTECTED_THINGS_ENABLE -DNO_STEAM_GAMECOORDINATOR)
target_include_directories(${OUTLIBNAME} PRIVATE ${SRCDIR}/thirdparty/protobuf-2.5.0/src)

if( LINUXALL )
    target_compile_options(${OUTLIBNAME} PRIVATE "-fPIC")
endif()

#$Folder "Matchmaking"
#{
    target_sources(${OUTLIBNAME} PRIVATE "matchmakingqos.cpp")
    target_sources(${OUTLIBNAME} PRIVATE "mm_events.cpp")
    target_sources(${OUTLIBNAME} PRIVATE "mm_extensions.cpp")
    target_sources(${OUTLIBNAME} PRIVATE "mm_framework.cpp")
    target_sources(${OUTLIBNAME} PRIVATE "mm_netmsgcontroller.cpp")
    if( MM_DS MATCHES "0" )
        target_sources(${OUTLIBNAME} PRIVATE "mm_session.cpp")
    endif()
    target_sources(${OUTLIBNAME} PRIVATE "mm_voice.cpp")
#}

#$Folder "Sessions"
#{
    if( MM_DS MATCHES "0" )
        target_sources(${OUTLIBNAME} PRIVATE "ds_searcher.cpp")
        target_sources(${OUTLIBNAME} PRIVATE "match_searcher.cpp")
    endif()

    target_sources(${OUTLIBNAME} PRIVATE "mm_netmgr.cpp")

    if( MM_DS MATCHES "0" )
        target_sources(${OUTLIBNAME} PRIVATE "mm_session_offline_custom.cpp")
        target_sources(${OUTLIBNAME} PRIVATE "mm_session_online_client.cpp")
        target_sources(${OUTLIBNAME} PRIVATE "mm_session_online_host.cpp")
        target_sources(${OUTLIBNAME} PRIVATE "mm_session_online_search.cpp")
        target_sources(${OUTLIBNAME} PRIVATE "mm_session_online_teamsearch.cpp")
        target_sources(${OUTLIBNAME} PRIVATE "sys_session.cpp")
    endif()
#}

#$Folder "Platform - Steam"
#{
    target_sources(${OUTLIBNAME} PRIVATE "steam_apihook.cpp")
    target_sources(${OUTLIBNAME} PRIVATE "steam_apihook.h")
    target_sources(${OUTLIBNAME} PRIVATE "steam_datacenterjobs.h")
    target_sources(${OUTLIBNAME} PRIVATE "steam_datacenterjobs.cpp") #[!$X360]
    if( NOT DEDICATED )
        target_sources(${OUTLIBNAME} PRIVATE "steam_lobbyapi.cpp") #[!$NO_STEAM && !$DEDICATED && !$X360]
    endif()
    target_sources(${OUTLIBNAME} PRIVATE "steam_lobbyapi.h")
#}

#$Folder "Platform - Xbox 360"
#{
    #$File	"x360_lobbyapi.cpp" [$X360]
    #$File	"x360_lobbyapi.h"
    #$File	"x360_netmgr.cpp" [$X360]
    #$File	"x360_netmgr.h"
    #TODO: can we remove this??
    target_sources(${OUTLIBNAME} PRIVATE "x360_xlsp_cmd.cpp")
    #$File	"x360_xlsp_cmd.h"
#}

#$Folder "Systems"
#{
    target_sources(${OUTLIBNAME} PRIVATE "datacenter.cpp")
    target_sources(${OUTLIBNAME} PRIVATE "mm_dlc.cpp")
    if( MM_DS MATCHES "0" )
        target_sources(${OUTLIBNAME} PRIVATE "leaderboards.cpp")
    endif()
    target_sources(${OUTLIBNAME} PRIVATE "matchsystem.cpp")
    if( MM_DS MATCHES "0" )
        target_sources(${OUTLIBNAME} PRIVATE "player.cpp")
    endif()
    target_sources(${OUTLIBNAME} PRIVATE "playermanager.cpp")
    if( MM_DS MATCHES "0" )
        target_sources(${OUTLIBNAME} PRIVATE "searchmanager.cpp")
    endif()
    target_sources(${OUTLIBNAME} PRIVATE "servermanager.cpp")
    target_sources(${OUTLIBNAME} PRIVATE "playerrankingdata.cpp")
#}

#$Folder "Utils"
#{
    target_sources(${OUTLIBNAME} PRIVATE "extkeyvalues.cpp")
#}

#$Folder "Source Files"
#{
    target_sources(${OUTLIBNAME} PRIVATE "${SRCDIR}/public/filesystem_helpers.cpp")
    target_sources(${OUTLIBNAME} PRIVATE "main.cpp")
#}

#[!$X360 && !$PS3]
target_link_libraries(${OUTLIBNAME} ${LIBPUBLIC}/gcsdk_client.a) #link to evil game coordinator lib
