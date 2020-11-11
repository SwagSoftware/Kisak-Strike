#//-----------------------------------------------------------------------------
#//	SERVER_CSTRIKE15.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

#There are other projects in this folder, but this cmake is focused towards csgo a.k.a. cstrike15

cmake_minimum_required(VERSION 3.16)
project(Server_CSGO)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(CMAKE_MODULE_PATH ${SRCDIR}/cmake)
set(GAMENAME "csgo")
set(CSGO "1")
set(SRVSRCDIR ${CMAKE_CURRENT_SOURCE_DIR})

set(NOSTINKYLINKIES "1")

add_definitions(-DBOTS -DCSTRIKE_DLL -DUSE_ECONOMY_FEATURES)

include( ./server_base.cmake )
include( ./server_econ_base.cmake )

if( LINUXALL )
    #lwss- add Wno-narrowing to fix compilation
    target_compile_options(${OUTBINNAME} PRIVATE "-Wno-narrowing")
    #lwss end
endif()

include_directories(${SRCDIR}/game/shared/cstrike15)
include_directories(${SRCDIR}/game/shared/cstrike15/bot)
include_directories(${SRVSRCDIR})
include_directories(${SRVSRCDIR}/cstrike15)
include_directories(${SRVSRCDIR}/cstrike15/bot)
include_directories(${SRVSRCDIR}/cstrike15/bot/states)
include_directories(${SRVSRCDIR}/cstrike15/control)
include_directories(${SRVSRCDIR}/cstrike15/hostage)

#cstrike15 protos
TargetBuildAndAddProto( ${OUTBINNAME} ${SRCDIR}/game/shared/cstrike15/cstrike15_gcmessages.proto "${GENERATED_PROTO_DIR}")
TargetBuildAndAddProto( ${OUTBINNAME} ${SRCDIR}/game/shared/cstrike15/cstrike15_usermessages.proto "${GENERATED_PROTO_DIR}")

target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/hl2/Func_Monitor.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/h_cycler.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/predicted_viewmodel.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/team_objectiveresource.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/team_spawnpoint.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/server_log_http_dispatcher.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/teamplayroundbased_gamerules.cpp")

#$Folder	"Economy" [!$X360 && !$PS3]
#{
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cstrike15_item_inventory.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cstrike15_item_system.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cstrike15_item_schema.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_econ_item_string_table.cpp")
#}

#$Folder	"CounterStrike DLL"
#{
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/achievements_cs.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/basecsgrenade_projectile.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_ammodef.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_autobuy.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_client.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_entity_spotting.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_eventlog.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_gameinterface.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_gamemovement.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_gamerules.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_gamestats.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_gamestats_shared.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/steamworks_gamestats_server.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_hltvdirector.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_player.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_player_resource.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_player_shared.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_playeranimstate.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/csgo_playeranimstate.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_playermove.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_pushentity.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/dedicated_server_ugc_manager.cpp")#[!$X360 && !$PS3])
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_shareddefs.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_team.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_vehicle_jeep.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_weapon_parse.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/flashbang_projectile.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/func_bomb_target.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/func_buy_zone.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/func_hostage_rescue.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/func_no_defuse.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/funfact_cs.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/funfactmgr_cs.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/fx_cs_shared.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/hegrenade_projectile.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/info_view_parameters.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/item_ammo.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/item_assaultsuit.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/item_defuser.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/item_kevlar.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/item_nvgs.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/mapinfo.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/point_hiding_spot.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/point_surroundtest.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/smokegrenade_projectile.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/te_radioicon.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/te_shotgun_shot.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/player_pickup.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/molotov_projectile.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/decoy_projectile.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/viewmodel_arm_config.inc")

    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/sensorgrenade_projectile.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_voteissues.cpp")

    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/gametypes.cpp")

    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/XZip.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/XUnzip.cpp")

    #$Folder	"Weapon"
    #{
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_basecsgrenade.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_c4.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_csbase.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_csbasegun.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_deagle.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_elite.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_flashbang.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_hegrenade.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_ironsightcontroller.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_knife.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_m3.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_sg552.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_smokegrenade.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_xm1014.cpp")

        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_sawedoff.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_taser.cpp")

        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_molotov.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_decoy.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_sensorgrenade.cpp")

        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_nova.cpp")

        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_baseitem.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/item_healthshot.cpp")
    #}

    #$Folder "Effects"
    #{
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/Effects/inferno.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/Effects/chicken.cpp")

        #$Folder "Shared"
        #{
            target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/infernoshared.cpp")
        #}
    #}

    #$Folder	"Bot"
    #{
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_chatter.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_chatter_coop.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_event.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_event_bomb.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_event_player.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_event_weapon.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_init.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_listen.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_manager.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_nav.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_pathfind.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_radio.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_statemachine.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_update.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_vision.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_weapon.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_bot_weapon_id.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/cs_gamestate.cpp")

        #$Folder	"States"
        #{
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_attack.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_buy.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_defuse_bomb.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_pickup_hostage.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_escape_from_bomb.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_fetch_bomb.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_follow.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_hide.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_hunt.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_idle.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_idle_coop.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_investigate_noise.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_move_to.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_open_door.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_plant_bomb.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_use_entity.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/bot/states/cs_bot_escape_from_flames.cpp")
        #}

        #$Folder	"Shared"
        #{
            target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/bot/bot.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/bot/bot_hide.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/bot/bot_manager.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/bot/bot_profile.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/bot/bot_util.cpp")
            target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/common/platforminputdevice.cpp")

            target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/bot/shared_util.cpp")
        #}
    #}

    #$Folder	"Hostage"
    #{
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/hostage/cs_simple_hostage.cpp")
    #}

    #$Folder	"Navigation Mesh"
    #{
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_nav_area.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_nav_generate.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_nav_mesh.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRVSRCDIR}/cstrike15/cs_nav_path.cpp")
    #}
#}

