#//-----------------------------------------------------------------------------
#//	CLIENT_CSTRIKE15.VPC
#//
#//	Project Script
#//-----------------------------------------------------------------------------

#There are other projects in this folder, but this cmake is focused towards csgo a.k.a. cstrike15

cmake_minimum_required(VERSION 3.16)
project(Client_CSGO)
set(SRCDIR "${CMAKE_SOURCE_DIR}")
set(GAMENAME "csgo")
set(CSGO "1")
#set(DONT_DOWNGRADE_ABI "1")

#dont need to build this when building dedicated server
if( DEDICATED )
    return()
endif()

add_definitions(-DCSTRIKE_CLIENT_DLL -DCSTRIKE_DLL -DGAMEUI_EMBEDDED -DGAMEUI_EXPORTS -DVERSION_SAFE_STEAM_API_INTERFACES -DUSE_ECONOMY_FEATURES)
if( USE_SCALEFORM )
    add_definitions(-DINCLUDE_SCALEFORM)
elseif( USE_ROCKETUI )
    add_definitions(-DINCLUDE_ROCKETUI)
endif()
add_definitions(-DALLOW_TEXT_MODE=1)

include( ./client_base.cmake )
include( ./client_econ_base.cmake )

include_directories(${SRCDIR}/game/shared/cstrike15/control)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cstrike15)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cstrike15/VGUI)
include_directories(${SRCDIR}/game/shared/cstrike15)
include_directories(${SRCDIR}/game/shared)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cstrike15/gameui)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cstrike15/Scaleform)

#cstrike15 protos
TargetBuildAndAddProto( ${OUTBINNAME} ${SRCDIR}/game/shared/cstrike15/cstrike15_gcmessages.proto "${GENERATED_PROTO_DIR}")
TargetBuildAndAddProto( ${OUTBINNAME} ${SRCDIR}/game/shared/cstrike15/cstrike15_usermessages.proto "${GENERATED_PROTO_DIR}")
TargetBuildAndAddProto( ${OUTBINNAME} ${SRCDIR}/gcsdk/gcsdk_gcmessages.proto "${GENERATED_PROTO_DIR}")
TargetBuildAndAddProto( ${OUTBINNAME} ${SRCDIR}/gcsdk/gcsystemmsgs.proto "${GENERATED_PROTO_DIR}")

#force include
if( WINDOWS OR POSIX )
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/tier0/platform.h")
endif()

#instead we add csgo checks to client_base.cmake
#-$File	"${SRCDIR}/game/shared/weapon_parse_default.cpp"
#-$File	"c_mod_lesson_stubs.cpp"

#32bit time?


target_sources(${OUTBINNAME} PRIVATE "c_team_objectiveresource.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/bannedwords.cpp")
target_sources(${OUTBINNAME} PRIVATE "hud_voicestatus.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/predicted_viewmodel.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/player_pickup.cpp")

target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/XZip.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/XUnzip.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/tier1/fileio.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/utils/common/bsplib.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/filesystem_init.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/utils/common/filesystem_tools.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/utils/common/cmdlib.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/utils/common/scriplib.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/lumpfiles.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/public/zip_utils.cpp")

if( POSIX )
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/filesystem/linux_support.cpp")
endif()

if( NOT PS3 AND NOT X360 )
#$Folder	"Economy" [!$X360 && !$PS3]
#{
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cstrike15_item_inventory.cpp")

    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cstrike15_item_system.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cstrike15_item_schema.cpp")

    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_econ_item_string_table.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/econ/econ_game_account_client.cpp")
#}
endif()

#$Folder	"CounterStrike DLL"

target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/achievements_cs.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/basecsgrenade_projectile.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/buy_presets/buy_preset.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/buy_presets/buy_preset_debug.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/buy_presets/buy_preset_weapon_info.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/buy_presets/buy_presets.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_gameplay_hints.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_player_rank_mgr.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/c_cs_lesson.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/c_cs_hostage.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/c_cs_player.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/c_cs_playerresource.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/c_cs_team.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/c_csrootpanel.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/c_plantedc4.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/c_te_radioicon.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/c_te_shotgun_shot.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/clientmode_csnormal.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_ammodef.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_gamemovement.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_gamerules.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_gamestats_shared.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_in_main.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_player_shared.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_playeranimstate.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/csgo_playeranimstate.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_prediction.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_shareddefs.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_client_gamestats.cpp")
target_sources(${OUTBINNAME} PRIVATE "steamworks_gamestats_client.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_view_scene.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cs_custom_material_swap.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_custom_clothing_visualsdata_processor.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_custom_epidermis_visualsdata_processor.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_custom_weapon_visualsdata_processor.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_custom_embroider_visualsdata_processor.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_custom_texture_saver.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/client/model_combiner.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_weapon_parse.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/fx_cs_blood.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/fx_cs_impacts.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/fx_cs_knifeslash.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/fx_cs_muzzleflash.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/fx_cs_shared.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/fx_cs_weaponfx.cpp")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/fx_cs_christmaslights.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/bot/shared_util.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/viewmodel_arm_config.inc")
target_sources(${OUTBINNAME} PRIVATE "cstrike15/vgui_rootpanel_cs.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/ugc_workshop_manager.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/cs_workshop_manager.cpp")
target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/common/platforminputdevice.cpp")

target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/gametypes.cpp")

#$Folder "GameUI - Embedded"
#{
    #$Folder "Base GameUI"
    #{
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/backgroundmenubutton.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/basesavegamedialog.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/bitmapimagepanel.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/commandcheckbutton.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/contentcontroldialog.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/createmultiplayergamebotpage.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/createmultiplayergamedialog.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/createmultiplayergamegameplaypage.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/createmultiplayergameserverpage.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/customtabexplanationdialog.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/cvarnegatecheckbutton.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/cvarslider.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/cvartextentry.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/cvartogglecheckbutton.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/gameconsole.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/gameconsoledialog.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/gameui_interface.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/gameui_util.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/keytogglecheckbutton.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/labeledcommandcombobox.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/loadingtippanel.cpp")
    if( NOT POSIX )
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/logofile.cpp")
    endif()
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/modinfo.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/mousemessageforwardingpanel.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/multiplayeradvanceddialog.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/optionsdialog.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/optionssubaudio.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/optionssubdifficulty.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/optionssubgame.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/optionssubkeyboard.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/optionssubmouse.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/optionssubmultiplayer.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/optionssubportal.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/optionssubvideo.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/optionssubvoice.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/panellistpanel.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/playerlistdialog.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/rungameengine.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/scriptobject.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/sys_utils.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/textentrybox.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/urlbutton.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/vcontrolslistpanel.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/uigamedata.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/uigamedata_storage.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/uiavatarimage.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/vguisystemmoduleloader.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/ironsight_editor.cpp")
    #}

    #$Folder "External Stuff"
    #{
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/common/language.cpp")
    #}

    #$Folder "CSGO GameUI"
    #{
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/BasePanel.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/tracker/common/msgbuffer.cpp")
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/common/GameUI/ObjectList.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/bonusmapsdatabase.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/savegamebrowserdialog.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/cstrike15/cstrike15basepanel.cpp")
    #}

    #$Folder	"Dialogs"
    #{
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/benchmarkdialog.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/bonusmapsdialog.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/commentarydialog.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/commentaryexplanationdialog.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/loadgamedialog_xbox.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/loadgamedialog.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/newgamedialog.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/savegamedialog_xbox.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/savegamedialog.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/loadcommentarydialog.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/gameui/changegamedialog.cpp")
    #}
#}

#$Folder "HUD Elements"
#{
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_hud_chat.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_hud_scope.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/cs_hud_weaponselection.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/hud_flashbang.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/radio_status.cpp")
#}

#$Folder "Weapon"
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
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/molotov_projectile.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/decoy_projectile.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/smokegrenade_projectile.cpp")

    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/weapon_baseitem.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/item_healthshot.cpp")
    target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/sensorgrenade_projectile.cpp")
#}

#$Folder "Effects"
#{
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/Effects/clientinferno.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/Effects/c_chicken.cpp")

    #$Folder "Shared"
    #{
        target_sources(${OUTBINNAME} PRIVATE "${SRCDIR}/game/shared/cstrike15/infernoshared.cpp")
    #}
#}

if( USE_SCALEFORM )
    #$Folder "scaleform"
    #{
        #$Folder "HUD Elements"
        #{
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhud_autodisconnect.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhud_chat.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhudradar.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhud_radio.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhud_rosettaselector.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhudmoney.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhudreticle.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhudweaponpanel.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhud_uniquealerts.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhudhealtharmorpanel.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhud_deathnotice.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhud_teamcounter.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhudinfopanel.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhuddamageindicator.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfweaponselection.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhudwinpanel.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhudfreezepanel.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhudtrialtimer.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhudvotepanel.cpp")
            target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/HUD/sfhudcallvotepanel.cpp")
        #}

        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/createmainmenuscreen_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/html_base_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/html_control_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/blog_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/createstartscreen_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/createlegalanim_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/teammenu_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/motion_calibration_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/messagebox_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/overwatchresolution_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/mapoverview.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/options_video_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/options_audio_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/options_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/upsell_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/howtoplaydialog_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/medalstatsdialog_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/leaderboardsdialog_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/loadingscreen_scaleform.cpp")
        target_sources(${OUTBINNAME} PRIVATE "cstrike15/Scaleform/splitscreensignon.cpp")
    #}
elseif( USE_ROCKETUI )
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkmenu_main.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkhud_chat.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkhud_pausemenu.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkhud_teammenu.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkhud_loadingscreen.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkhud_infobar.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkhud_scoreboard.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkhud_roundtimer.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkhud_killfeed.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkhud_buymenu.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkhud_radar.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkpanel_options.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/RocketUI/rkpanel_popup.cpp")
endif()

#$Folder	"vgui"
#{
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/VGUI/counterstrikeviewport.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/vgui_c4panel.cpp")
    target_sources(${OUTBINNAME} PRIVATE "cstrike15/vgui_viewc4panel.cpp")
#}

#$Folder "hltvreplay"
#{
    target_sources(${OUTBINNAME} PRIVATE "hltvreplaysystem.cpp")
#}


target_link_libraries(${OUTBINNAME} meshutils_client)
if( WINDOWS OR POSIX )
    target_link_libraries(${OUTBINNAME} jpeglib_client)
    target_link_libraries(${OUTBINNAME} vpklib_client)
endif()
if( USE_ROCKETUI )
    #target_link_libraries(${OUTBINNAME} ${SRCDIR}/thirdparty/RmlUi/build/libRmlCore.so)
    target_link_libraries(${OUTBINNAME} RmlCore)
endif()
target_link_libraries(${OUTBINNAME} matsys_controls_client)
target_link_libraries(${OUTBINNAME} ${LIBPUBLIC}/libsteam_api.so) #proprietary steam api
target_link_libraries(${OUTBINNAME} kisak_gcsdk_client)
target_link_libraries(${OUTBINNAME} tier2_client tier3_client vgui_controls_client vtf_client bzip2_client libvstdlib_client dmxloader_client bitmap_client mathlib_client tier2_client )
#target_link_libraries(${OUTBINNAME} protobuf )

