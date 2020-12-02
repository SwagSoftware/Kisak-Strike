#include "rkhud_radar.h"

#include "cbase.h"
#include "hud_macros.h"
#include "c_cs_player.h"

#include <tier0/valve_minmax_off.h> // included to fix an error with min/max and rocketui
#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"

DECLARE_HUDELEMENT( RkHudRadar );
DECLARE_HUD_MESSAGE( RkHudRadar, ProcessSpottedEntityUpdate );

ConVar rocket_hud_radar_info_linger_time( "rocket_hud_radar_info_linger_time", "3", 0, "How long in seconds does the data stay visible after an update" );
ConVar rocket_hud_radar_scale( "rocket_hud_radar_scale", "0.15", 0, "scale for radar" );

static void RadarSizeChanged( IConVar *pConvar, const char *szOldValue, float fOldValue )
{
    RkHudRadar *pRadar = GET_HUDELEMENT( RkHudRadar );
    if( !pRadar )
    {
        Warning( "Couldn't grab hud radar to update size!\n" );
        return;
    }
    pRadar->UpdateRadarSize();
}
ConVar rocket_hud_radar_height( "rocket_hud_radar_height", "400", 0, "height in pixels for the radar", RadarSizeChanged );
ConVar rocket_hud_radar_width( "rocket_hud_radar_width", "400", 0, "width in pixels for the radar", RadarSizeChanged );


bool RkHudRadar::MsgFunc_ProcessSpottedEntityUpdate(const CCSUsrMsg_ProcessSpottedEntityUpdate &msg)
{
    if( msg.new_update() )
    {
        // Clear everything.
        //for( int i = 0; i < MAX_PLAYERS; i++ )
        //    m_spottedPlayers[i].timelastSpotted = 0.0f;
    }

    for( int i = 0; i < msg.entity_updates_size(); i++ )
    {
        const auto &update = msg.entity_updates(i);

        int entID = update.entity_idx();
        // make sure this is a valid id for any type of entity.
        if( entID < 1 || entID >= MAX_EDICTS )
            continue;

        // these are sent from the server as /4
        int x = update.origin_x() * 4;
        int y = update.origin_y() * 4;
        int z = update.origin_z() * 4;
        int yaw = update.angle_y();

        const char *szEntClassName;
        int classID = update.class_id();
        for ( ClientClass *pCur = g_pClientClassHead; pCur; pCur = pCur->m_pNext )
        {
            if( pCur->m_ClassID == classID )
            {
                szEntClassName = pCur->GetName();
                break;
            }
        }

        if( !szEntClassName )
        {
            Warning( "Unknown entity class received in ProcessSpottedEntityUpdate.\n" );
        }

        // Clients are unaware of the defuser class type, so we need to flag defuse entities manually
        if( update.defuser() )
        {
            // TODO: this is the defuser!
        }
        else if( !V_strcmp( "CCSPlayer", szEntClassName ) )
        {
            // out of bounds!
            if( entID < 1 || entID > MAX_PLAYERS )
                return true;

            // subtract 1 to convert 1-64 to 0-63
            SpottedInfo &playerInfo = m_spottedPlayers[ (entID-1) ];
            playerInfo.entId = update.entity_idx();
            playerInfo.originX = x;
            playerInfo.originY = y;
            playerInfo.originZ = z;
            playerInfo.angleYaw = yaw;
            if( update.has_player_has_defuser() )
            {
                playerInfo.playerWithDefuser = true;
                // TODO: set defuser pos
            }
            if( update.player_has_c4() )
            {
                playerInfo.playerWithC4 = true;
                // TODO: set bomb pos
            }
            playerInfo.timelastSpotted = gpGlobals->curtime;
        }
        else if( !V_strcmp( "CC4", szEntClassName ) || !V_strcmp( "CPlantedC4", szEntClassName ) )
        {
            // TODO: set bomb pos
        }
        else if( !V_strcmp( "CHostage", szEntClassName ) )
        {
            // TODO: set hostage pos
        }
    }

    return true;
}

// https://www.unknowncheats.me/forum/general-programming-and-reversing/135529-implement-simple-radar.html
static inline void RotatePoint( float x, float y, float centerX, float centerY, float angle, float *outX, float *outY )
{
    angle = DEG2RAD( angle );

    float cosTheta = cosf( angle );
    float sinTheta = sinf( angle );

    *outX = (cosTheta * ( x - centerX )) - (sinTheta * ( y - centerY ));
    *outY = (sinTheta * ( x - centerX )) + (cosTheta * ( y - centerY ));
    *outX += centerX;
    *outY += centerY;
}

void RkHudRadar::UpdateRadarSize()
{
    if( !m_pInstance )
        return;

    // Bounds of radar
    m_elemBody->SetProperty("width", Rml::String(rocket_hud_radar_width.GetString()) + "px");
    m_elemBody->SetProperty("height", Rml::String(rocket_hud_radar_height.GetString()) + "px");
    m_radarWidth = rocket_hud_radar_width.GetFloat();
    m_radarHeight = rocket_hud_radar_height.GetFloat();
    m_radarX = m_elemBody->GetAbsoluteLeft();
    m_radarY = m_elemBody->GetAbsoluteTop();
    // these are children so we dont need the x+ or y+
    m_radarCenterX = ( m_radarWidth / 2.0f );
    m_radarCenterY = ( m_radarHeight / 2.0f );
}

void RkHudRadar::UpdateRadarFrame()
{
    C_CSPlayer *activePlayer = C_CSPlayer::GetLocalCSPlayer();
    if( !activePlayer )
        return;

    // observing someone? switch to that player.
    if( activePlayer->IsObserver() && (activePlayer->GetObserverMode() == OBS_MODE_IN_EYE || activePlayer->GetObserverMode() == OBS_MODE_CHASE) )
        activePlayer = ToCSPlayer(activePlayer->GetObserverTarget());

    float currTime = gpGlobals->curtime;

    // The Radar packets are designed to supplement an existing regular radar.
    // We will do a regular radar, but with the dormant check, see if we have some recent maphack data from server.
    for( int i = 1; i <= MAX_PLAYERS; i++ )
    {
        if( i == engine->GetLocalPlayer() )
            continue;

        // array index for the player elements
        int index = i - 1;
        CBasePlayer *player = UTIL_PlayerByIndex( i );
        const SpottedInfo &playerInfo = m_spottedPlayers[index];

        if( !player || !player->IsAlive() )
        {
            m_playerElements[index]->SetProperty("opacity", "0");
            continue;
        }

        // if dormant and we dont have any recent information from the server
        if( player->IsDormant() && ( ( currTime - playerInfo.timelastSpotted ) > rocket_hud_radar_info_linger_time.GetFloat() ) )
        {
            m_playerElements[index]->SetProperty("opacity", "0");
            continue;
        }

        // At this point we either have a visible player or recent server radar data
        m_playerElements[index]->SetProperty("opacity", "1");

        float originDiffX;
        float originDiffY;

        if( player->IsDormant() )
        {
            originDiffX = activePlayer->GetAbsOrigin().x - playerInfo.originX;
            originDiffY = activePlayer->GetAbsOrigin().y - playerInfo.originY;
        }
        else
        {
           originDiffX = activePlayer->GetAbsOrigin().x - player->GetAbsOrigin().x;
           originDiffY = activePlayer->GetAbsOrigin().y - player->GetAbsOrigin().y;
        }

        originDiffX *= rocket_hud_radar_scale.GetFloat();
        originDiffY *= rocket_hud_radar_scale.GetFloat();
        originDiffX *= -1; // x goes other way

        // add the center of the radar.
        originDiffX += m_radarCenterX;
        originDiffY += m_radarCenterY;

        float rotatedX;
        float rotatedY;
        RotatePoint( originDiffX, originDiffY, m_radarCenterX, m_radarCenterY, activePlayer->EyeAngles().y - 90.0f, &rotatedX, &rotatedY );

        // these guys are off the radar. Go ahead and hide them.
        if( rotatedX > m_radarWidth || rotatedX < 0 || rotatedY > m_radarHeight || rotatedY < 0 )
        {
            m_playerElements[index]->SetProperty("opacity", "0");
            continue;
        }

        m_playerElements[index]->SetProperty("left", std::to_string(int(rotatedX)) + "px");
        m_playerElements[index]->SetProperty("top", std::to_string(int(rotatedY)) + "px");
    }
}

static void UnloadRkRadar()
{
    RkHudRadar *pRadar = GET_HUDELEMENT( RkHudRadar );
    if( !pRadar )
    {
        Warning( "Couldn't grab hud radar to unload!\n" );
        return;
    }

    if( !pRadar->m_pInstance )
        return;

    pRadar->m_pInstance->Close();
    pRadar->m_pInstance = nullptr;
    pRadar->m_bVisible = false;
}
static void LoadRkRadar()
{
    RkHudRadar *pRadar = GET_HUDELEMENT( RkHudRadar );
    if( !pRadar )
    {
        Warning( "Couldn't grab hud radar to load!\n" );
        return;
    }

    if( pRadar->m_pInstance )
    {
        Warning( "RkRadar already loaded! call unload first!\n" );
        return;
    }

    pRadar->m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_HUD, "hud_radar.rml", &LoadRkRadar, UnloadRkRadar );
    if( !pRadar->m_pInstance )
    {
        Error( "Couldn't create hud_radar document!\n" );
        /* Exit */
    }

    pRadar->m_elemBody = pRadar->m_pInstance->GetElementById( "body" );
    if( !pRadar->m_elemBody )
    {
        Error("Couldn't find required element id 'body' in hud_radar\n" );
        /* Exit */
    }

    pRadar->m_elemContainer = pRadar->m_pInstance->GetElementById( "container" );
    if( !pRadar->m_elemContainer )
    {
        Error("Couldn't find required element id 'container' in hud_radar\n" );
        /* Exit */
    }

    // Create all the player elements. These will be moved around, hidden and shown as the radar updates
    for( int i = 0; i < MAX_PLAYERS; i++ )
    {
        Rml::ElementPtr ptr = pRadar->m_pInstance->CreateElement("div");
        pRadar->m_playerElements[i] = ptr.get();
        if( !pRadar->m_playerElements[i] )
        {
            Error("Couldn't create player element(%d) for radar!", i);
            /* Exit */
        }
        pRadar->m_playerElements[i]->SetClass("player", true);
        pRadar->m_playerElements[i]->SetProperty("opacity", "0");

        pRadar->m_elemContainer->AppendChild( std::move(ptr) );
    }

    pRadar->UpdateRadarSize();
}

RkHudRadar::RkHudRadar(const char *value) : CHudElement( value ),
                                            m_bVisible( false ),
                                            m_pInstance( nullptr )
{
    SetHiddenBits( /* HIDEHUD_MISCSTATUS */ 0 );
}

RkHudRadar::~RkHudRadar() noexcept
{
    UnloadRkRadar();
}

void RkHudRadar::LevelInit()
{
    LoadRkRadar();

    HOOK_HUD_MESSAGE( RkHudRadar, ProcessSpottedEntityUpdate );
}

void RkHudRadar::LevelShutdown()
{
    UnloadRkRadar();
}

void RkHudRadar::ShowPanel(bool bShow, bool force)
{
    if( !m_pInstance )
        return;

    if( bShow )
    {
        if( !m_bVisible )
        {
            m_pInstance->Show();
        }
        UpdateRadarFrame();
    }
    else
    {
        if( m_bVisible )
        {
            m_pInstance->Hide();
        }
    }

    m_bVisible = bShow;
}

void RkHudRadar::SetActive(bool bActive)
{
    ShowPanel( bActive, false );
    CHudElement::SetActive( bActive );
}

bool RkHudRadar::ShouldDraw()
{
    C_CSPlayer *localPlayer = C_CSPlayer::GetLocalCSPlayer();

    return localPlayer &&
           cl_drawhud.GetBool() &&
           ( localPlayer->IsAlive() || ( localPlayer->IsObserver() && localPlayer->GetObserverMode() == OBS_MODE_IN_EYE || localPlayer->GetObserverMode() == OBS_MODE_CHASE ) ) &&
           CHudElement::ShouldDraw();
}