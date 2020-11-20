#include "rkhud_roundtimer.h"

#include "cbase.h"
#include "hud_macros.h"
#include "c_cs_player.h"
#include "c_playerresource.h"

#include <tier0/valve_minmax_off.h> // included to fix an error with min/max and rocketui
#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"

DECLARE_HUDELEMENT( RkHudRoundTimer );

// Struct layout for data-binding model.
struct RoundTimerData
{
    int MinutesLeft;
    int SecondsLeft;
    bool bombPlanted;
    int ctScore;
    int tScore;
} roundTimerData;

RkHudRoundTimer::RkHudRoundTimer(const char *value) : CHudElement( value ),
                                                      m_bVisible( false ),
                                                      m_pInstance( nullptr )
{
    SetHiddenBits( /* HIDEHUD_MISCSTATUS */ 0 );
}

static void UnloadRkRoundTimer()
{
    RkHudRoundTimer *pRoundTimer = GET_HUDELEMENT(RkHudRoundTimer);
    if (!pRoundTimer)
    {
        Warning("Couldn't grab hud roundtimer to load!\n");
        return;
    }

    // Not loaded
    if( !pRoundTimer->m_pInstance )
        return;

    Rml::Context *hudCtx = RocketUI()->AccessHudContext();
    if( hudCtx )
    {
        hudCtx->RemoveDataModel( "roundtimer_model" );
        pRoundTimer->m_dataModel = nullptr;
    }
    else
    {
        Warning("Couldn't access hudctx to unload scoreboard datamodel!\n");
    }

    pRoundTimer->m_pInstance->Close();
    pRoundTimer->m_pInstance = nullptr;
}

static void LoadRkRoundTimer()
{
    RkHudRoundTimer *pRoundTimer = GET_HUDELEMENT(RkHudRoundTimer);
    if (!pRoundTimer)
    {
        Warning("Couldn't grab hud roundtimer to load!\n");
        return;
    }

    Rml::Context *hudCtx = RocketUI()->AccessHudContext();
    if (!hudCtx)
    {
        Error("Couldn't access hudctx!\n");
        /* Exit */
    }

    if( pRoundTimer->m_pInstance || pRoundTimer->m_dataModel )
    {
        Warning( "RkRoundTimer already loaded, call unload first!\n");
        return;
    }

    // Create the data binding, this will sync data between rocketui and the game.
    Rml::DataModelConstructor constructor = hudCtx->CreateDataModel("roundtimer_model" );
    if( !constructor )
    {
        Error( "Couldn't create datamodel for roundtimer!\n" );
        /* Exit */
    }

    constructor.Bind( "minutes_left", &roundTimerData.MinutesLeft );
    constructor.Bind( "seconds_left", &roundTimerData.SecondsLeft );
    constructor.Bind( "bomb_planted", &roundTimerData.bombPlanted );
    constructor.Bind( "ct_score", &roundTimerData.ctScore );
    constructor.Bind( "t_score", &roundTimerData.tScore );

    pRoundTimer->m_dataModel = constructor.GetModelHandle();

    pRoundTimer->m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_HUD, "hud_roundtimer.rml", &LoadRkRoundTimer, &UnloadRkRoundTimer );

    if( !pRoundTimer->m_pInstance )
    {
        Error( "Couldn't create hud_roundtimer document!\n" );
        /* Exit */
    }

    pRoundTimer->ShowPanel( false, false );
}

RkHudRoundTimer::~RkHudRoundTimer() noexcept
{
    UnloadRkRoundTimer();
}

void RkHudRoundTimer::LevelInit()
{
    LoadRkRoundTimer();
}

void RkHudRoundTimer::LevelShutdown()
{
    UnloadRkRoundTimer();
}

void RkHudRoundTimer::ShowPanel(bool bShow, bool force)
{
    if( !m_pInstance )
        return;

    if( bShow )
    {
        if( !m_bVisible )
        {
            m_pInstance->Show();
        }

        int remainingTime;
        if ( CSGameRules()->IsFreezePeriod() )
        {
            // countdown to the start of the round while we're in freeze period
            remainingTime = (int)ceil( CSGameRules()->GetRoundStartTime() - gpGlobals->curtime );
        }
        else
        {
            remainingTime = (int)ceil( CSGameRules()->GetRoundRemainingTime() );
        }
        roundTimerData.SecondsLeft = remainingTime % 60;
        roundTimerData.MinutesLeft = remainingTime / 60;
        roundTimerData.ctScore = g_PR->GetTeamScore( TEAM_CT );
        roundTimerData.tScore = g_PR->GetTeamScore( TEAM_TERRORIST );
        roundTimerData.bombPlanted = CSGameRules()->m_bBombPlanted;


        m_dataModel.DirtyVariable( "minutes_left" );
        m_dataModel.DirtyVariable( "seconds_left" );
        m_dataModel.DirtyVariable( "bomb_planted" );
        m_dataModel.DirtyVariable( "ct_score" );
        m_dataModel.DirtyVariable( "t_score" );

        m_dataModel.Update();
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

void RkHudRoundTimer::SetActive(bool bActive)
{
    ShowPanel( bActive, false );
    CHudElement::SetActive( bActive );
}

bool RkHudRoundTimer::ShouldDraw()
{
    C_CSPlayer *localPlayer = C_CSPlayer::GetLocalCSPlayer();

    if( !localPlayer || localPlayer->GetTeamNumber() == TEAM_UNASSIGNED )
        return false;

    return cl_drawhud.GetBool() && CHudElement::ShouldDraw();
}