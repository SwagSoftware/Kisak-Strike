#include "rkhud_killfeed.h"

#include "cbase.h"
#include "hud_macros.h"
#include "c_cs_player.h"

#include <tier0/valve_minmax_off.h> // included to fix an error with min/max and rocketui

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"
#include <deque>

DECLARE_HUDELEMENT( RkHudKillfeed );

ConVar rocket_hud_killfeed_linger_time( "rocket_hud_killfeed_linger_time", "5", 0, "How long in seconds to keep each killfeed entry on screen." );

struct KillfeedEntry
{
    Rml::String attackerName;
    Rml::String gunName;
    Rml::String victimName;
    bool headshot;
    bool wallbang;
    float noticeSpawnTime;
};

// Struct layout for data-binding model.
struct KillFeedData
{
    std::deque<KillfeedEntry> entries;
} killFeedData;

void RkHudKillfeed::OnPlayerDeath( IGameEvent *event )
{
    KillfeedEntry entry;
    int nAttacker = engine->GetPlayerForUserID( event->GetInt( "attacker" ) );
    int nVictim = engine->GetPlayerForUserID( event->GetInt( "userid" ) );

    CCSPlayer* attacker = ToCSPlayer( ClientEntityList().GetBaseEntity( nAttacker ) );
    CCSPlayer* victim = ToCSPlayer( ClientEntityList().GetBaseEntity( nVictim ) );

    if( !attacker || !victim )
        return;

    entry.victimName = victim->GetPlayerName();
    entry.attackerName = attacker->GetPlayerName();
    entry.gunName = event->GetString( "weapon" );
    entry.headshot = ( event->GetInt( "headshot" ) > 0 );
    entry.wallbang = ( event->GetInt( "penetrated" ) > 0 );

    entry.noticeSpawnTime = gpGlobals->curtime;

    killFeedData.entries.push_back( entry );

    m_dataModel.DirtyVariable( "killfeed_entries");
    m_dataModel.Update();
}

// called every frame
void RkHudKillfeed::CheckForOldEntries()
{
    if( killFeedData.entries.empty() )
        return;

    // pop off the first guy, then we're done. This gets called often enough to not matter about the rest.
    if( (gpGlobals->curtime - killFeedData.entries.front().noticeSpawnTime) > rocket_hud_killfeed_linger_time.GetFloat() )
    {
        killFeedData.entries.pop_front();
        m_dataModel.DirtyVariable( "killfeed_entries");
        m_dataModel.Update();
    }
}

static void UnloadRkKillFeed()
{
    RkHudKillfeed *pKillFeed = GET_HUDELEMENT( RkHudKillfeed );
    if( !pKillFeed )
    {
        Warning( "Couldn't grab hud killfeed to load!\n" );
        return;
    }

    // Not loaded.
    if( !pKillFeed->m_pInstance )
        return;

    Rml::Context *hudCtx = RocketUI()->AccessHudContext();
    if( hudCtx )
    {
        hudCtx->RemoveDataModel( "killfeed_model" );
        pKillFeed ->m_dataModel = nullptr;
    }
    else
    {
        Warning( "couldn't access hudctx to unload killfeed datamodel!\n" );
    }

    pKillFeed->m_pInstance->Close();
    pKillFeed->m_pInstance = nullptr;
    pKillFeed->m_bVisible = false;
}

static void LoadRkKillFeed()
{
    RkHudKillfeed *pKillFeed = GET_HUDELEMENT( RkHudKillfeed );
    if( !pKillFeed )
    {
        Warning( "Couldn't grab hud killfeed to load!\n" );
        return;
    }

    Rml::Context *hudCtx = RocketUI()->AccessHudContext();
    if( !hudCtx )
    {
        Error( "Couldn't access hudctx!\n" );
        /* Exit */
    }

    if( pKillFeed->m_pInstance || pKillFeed->m_dataModel )
    {
        Warning( "RkKillFeed already loaded, call unload first!\n" );
        return;
    }

    // Create the data binding, this will sync data between rocketui and the game.
    Rml::DataModelConstructor constructor = hudCtx->CreateDataModel( "killfeed_model" );
    if( !constructor )
    {
        Error( "Couldn't create datamodel for killfeed!\n" );
        /* Exit */
    }

    // Register KillfeedEntry struct definition
    if( auto killfeedentry_handle = constructor.RegisterStruct<KillfeedEntry>() )
    {
        killfeedentry_handle.RegisterMember( "attacker_name", &KillfeedEntry::attackerName );
        killfeedentry_handle.RegisterMember( "gun_name", &KillfeedEntry::gunName );
        killfeedentry_handle.RegisterMember( "victim_name", &KillfeedEntry::victimName );
        killfeedentry_handle.RegisterMember( "headshot", &KillfeedEntry::headshot );
        killfeedentry_handle.RegisterMember( "wallbang", &KillfeedEntry::wallbang );
    }

    // Register array-type of KillfeedEntry
    constructor.RegisterArray<std::deque<KillfeedEntry>>();

    // Bind the killfeed entry array
    constructor.Bind( "killfeed_entries", &killFeedData.entries );

    pKillFeed->m_dataModel = constructor.GetModelHandle();

    // Load document from file.
    pKillFeed->m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_HUD, "hud_killfeed.rml", &LoadRkKillFeed, &UnloadRkKillFeed );
    if( !pKillFeed->m_pInstance )
    {
        Error( "Couldn't create hud_killfeed document!\n" );
        /* Exit */
    }

    pKillFeed->ShowPanel( true, false );
}

RkHudKillfeed::RkHudKillfeed(const char *value) : CHudElement( value ),
                                                  m_bVisible( false ),
                                                  m_pInstance( nullptr )
{
    SetHiddenBits( /* HIDEHUD_MISCSTATUS */ 0 );
}

RkHudKillfeed::~RkHudKillfeed() noexcept
{
    StopListeningForAllEvents();

    UnloadRkKillFeed();
}

void RkHudKillfeed::LevelInit()
{
    ListenForGameEvent( "player_death" );

    LoadRkKillFeed();
}

void RkHudKillfeed::LevelShutdown()
{
    killFeedData.entries.clear();

    UnloadRkKillFeed();
}

void RkHudKillfeed::ShowPanel(bool bShow, bool force)
{
    if( !m_pInstance )
        return;

    if( bShow )
    {
        if( !m_bVisible )
        {
            m_pInstance->Show();
        }
        CheckForOldEntries();
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

void RkHudKillfeed::SetActive(bool bActive)
{
    ShowPanel( bActive, false );
    CHudElement::SetActive( bActive );
}

bool RkHudKillfeed::ShouldDraw()
{
    return cl_drawhud.GetBool() && CHudElement::ShouldDraw();
}

void RkHudKillfeed::FireGameEvent(IGameEvent *event)
{
    // We only listen for "player_death"
    OnPlayerDeath( event );
}