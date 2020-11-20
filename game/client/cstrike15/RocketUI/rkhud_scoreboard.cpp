#include "rkhud_scoreboard.h"

#include "cbase.h"
#include "hud_macros.h"
#include "c_cs_player.h"
#include "in_buttons.h"
#include "c_playerresource.h"

#include <tier0/valve_minmax_off.h> // included to fix an error with min/max and rocketui
#include <c_team.h>

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"

DECLARE_HUDELEMENT( RkHudScoreboard );

// Struct layout for data-binding model.
struct PlayerEntry
{
    int entid;
    Rml::String name;
    int teamnum;

    int cash;
    int kills;
    int deaths;
    int assists;
    int ping;
};
struct ScoreboardData
{
    int tScore;
    int ctScore;
    int numSpecs;
    Rml::Vector<PlayerEntry> ctPlayers;
    Rml::Vector<PlayerEntry> tPlayers;
} scoreboardData;

static void UnloadRkScoreboard()
{
    RkHudScoreboard *pScoreboard = GET_HUDELEMENT( RkHudScoreboard );
    if( !pScoreboard )
    {
        Warning( "Couldn't grab RkHudScoreboard element to unload!\n");
        return;
    }

    // Not loaded
    if( !pScoreboard->m_pInstance )
        return;

    Rml::Context *hudCtx = RocketUI()->AccessHudContext();
    if( hudCtx )
    {
        hudCtx->RemoveDataModel( "scoreboard_model" );
        pScoreboard->m_dataModel = nullptr;
    }
    else
    {
        Warning("Couldn't access hudctx to unload scoreboard datamodel!\n");
    }

    pScoreboard->m_pInstance->Close();
    pScoreboard->m_pInstance = nullptr;
}

static void LoadRkScoreboard()
{
    RkHudScoreboard *pScoreboard = GET_HUDELEMENT( RkHudScoreboard );
    if( !pScoreboard )
    {
        Warning( "Couldn't grab hud RkHudScoreboard to load!\n");
        return;
    }

    Rml::Context *hudCtx = RocketUI()->AccessHudContext();
    if( !hudCtx )
    {
        Error("Couldn't access hudctx!\n");
        /* Exit */
    }

    // Create the data-binding to sync data between rocketui and the game.
    Rml::DataModelConstructor constructor = hudCtx->CreateDataModel( "scoreboard_model" );
    if( !constructor )
    {
        Error( "Couldn't create datamodel for scoreboard!\n");
        /* Exit */
    }

    // Register the PlayerEntry struct definition
    if( auto playerentry_handle  = constructor.RegisterStruct<PlayerEntry>())
    {
        playerentry_handle.RegisterMember("entid", &PlayerEntry::entid);
        playerentry_handle.RegisterMember("name", &PlayerEntry::name);
        playerentry_handle.RegisterMember("teamnum", &PlayerEntry::teamnum);
        playerentry_handle.RegisterMember("cash", &PlayerEntry::cash);
        playerentry_handle.RegisterMember("kills", &PlayerEntry::kills);
        playerentry_handle.RegisterMember("deaths", &PlayerEntry::deaths);
        playerentry_handle.RegisterMember("assists", &PlayerEntry::assists);
        playerentry_handle.RegisterMember("ping", &PlayerEntry::ping);
    }

    // Register Array-type of PlayerEntry
    constructor.RegisterArray<Rml::Vector<PlayerEntry>>();

    // Bind the two PlayerEntry arrays of players
    constructor.Bind("ctplayers", &scoreboardData.ctPlayers);
    constructor.Bind("tplayers", &scoreboardData.tPlayers);

    // Bind the plain data
    constructor.Bind("ct_score", &scoreboardData.ctScore);
    constructor.Bind("t_score", &scoreboardData.tScore);
    constructor.Bind("num_specs", &scoreboardData.numSpecs);

    pScoreboard->m_dataModel = constructor.GetModelHandle();

    // Load document from file.
    pScoreboard->m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_HUD, "hud_scoreboard.rml", LoadRkScoreboard, UnloadRkScoreboard );
    if( !pScoreboard->m_pInstance )
    {
        Error("Couldn't create hud_scoreboard document!\n");
        /* Exit */
    }

    // Cache some elements from the document for use later.
    pScoreboard->m_elemCtSection = pScoreboard->m_pInstance->GetElementById( "ct" );
    if( !pScoreboard->m_elemCtSection )
    {
        Error( "Couldn't find required element in hud_scoreboard!\n");
        /* Exit */
    }

    pScoreboard->m_elemTSection = pScoreboard->m_pInstance->GetElementById( "t" );
    if( !pScoreboard->m_elemTSection )
    {
        Error( "Couldn't find required element in hud_scoreboard!\n");
        /* Exit */
    }
}

RkHudScoreboard::RkHudScoreboard(const char *value) : CHudElement( value ),
                                                      m_bVisible( false ),
                                                      m_pInstance( nullptr )
{
    SetHiddenBits( /* HIDEHUD_MISCSTATUS */ 0 );
}

RkHudScoreboard::~RkHudScoreboard() noexcept
{
    UnloadRkScoreboard();
}

void RkHudScoreboard::LevelInit()
{
    LoadRkScoreboard();
}

void RkHudScoreboard::LevelShutdown()
{
    UnloadRkScoreboard();
}

void RkHudScoreboard::Update()
{
    IGameResources *gr = GameResources();
    if( !gr )
        return;

    for( int i = 1; i < gpGlobals->maxClients; i++ )
    {
        PlayerEntry *playerEntry = nullptr;
        int foundIndex = 0;

        int playerTeam = gr->GetTeam(i);

        // look for existing entry in these arrays
        for( int index = 0; index < scoreboardData.ctPlayers.size(); index++ )
        {
            if( scoreboardData.ctPlayers[index].entid == i )
            {
                // delete this entry if team has changed since.
                if( playerTeam != scoreboardData.ctPlayers[index].teamnum )
                {
                    scoreboardData.ctPlayers.erase( scoreboardData.ctPlayers.begin() + index );
                    if( index > 0 )
                        index--;
                    continue;
                }
                playerEntry = &scoreboardData.ctPlayers[index];
                foundIndex = index;
                break;
            }
        }
        for( int index = 0; index < scoreboardData.tPlayers.size(); index++ )
        {
            if( scoreboardData.tPlayers[index].entid == i )
            {
                // delete this entry if team has changed since.
                if( playerTeam != scoreboardData.tPlayers[index].teamnum )
                {
                    scoreboardData.tPlayers.erase( scoreboardData.tPlayers.begin() + index );
                    if( index > 0 )
                        index--;
                    continue;
                }
                playerEntry = &scoreboardData.tPlayers[index];
                foundIndex = index;
                break;
            }
        }

        if( gr->IsConnected( i ) )
        {
            // Create a new entry for this player.
            if( !playerEntry )
            {
                PlayerEntry entry = {0};
                entry.entid = i;
                if( playerTeam == TEAM_TERRORIST )
                {
                    entry.teamnum = TEAM_TERRORIST;
                    scoreboardData.tPlayers.push_back( entry );
                    playerEntry = &scoreboardData.tPlayers[scoreboardData.tPlayers.size() - 1];
                }
                else if( playerTeam == TEAM_CT )
                {
                    entry.teamnum = TEAM_CT;
                    scoreboardData.ctPlayers.push_back( entry );
                    playerEntry = &scoreboardData.ctPlayers[scoreboardData.ctPlayers.size() - 1];
                }
                else
                {
                    //spectators dont get an entry
                    continue;
                }
            }

            C_CSPlayer *player = ToCSPlayer( UTIL_PlayerByIndex( i ) );
            if( !player )
                continue;

            // Update the data in the player's entry. *playerEntry will be set at this point.
            if( player->IsBot() )
            {
                playerEntry->name = ( Rml::String("BOT ") + gr->GetPlayerName( i ) );
            }
            else
            {
                playerEntry->name = gr->GetPlayerName( i );
            }
            playerEntry->cash = player->GetAccount();
            playerEntry->kills = gr->GetKills( i );
            playerEntry->deaths = gr->GetDeaths( i );
            playerEntry->assists = g_PR->GetAssists( i );
            playerEntry->ping = gr->GetPing( i );
        }
        else
        {
            if( playerEntry )
            {
                // Player has disconnected, need to remove his row.
                if( playerEntry->teamnum == TEAM_TERRORIST )
                {
                    scoreboardData.tPlayers.erase( scoreboardData.tPlayers.begin() + foundIndex );
                }
                else if( playerEntry->teamnum == TEAM_CT )
                {
                    scoreboardData.ctPlayers.erase( scoreboardData.ctPlayers.begin() + foundIndex );
                }
            }
        }
    }


    scoreboardData.numSpecs = GetGlobalTeam( TEAM_SPECTATOR )->GetNumPlayers();
    scoreboardData.ctScore = gr->GetTeamScore( TEAM_CT );
    scoreboardData.tScore = gr->GetTeamScore( TEAM_TERRORIST );

    // Dirty the variables to note change.
    m_dataModel.DirtyVariable( "ct_score" );
    m_dataModel.DirtyVariable( "t_score" );
    m_dataModel.DirtyVariable( "num_specs" );

    m_dataModel.DirtyVariable( "ctplayers");
    m_dataModel.DirtyVariable( "tplayers");

    m_dataModel.Update();
}

// this is called every frame, keep that in mind.
void RkHudScoreboard::ShowPanel( bool bShow, bool force )
{
    if( !m_pInstance )
        return;

    if( bShow )
    {
        // Update the information every frame while the scoreboard is open.
        Update();

        if( !m_bVisible )
        {
            m_pInstance->Show();
        }
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

void RkHudScoreboard::SetActive(bool bActive)
{
    ShowPanel( bActive, false );
    CHudElement::SetActive( bActive );
}

bool RkHudScoreboard::ShouldDraw()
{
    int buttons = input->GetButtonBits( false );

    if( !(buttons & IN_SCORE) )
        return false;

    return cl_drawhud.GetBool() && CHudElement::ShouldDraw();
}