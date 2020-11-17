#include "cs_gc_client.h"

#include <gcsdk/msgbase.h>
#include "gcsystemmsgs.pb.h"
static class CCSGCClientSystem s_CCSGCClientSystem;
class CCSGCClientSystem *CCSGCClientSystem() { return &s_CCSGCClientSystem; }

CCSGCClientSystem::CCSGCClientSystem()
{
    //Replace the base GCClientSystem
    SetGCClientSystem( this );
}

CCSGCClientSystem::~CCSGCClientSystem() noexcept
{
    // Prevent the GCClientSystem from being used after this is destroyed
    SetGCClientSystem( NULL );
}

bool CCSGCClientSystem::Init()
{
    // The retail build does nothing here.

    // Register the CSGO-specific messages with the gcsdk so it can log the names.
    GCSDK::MsgRegistrationFromEnumDescriptor( ECsgoGCMsg_descriptor(), GCSDK::MT_GC );
    return true;
}

void CCSGCClientSystem::PostInit()
{
    BaseClass::PostInit();
}

void CCSGCClientSystem::PreInitGC()
{
    // This is where the Game will tell Steam not to update the App for XXX Minutes
    // I don't think this is necessary for Kisak-strike
}

void CCSGCClientSystem::PostInitGC()
{
    // This is accurate to the retail build
    /* Do nothing */
}

void CCSGCClientSystem::LevelInitPreEntity()
{
    BaseClass::LevelInitPreEntity();
}
void CCSGCClientSystem::LevelShutdownPostEntity()
{
    BaseClass::LevelShutdownPostEntity();
}
void CCSGCClientSystem::Shutdown()
{
    BaseClass::Shutdown();
}
void CCSGCClientSystem::Update( float frametime )
{
    BaseClass::Update( frametime );
}

void CCSGCClientSystem::OnReceivedMatchmakingWelcomeMessage(const CMsgGCCStrike15_v2_MatchmakingGC2ClientHello &msg)
{
    Msg("[GC] Matchmaking Hello Recieved...\n");
    if( msg.has_account_id() )
    {
        Msg("\tAccount ID: %d\n", msg.account_id() );
    }
    if( msg.has_ongoingmatch() )
    {
        Msg("\tWow, it looks like you're in an ongoing MatchMaking Match!\n");
    }
    if( msg.has_vac_banned() )
    {
        Msg("\tYou %s VAC Banned\n", msg.vac_banned() > 0 ? "ARE" : "are NOT");
    }
    if( msg.has_ranking() )
    {
        Msg( "\tYou have some ranking info available.\n");
    }
    if( msg.has_commendation() )
    {
        Msg( "\tYou have commendation info available.\n" );
    }
    if( msg.has_medals() )
    {
        Msg( "\tYou have medals info available.\n" );
    }
    if( msg.has_my_current_event() )
    {
        Msg( "\tYou have a current Tournament event.\n" );
        // current team
        // current event stages
        // ...
    }
    if( msg.has_survey_vote() )
    {
        Msg( "\tSurvey Vote: %d\n", msg.survey_vote() );
    }
    if( msg.has_activity() )
    {
        Msg( "\tYou have Account Activity...\n");
        Msg( "\t\tactivity: %d\n", msg.activity().activity());
        Msg( "\t\tmode: %d", msg.activity().mode());
        Msg( "\t\tmap: %d", msg.activity().map());
    }
    if( msg.has_player_level() )
    {
        Msg("\tPlayer Level: %d\n", msg.player_level() );
    }
    if( msg.has_player_cur_xp() )
    {
        Msg("\tPlayer XP: %d\n", msg.player_cur_xp() );
    }
    if( msg.has_player_xp_bonus_flags() )
    {
        Msg("\tPlayer Bonus XP: %d\n", msg.player_xp_bonus_flags() );
    }
    if( msg.has_ranking() )
    {
        Msg("\tYou have Ranking Data available.\n" );
    }
}

class CGCClientCStrike15MatchmakingWelcomeMessage : public GCSDK::CGCClientJob
{
public:
    CGCClientCStrike15MatchmakingWelcomeMessage( GCSDK::CGCClient *pGCClient ) : GCSDK::CGCClientJob( pGCClient ) { }

    virtual bool BYieldingRunJobFromMsg( GCSDK::IMsgNetPacket *pNetPacket )
    {
        GCSDK::CProtoBufMsg<CMsgGCCStrike15_v2_MatchmakingGC2ClientHello> msg( pNetPacket );
        CCSGCClientSystem()->OnReceivedMatchmakingWelcomeMessage( msg.Body() );
        return true;
    }
};
GC_REG_JOB( GCSDK::CGCClient, CGCClientCStrike15MatchmakingWelcomeMessage, "CGCClientCStrike15MatchmakingWelcomeMessage", k_EMsgGCCStrike15_v2_MatchmakingGC2ClientHello, GCSDK::k_EServerTypeGCClient );
