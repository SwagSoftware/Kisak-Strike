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
    // This is accurate to the retail build
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
    Msg("[GC] Matchmaking Hello Recieved...\nYou are Account#(%lu) - you are %s\n", msg.account_id(), msg.vac_banned() > 0 ? "VAC BANNED!" : "NOT VAC Banned." );
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
