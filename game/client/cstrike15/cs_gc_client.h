#ifndef KISAKSTRIKE_CS_GC_CLIENT_H
#define KISAKSTRIKE_CS_GC_CLIENT_H

#if !defined( NO_STEAM )

#include "steam/steam_api.h"
#endif

#include "gcsdk/gcclientsdk.h" // /gcsdk stuff
#include "networkvar.h"
#include <gc_clientsystem.h> // base GCClient
#include "cstrike15_gcmessages.pb.h"

class CCSGCClientSystem : public CGCClientSystem//, public GCSDK::ISharedObjectListener
{
    DECLARE_CLASS_GAMEROOT( CCSGCClientSystem, CGCClientSystem );
public:
    CCSGCClientSystem( void );
    ~CCSGCClientSystem( void );

    // CAutoGameSystemPerFrame
    virtual bool Init() OVERRIDE;
    virtual void PostInit() OVERRIDE;
    virtual void LevelInitPreEntity() OVERRIDE;
    virtual void LevelShutdownPostEntity() OVERRIDE;
    virtual void Shutdown() OVERRIDE;
    virtual void Update( float frametime ) OVERRIDE;

    // CGCClientSystem
    virtual void PreInitGC() OVERRIDE;
    virtual void PostInitGC() OVERRIDE;

    // this
    void OnReceivedMatchmakingWelcomeMessage( const CMsgGCCStrike15_v2_MatchmakingGC2ClientHello &msg );
};


CCSGCClientSystem* CCSGCClientSystem();

#endif //KISAKSTRIKE_CS_GC_CLIENT_H
