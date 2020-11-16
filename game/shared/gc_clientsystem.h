#ifndef KISAKSTRIKE_GC_CLIENTSYSTEM_H
#define KISAKSTRIKE_GC_CLIENTSYSTEM_H

#include <gcsdk/gcclientsdk.h>
#include "igamesystem.h"
#include "networkvar.h"
#include <gcsdk/gcsystemmsgs.h>
#include <gcsdk_gcmessages.pb.h>

#ifdef CLIENT_DLL
#include "clientsteamcontext.h"
#endif

class CMsgClientWelcome;
//class CMsgClientGoodbye;

//=============================================================================
//
//	Client GC System.
//
//=============================================================================
class CGCClientSystem : public CAutoGameSystemPerFrame
{
    DECLARE_CLASS_GAMEROOT( CGCClientSystem, CAutoGameSystem );

public:

    // Constructor/Destructor.
    CGCClientSystem();
    ~CGCClientSystem();

    // Init/Shutdown.
    virtual void PostInit() OVERRIDE;
    virtual void LevelInitPreEntity() OVERRIDE;
    virtual void LevelShutdownPostEntity() OVERRIDE;
    virtual void Shutdown() OVERRIDE;

    // Updates.  Gameservers do this at a slightly different place than clients
#ifdef CLIENT_DLL
    virtual void Update( float frametime ) OVERRIDE;
#else
    virtual void PreClientUpdate() OVERRIDE;
#endif

    // Connection status
    bool BConnectedtoGC() const { return m_bConnectedToGC; }

    // GC Messages
    bool BSendMessage( uint32 unMsgType, const uint8 *pubData, uint32 cubData );
    bool BSendMessage( const GCSDK::CGCMsgBase& msg );
    bool BSendMessage( const GCSDK::CProtoBufMsgBase& msg );

    // GC SOCache
    GCSDK::CGCClientSharedObjectCache *GetSOCache( const CSteamID &steamID );
    GCSDK::CGCClientSharedObjectCache *FindOrAddSOCache( const CSteamID &steamID );

    // GC Client
    GCSDK::CGCClient *GetGCClient();

    // Steam
#ifndef CLIENT_DLL
    void GameServerActivate();
#endif

protected:

    void SetupGC();
    virtual void InitGC();
    virtual void PreInitGC() {}
    virtual void PostInitGC() {}

#ifdef CLIENT_DLL
    friend class CGCClientJobClientWelcome;
    virtual void ReceivedClientWelcome( const CMsgClientWelcome &msg );
    friend class CGCClientJobGCConnectionStatus;
    virtual void ReceivedClientGCConnectionStatus( const CMsgConnectionStatus &msg );
    //lwss- goodbye has been removed
    //friend class CGCClientJobClientGoodbye;
    //virtual void ReceivedClientGoodbye( const CMsgClientGoodbye &msg );
#else
    friend class CGCClientJobServerWelcome;
    virtual void ReceivedServerWelcome( const CMsgServerWelcome &msg );

    //lwss- goodbye has been removed
    //friend class CGCClientJobServerGoodbye;
    //virtual void ReceivedServerGoodbye( const CMsgServerGoodbye &msg );
#endif

    void SetConnectedToGC( bool bConnected ) { m_bConnectedToGC = bConnected; }

private:

#ifdef CLIENT_DLL
    void SteamLoggedOnCallback( const SteamLoggedOnChange_t &loggedOnState );
#else
    STEAM_GAMESERVER_CALLBACK( CGCClientSystem, OnLogonSuccess, SteamServersConnected_t, m_CallbackLogonSuccess );
#endif

    bool m_bInittedGC;
    bool m_bConnectedToGC;
    bool m_bLoggedOn;
    GCSDK::CGCClient m_GCClient;
    double m_timeLastSendHello;

    void ThinkConnection();

    friend class CGCClientSystemJob;
};


void SetGCClientSystem( CGCClientSystem* pGCClientSystem );
CGCClientSystem *GCClientSystem();


#endif //KISAKSTRIKE_GC_CLIENTSYSTEM_H