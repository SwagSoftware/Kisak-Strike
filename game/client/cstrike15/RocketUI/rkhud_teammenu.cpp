#include "rkhud_teammenu.h"

#include "cbase.h"
#include "cdll_client_int.h" // extern globals to interfaces like engineclient

#include <tier0/valve_minmax_off.h> // included to fix an error with min/max and rocketui

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"

Rml::ElementDocument *RocketTeamMenuDocument::m_pInstance = nullptr;
bool RocketTeamMenuDocument::m_bVisible = false;
bool RocketTeamMenuDocument::m_bGrabbingInput = false;
RocketTeamMenuEventListener* RocketTeamMenuDocument::m_pEventListener = nullptr;

/* Event Listener added to each team-button */
class RkTeamMenuButtons : public Rml::EventListener
{
public:
    void ProcessEvent(Rml::Event& keyevent) override
    {
        // Currently, only mousedown events.
        Rml::Element *elem = keyevent.GetTargetElement();
        if( !elem )
            return;

        Rml::String id = elem->GetId();
        if( id == "team_ct" )
        {
            RocketTeamMenuDocument::ShowPanel( false );
            engine->ClientCmd_Unrestricted("jointeam 3");
        }
        else if( id == "team_t" )
        {
            RocketTeamMenuDocument::ShowPanel( false );
            engine->ClientCmd_Unrestricted("jointeam 2");
        }
        else if( id == "team_spec" )
        {
            RocketTeamMenuDocument::ShowPanel( false );
            engine->ClientCmd_Unrestricted( "jointeam 1");
        }
    }
};

static RkTeamMenuButtons teamMenuButtonsListener;

RocketTeamMenuEventListener::~RocketTeamMenuEventListener()
{
}

void RocketTeamMenuEventListener::StartAlwaysListenEvents()
{
    ListenForGameEvent( "jointeam_failed" );
    ListenForGameEvent( "player_spawned" );
    ListenForGameEvent( "teamchange_pending" );
}

void RocketTeamMenuEventListener::StopAlwaysListenEvents()
{
    StopListeningForAllEvents();
}

//listen for a few events related to joining teams.
void RocketTeamMenuEventListener::FireGameEvent(IGameEvent *event)
{
    const char *type = event->GetName();

    if( !V_strcmp( type, "jointeam_failed" ) )
    {
        ConMsg("oy vey the jointeam failed\n");
    }
    else if( !V_strcmp( type, "player_spawned" ) )
    {
        C_BasePlayer *localPlayer = C_BasePlayer::GetLocalPlayer();
        // If this was us.
        if( localPlayer && localPlayer->GetUserID() == event->GetInt( "userid" ) )
        {
            RocketTeamMenuDocument::ShowPanel( false, false );
        }
    }
    else if( !V_strcmp( type, "teamchange_pending" ) )
    {
        RocketTeamMenuDocument::ShowPanel( false, false );
    }
}

RocketTeamMenuDocument::RocketTeamMenuDocument()
{

}

RocketTeamMenuDocument::~RocketTeamMenuDocument() noexcept
{
}

void RocketTeamMenuDocument::LoadDialog()
{
    if( !m_pInstance )
    {
        m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_HUD, "hud_teammenu.rml", RocketTeamMenuDocument::LoadDialog, RocketTeamMenuDocument::UnloadDialog );

        if( !m_pInstance )
        {
            Error( "Couldn't create rocketui team-menu!\n");
            /* Exit */
        }

        // Add a listener to each button, this seems better than custom events for these
        Rml::ElementList teamButtons;
        m_pInstance->GetElementsByTagName( teamButtons, "button" );
        for( Rml::Element *elem : teamButtons )
        {
            elem->AddEventListener( Rml::EventId::Mousedown, &teamMenuButtonsListener );
        }

        m_pEventListener = new RocketTeamMenuEventListener;
        m_pEventListener->StartAlwaysListenEvents();
    }
}

void RocketTeamMenuDocument::UnloadDialog()
{
    if( m_pInstance )
    {
        m_pInstance->Close();
        m_pInstance = nullptr;
        if( m_pEventListener )
        {
            m_pEventListener->StopListeningForAllEvents();
            delete m_pEventListener;
            m_pEventListener = nullptr;
        }
    }
}

void RocketTeamMenuDocument::ShowPanel(bool bShow, bool immediate)
{
    // oh yeah buddy this'll get called before the loading sometimes
    // so if it does, load it for the caller.
    if( bShow )
    {
        if( !m_pInstance )
            LoadDialog();

        m_pInstance->Show();

        if( !m_bGrabbingInput )
        {
            RocketUI()->DenyInputToGame( true, "TeamMenu" );
            m_bGrabbingInput = true;
        }
    }
    else
    {
        if( m_pInstance )
            m_pInstance->Hide();

        if( m_bGrabbingInput )
        {
            RocketUI()->DenyInputToGame( false, "TeamMenu" );
            m_bGrabbingInput = false;
        }
    }

    m_bVisible = bShow;
}
