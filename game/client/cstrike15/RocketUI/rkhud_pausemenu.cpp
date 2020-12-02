#include "rkhud_pausemenu.h"

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"

#include "cdll_client_int.h" // extern globals to interfaces like engineclient

#include "rkhud_chat.h"
#include "rkhud_teammenu.h"
#include "rkhud_buymenu.h"
#include "rkpanel_options.h"

Rml::ElementDocument *RocketPauseMenuDocument::m_pInstance = nullptr;
bool RocketPauseMenuDocument::m_bVisible = false;
bool RocketPauseMenuDocument::m_bGrabbingInput = false;

/* Event Listener added to each button */
class RkPauseMenuButtons : public Rml::EventListener
{
public:
    void ProcessEvent(Rml::Event& mousedownevent) override
    {
        // Currently, only mousedown events.
        Rml::Element *elem = mousedownevent.GetTargetElement();
        if( !elem )
            return;

        Rml::String id = elem->GetId();
        if( id == "pm_resume" )
        {
            RocketPauseMenuDocument::ShowPanel( false );
        }
        else if( id == "pm_choose" )
        {
            RocketTeamMenuDocument::ShowPanel( true );
            RocketPauseMenuDocument::ShowPanel( false );
        }
        else if( id == "pm_callvote" )
        {

        }
        else if( id == "pm_options" )
        {
            RocketOptionsDocument::ShowPanel( true );
            RocketPauseMenuDocument::ShowPanel( false );
        }
        else if( id == "pm_disconnect" )
        {
            engine->ClientCmd_Unrestricted("disconnect");
        }
    }
};

static RkPauseMenuButtons pauseMenuButtonsListener;

RocketPauseMenuDocument::RocketPauseMenuDocument()
{
}

void RocketPauseMenuDocument::LoadDialog()
{
    if( !m_pInstance )
    {
        m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_HUD, "hud_pausemenu.rml", RocketPauseMenuDocument::LoadDialog, RocketPauseMenuDocument::UnloadDialog );
        if( !m_pInstance )
        {
            Error( "Couldn't create rocketui pause menu!\n");
            /* Exit */
        }
        RocketUI()->RegisterPauseMenu( RocketPauseMenuDocument::TogglePanel );

        // Add a listener to each button, this seems better than custom events for these
        Rml::ElementList menuButtons;
        m_pInstance->GetElementsByTagName( menuButtons, "button" );
        for( Rml::Element *elem : menuButtons )
        {
            elem->AddEventListener( Rml::EventId::Mousedown, &pauseMenuButtonsListener );
        }
    }
}

void RocketPauseMenuDocument::UnloadDialog()
{
    if( m_pInstance )
    {
        RocketUI()->RegisterPauseMenu( nullptr );
        m_pInstance->Close();
        m_pInstance = nullptr;
        if( m_bGrabbingInput )
        {
            RocketUI()->DenyInputToGame( false, "PauseMenu" );
            m_bGrabbingInput = false;
        }
    }
}

void RocketPauseMenuDocument::UpdateDialog()
{
}

void RocketPauseMenuDocument::TogglePanel()
{
    ShowPanel( !m_bVisible );
}

void RocketPauseMenuDocument::ShowPanel(bool bShow, bool immediate)
{
    // oh yeah buddy this'll get called before the loading sometimes
    // so if it does, load it for the caller.
    if( bShow )
    {
        if( !m_pInstance )
            LoadDialog();

        RkHudChat *pChat = GET_HUDELEMENT( RkHudChat );
        RkHudBuyMenu *pBuyMenu = GET_HUDELEMENT( RkHudBuyMenu );

        if( !pChat || !pBuyMenu )
            return;

        if( pChat->ChatRaised() )
        {
            return;
        }
        if( pBuyMenu->m_bVisible || !pBuyMenu->m_pInstance )
        {
            return;
        }

        m_pInstance->Show();

        if( !m_bGrabbingInput )
        {
            RocketUI()->DenyInputToGame( true, "PauseMenu" );
            m_bGrabbingInput = true;
        }
    }
    else
    {
        if( m_pInstance )
            m_pInstance->Hide();

        if( m_bGrabbingInput )
        {
            RocketUI()->DenyInputToGame( false, "PauseMenu" );
            m_bGrabbingInput = false;
        }
    }

    m_bVisible = bShow;
}