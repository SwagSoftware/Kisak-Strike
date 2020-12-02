#include "rkpanel_options.h"

#include "cbase.h"
#include "cdll_client_int.h" // extern globals to interfaces like engineclient

#include <tier0/valve_minmax_off.h> // included to fix an error with min/max and rocketui

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"

Rml::ElementDocument *RocketOptionsDocument::m_pInstance = nullptr;
bool RocketOptionsDocument::m_bVisible = false;
bool RocketOptionsDocument::m_bGrabbingInput = false;

/* Event Listener added to each button */
class RkOptionsMenuButtons : public Rml::EventListener
{
public:
    void ProcessEvent(Rml::Event& keyevent) override
    {
        const char *id = keyevent.GetTargetElement()->GetId().c_str();

        // Currently, only mousedown events.
        if( !strcmp( id, "close" ) )
        {
            RocketOptionsDocument::ShowPanel( false );
            // Unload this panel so it grabs fresh data each time it's opened and so the context can change.
            RocketOptionsDocument::UnloadDialog();
        }
        else if( !strcmp( id, "save" ) )
        {
            Msg("Saving Video Settings.\n");
            engine->ClientCmd_Unrestricted( "mat_savechanges" );
        }
    }
};
static RkOptionsMenuButtons optionsMenuButtonsListener;

RocketOptionsDocument::RocketOptionsDocument()
{
}

RocketOptionsDocument::~RocketOptionsDocument()
{
}

void RocketOptionsDocument::LoadDialog()
{
    if( !m_pInstance )
    {
        m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_CURRENT, "panel_options.rml", RocketOptionsDocument::LoadDialog, RocketOptionsDocument::UnloadDialog );
        if( !m_pInstance )
        {
            Error( "Couldn't create rocketui options!\n" );
            /* Exit */
        }
        m_pInstance->GetElementById("close")->AddEventListener( Rml::EventId::Click, &optionsMenuButtonsListener );
        m_pInstance->GetElementById("save")->AddEventListener( Rml::EventId::Click, &optionsMenuButtonsListener );
    }
}

void RocketOptionsDocument::UnloadDialog()
{
    if( m_pInstance )
    {
        m_pInstance->Close();
        m_pInstance = nullptr;

        if( m_bGrabbingInput )
            RocketUI()->DenyInputToGame( false, "OptionsPanel" );
    }
}

void RocketOptionsDocument::ShowPanel(bool bShow, bool immediate)
{
    if( bShow )
    {
        if( !m_pInstance )
            LoadDialog();

        m_pInstance->Show();

        if( !m_bGrabbingInput )
        {
            RocketUI()->DenyInputToGame( true, "OptionsPanel" );
            m_bGrabbingInput = true;
        }
    }
    else
    {
        if( m_pInstance )
            m_pInstance->Hide();

        if( m_bGrabbingInput )
        {
            RocketUI()->DenyInputToGame( false, "OptionsPanel" );
            m_bGrabbingInput = false;
        }
    }

    m_bVisible = bShow;
}