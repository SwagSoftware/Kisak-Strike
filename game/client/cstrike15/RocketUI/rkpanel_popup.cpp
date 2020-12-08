#include "rkpanel_popup.h"

#include "cbase.h"
#include "cdll_client_int.h" // extern globals to interfaces like engineclient

#include <tier0/valve_minmax_off.h> // included to fix an error with min/max and rocketui

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"

#include "convar.h"

CON_COMMAND(rocket_test_popup, "Create a Test Popup in RocketUI")
{
	if ( args.ArgC() != 3 )
	{
        Msg( "Example usage: rocket_test_popup <messages> <timeout>" );
	}
	else
    {
        float timeout = Q_atof(args.ArgV()[2]);
        const char *text = args.ArgV()[1];

        Msg( "Created new Popup...(%f) seconds\n", timeout);
        new RocketPopupDocument( text, timeout );
    }
}


void RocketPopupDocument::ProcessEvent(Rml::Event &event)
{
    if( event.GetId() == Rml::EventId::Animationend )
    {
        // Animation is over, this means it's time to close
        delete this;
    }
}

RocketPopupDocument::RocketPopupDocument(const char *text, float timeout)
{
    V_strncpy( m_textBuffer, text, MAX_POPUP_TEXT );
    m_fTimeout = timeout;

    m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_CURRENT, "panel_popup.rml" );
    m_pInstance->AddEventListener(Rml::EventId::Animationend, this );
    m_pInstance->GetElementById("text")->SetInnerRML( m_textBuffer );
    m_pInstance->Show();
    m_pInstance->Animate("opacity", Rml::Property{ 0.0f, Rml::Property::PX }, timeout, Rml::Tween{ Rml::Tween::Quadratic } );
}

RocketPopupDocument::~RocketPopupDocument()
{
    m_pInstance->RemoveEventListener( Rml::EventId::Animationend, this );
    m_pInstance->Close();
    m_pInstance = nullptr;
}