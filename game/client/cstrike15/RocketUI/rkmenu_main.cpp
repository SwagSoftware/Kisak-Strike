#include "rkmenu_main.h"

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"

#include "rkpanel_options.h"

Rml::ElementDocument *RocketMainMenuDocument::m_pInstance = nullptr;
bool RocketMainMenuDocument::showing = false;
bool RocketMainMenuDocument::grabbingInput = false;

class MainMenuEventListener : public Rml::EventListener
{
public:
    void ProcessEvent(Rml::Event &keyevent) override
    {
        // Click event on options menu button.
        RocketOptionsDocument::ShowPanel( true );
        keyevent.StopPropagation();
    }
};
static MainMenuEventListener mainMenuEventListener;

RocketMainMenuDocument::RocketMainMenuDocument()
{
}

void RocketMainMenuDocument::LoadDialog()
{
    if( !m_pInstance )
    {
        m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_MENU, "menu.rml", RocketMainMenuDocument::LoadDialog, RocketMainMenuDocument::UnloadDialog );
        m_pInstance->Show();
        if( !grabbingInput )
        {
            RocketUI()->DenyInputToGame( true, "MainMenu" );
            grabbingInput = true;
        }
        if( !m_pInstance )
        {
            Error("Couldn't create rocketui menu!\n");
            /* Exit */
        }
        m_pInstance->GetElementById("options-menu-btn")->AddEventListener(Rml::EventId::Click, &mainMenuEventListener);
    }
}

void RocketMainMenuDocument::UnloadDialog()
{
    if( m_pInstance )
    {
        m_pInstance->GetElementById("options-menu-btn")->RemoveEventListener(Rml::EventId::Click, &mainMenuEventListener);
        m_pInstance->Close();
        m_pInstance = nullptr;
        if( grabbingInput )
        {
            RocketUI()->DenyInputToGame( false, "MainMenu" );
            grabbingInput = false;
        }
    }
}

void RocketMainMenuDocument::UpdateDialog()
{
    // I dont think this is needed here..
    //if( m_pInstance )
    //    m_pInstance->UpdateDocument()
}

void RocketMainMenuDocument::ShowPanel(bool bShow, bool immediate)
{
    // oh yeah buddy this'll get called before the loading sometimes
    // so if it does, load it for the caller.
    if( bShow )
    {
        if( !m_pInstance )
            LoadDialog();

        m_pInstance->Show();

        if( !grabbingInput )
        {
            RocketUI()->DenyInputToGame( true, "MainMenu" );
            grabbingInput = true;
        }
    }
    else
    {
        if( m_pInstance )
            m_pInstance->Hide();

        if( grabbingInput )
        {
            RocketUI()->DenyInputToGame( false, "MainMenu" );
            grabbingInput = false;
        }
    }

    showing = bShow;
}