#include "rkmenu_main.h"

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"

Rml::ElementDocument *RocketMainMenuDocument::m_pInstance = nullptr;
bool RocketMainMenuDocument::showing = false;
bool RocketMainMenuDocument::grabbingInput = false;

RocketMainMenuDocument::RocketMainMenuDocument()
{
}

void RocketMainMenuDocument::LoadDialog()
{
    if( !m_pInstance )
    {
        m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_MENU, "menu.rml", RocketMainMenuDocument::LoadDialog, RocketMainMenuDocument::UnloadDialog );
        if( !m_pInstance )
        {
            Error("Couldn't create rocketui menu!\n");
            /* Exit */
        }
    }
}

void RocketMainMenuDocument::UnloadDialog()
{
    if( m_pInstance )
    {
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