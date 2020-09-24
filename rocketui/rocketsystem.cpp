#include "rocketsystem.h"

#include "rocketuiimpl.h"

#include "vgui/ISystem.h"

RocketSystem RocketSystem::m_Instance;

double RocketSystem::GetElapsedTime()
{
    return (double)RocketUIImpl::m_Instance.GetTime();
}

bool RocketSystem::LogMessage(Rml::Log::Type type, const Rml::String &message)
{
    bool ret = false;
    if( type == Rml::Log::LT_ERROR )
        ret = true;

    //FIXME: an actual logging function that shows up in the terminal. Source engine has like 20+
    fprintf( stderr, "[RocketUI]%s\n", message.c_str() );

    return ret;
}

void RocketSystem::SetClipboardText(const Rml::String& text)
{
    g_pVGuiSystem->SetClipboardText( text.c_str(), text.size() );
}

void RocketSystem::GetClipboardText(Rml::String& text)
{
    char buffer[1024];
    g_pVGuiSystem->GetClipboardText(0, buffer, sizeof(buffer) );
    text.copy(buffer, sizeof(buffer));
}