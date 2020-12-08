#ifndef KISAKSTRIKE_RKPANEL_POPUP_H
#define KISAKSTRIKE_RKPANEL_POPUP_H

#include <rocketui/rocketui.h>

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core/EventListener.h"

#define MAX_POPUP_TEXT 256

class RocketPopupDocument : Rml::EventListener
{
public:
    RocketPopupDocument( const char *text, float timeout = 5.0f );
    virtual ~RocketPopupDocument();
    bool IsActive() { return m_pInstance != nullptr; }
    bool IsVisible() { return m_bVisible; }
    Rml::ElementDocument *GetInstance() { return m_pInstance; }

    void ProcessEvent( Rml::Event &event ) override;
private:
    Rml::ElementDocument *m_pInstance;
    char m_textBuffer[MAX_POPUP_TEXT];

    float m_fTimeout;
    bool m_bVisible;
    bool m_bGrabbingInput;
};

#endif //KISAKSTRIKE_RKPANEL_POPUP_H
