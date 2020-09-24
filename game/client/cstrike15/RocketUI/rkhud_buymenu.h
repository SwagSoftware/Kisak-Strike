#ifndef KISAKSTRIKE_RKHUD_BUYMENU_H
#define KISAKSTRIKE_RKHUD_BUYMENU_H

#include <rocketui/rocketui.h>
#include "hudelement.h"

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core/DataModel.h"

extern ConVar cl_drawhud;

class RkHudBuyMenu : public CHudElement
{
public:
    explicit RkHudBuyMenu( const char * value );
    virtual ~RkHudBuyMenu();

    // Overrides from CHudElement
    void LevelInit(void);
    virtual void LevelShutdown(void);
    virtual void SetActive(bool bActive);
    virtual bool ShouldDraw(void);
    void ShowPanel(bool bShow, bool force);

    // CGameEventListener
    virtual void FireGameEvent( IGameEvent *event );

    Rml::ElementDocument *m_pInstance;
    bool		m_bVisible;
    bool        m_bGrabbingInput;
    Rml::DataModelHandle m_dataModel;

    void UpdateBuyMenu();
    void OnNewFrameBuyMenu();
};

#endif //KISAKSTRIKE_RKHUD_BUYMENU_H
