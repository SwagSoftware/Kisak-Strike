#ifndef KISAKSTRIKE_RKHUD_ROUNDTIMER_H
#define KISAKSTRIKE_RKHUD_ROUNDTIMER_H

#include <rocketui/rocketui.h>
#include "hudelement.h"

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core/DataModel.h"

extern ConVar cl_drawhud;

class RkHudRoundTimer : public CHudElement {
public:
    explicit RkHudRoundTimer( const char *value );
    virtual ~RkHudRoundTimer();

    // Overrides from CHudElement
    void LevelInit(void);
    virtual void LevelShutdown(void);
    virtual void SetActive(bool bActive);
    virtual bool ShouldDraw(void);
    void ShowPanel(bool bShow, bool force);

    Rml::ElementDocument *m_pInstance;
    Rml::DataModelHandle m_dataModel;
    bool m_bVisible;
};
#endif //KISAKSTRIKE_RKHUD_ROUNDTIMER_H
