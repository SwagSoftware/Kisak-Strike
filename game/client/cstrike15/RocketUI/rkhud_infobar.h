#ifndef KISAKSTRIKE_RKHUD_INFOBAR_H
#define KISAKSTRIKE_RKHUD_INFOBAR_H

#include <rocketui/rocketui.h>
#include "hudelement.h"

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core/DataModel.h"

extern ConVar cl_drawhud;

class RkHudInfoBar : public CHudElement {
public:
    explicit RkHudInfoBar(const char *value);
    virtual ~RkHudInfoBar();

    // Overrides from CHudElement
    void LevelInit(void);
    virtual void LevelShutdown(void);
    virtual void SetActive(bool bActive);
    virtual bool ShouldDraw(void);
    void ShowPanel(bool bShow, bool force);

    Rml::ElementDocument *m_pInstance;
    bool		m_bVisible;
    Rml::DataModelHandle m_dataModel;

};


#endif //KISAKSTRIKE_RKHUD_INFOBAR_H