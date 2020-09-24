#ifndef KISAKSTRIKE_RKHUD_SCOREBOARD_H
#define KISAKSTRIKE_RKHUD_SCOREBOARD_H

#include <rocketui/rocketui.h>
#include "hudelement.h"
#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core/DataModel.h"

extern ConVar cl_drawhud;

class RkHudScoreboard : public CHudElement {
public:
    explicit RkHudScoreboard(const char *value);
    virtual ~RkHudScoreboard();

    // Overrides from CHudElement
    void LevelInit(void);
    virtual void LevelShutdown(void);
    virtual void SetActive(bool bActive);
    virtual bool ShouldDraw(void);
    void ShowPanel(bool bShow, bool force);

    Rml::ElementDocument *m_pInstance;
    // Some precached elements from the instance.
    Rml::Element *m_elemCtSection;
    Rml::Element *m_elemTSection;
    Rml::DataModelHandle m_dataModel;

    bool		m_bVisible;

private:
    void Update();
};

#endif //KISAKSTRIKE_RKHUD_SCOREBOARD_H
