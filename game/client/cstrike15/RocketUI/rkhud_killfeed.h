#ifndef KISAKSTRIKE_RKHUD_KILLFEED_H
#define KISAKSTRIKE_RKHUD_KILLFEED_H

#include <rocketui/rocketui.h>
#include "hudelement.h"

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core/DataModel.h"

extern ConVar cl_drawhud;

class RkHudKillfeed : public CHudElement
{
public:
    explicit RkHudKillfeed( const char *value );
    virtual ~RkHudKillfeed();

    // Overrides from CHudElement
    void LevelInit(void);
    virtual void LevelShutdown(void);
    virtual void SetActive(bool bActive);
    virtual bool ShouldDraw(void);
    void ShowPanel(bool bShow, bool force);

    // CGameEventListener
    virtual void FireGameEvent( IGameEvent *event );


    Rml::ElementDocument *m_pInstance;
    bool m_bVisible;
    Rml::DataModelHandle m_dataModel;

private:
    void CheckForOldEntries();
    void OnPlayerDeath( IGameEvent *event );
};

#endif //KISAKSTRIKE_RKHUD_KILLFEED_H
