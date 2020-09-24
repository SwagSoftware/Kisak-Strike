#ifndef KISAKSTRIKE_RKHUD_TEAMMENU_H
#define KISAKSTRIKE_RKHUD_TEAMMENU_H

#include <rocketui/rocketui.h>
#include "GameEventListener.h"

class RocketTeamMenuEventListener : public CGameEventListener
{
public:
    virtual ~RocketTeamMenuEventListener();
    void StartAlwaysListenEvents();
    void StopAlwaysListenEvents();
    virtual void FireGameEvent( IGameEvent *event );
};

class RocketTeamMenuDocument
{
protected:
    static Rml::ElementDocument *m_pInstance;

    RocketTeamMenuDocument( );
    virtual ~RocketTeamMenuDocument();
public:
    static void LoadDialog( void );
    static void UnloadDialog( void );
    static void ShowPanel( bool bShow, bool immediate = false );
    static bool IsActive() { return m_pInstance != nullptr; }
    static bool IsVisible() { return m_bVisible; }
    static Rml::ElementDocument *GetInstance() { return m_pInstance; }

private:
    static bool m_bVisible;
    static bool m_bGrabbingInput;
    static RocketTeamMenuEventListener *m_pEventListener;
};

#endif //KISAKSTRIKE_RKHUD_TEAMMENU_H
