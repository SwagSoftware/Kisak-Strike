#ifndef KISAKSTRIKE_RKHUD_PAUSEMENU_H
#define KISAKSTRIKE_RKHUD_PAUSEMENU_H

#include <rocketui/rocketui.h>

class RocketPauseMenuDocument
{
protected:
    static Rml::ElementDocument *m_pInstance;

    RocketPauseMenuDocument( );
public:
    static void LoadDialog( void );
    static void UnloadDialog( void );
    static void RestorePanel( void );
    static void TogglePanel( void );
    static void ShowPanel( bool bShow, bool immediate = false );
    static bool IsActive() { return m_pInstance != nullptr; }
    static bool IsVisible() { return m_bVisible; }
    static void UpdateDialog( void );
    static Rml::ElementDocument *GetInstance() { return m_pInstance; }
private:
    static bool m_bVisible;
    static bool m_bGrabbingInput;
};


#endif
