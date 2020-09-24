#ifndef KISAKSTRIKE_RKMENU_MAIN_H
#define KISAKSTRIKE_RKMENU_MAIN_H

#include <rocketui/rocketui.h>

class RocketMainMenuDocument
{
protected:
    static Rml::ElementDocument *m_pInstance;

    RocketMainMenuDocument( );
public:
    static void LoadDialog( void );
    static void UnloadDialog( void );
    static void RestorePanel( void );
    static void ShowPanel( bool bShow, bool immediate = false );
    static bool IsActive() { return m_pInstance != nullptr; }
    static bool IsVisible() { return showing; }
    static void UpdateDialog( void );
    static Rml::ElementDocument *GetInstance() { return m_pInstance; }
private:
    static bool showing;
    static bool grabbingInput;
};

#endif //KISAKSTRIKE_RKMENU_MAIN_H