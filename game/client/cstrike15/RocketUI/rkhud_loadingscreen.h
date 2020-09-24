#ifndef KISAKSTRIKE_RKHUD_LOADINGSCREEN_H
#define KISAKSTRIKE_RKHUD_LOADINGSCREEN_H

#include <rocketui/rocketui.h>

class RocketLoadingScreenDocument
{
protected:
    static Rml::ElementDocument *m_pInstance;

    RocketLoadingScreenDocument( );
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
};


#endif //KISAKSTRIKE_RKHUD_LOADINGSCREEN_H
