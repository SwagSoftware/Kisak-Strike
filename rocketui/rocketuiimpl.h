#ifndef KISAKSTRIKE_ROCKETUI_H
#define KISAKSTRIKE_ROCKETUI_H

#if defined( USE_SDL ) || defined( OSX )
#include "appframework/ilaunchermgr.h"
#endif

#if defined ( DX_TO_GL_ABSTRACTION )
#include "togl/rendermechanism.h"
#endif

#include "rocketui/rocketui.h"
#include "tier3/tier3.h"
#include "shaderapi/IShaderDevice.h"
#include "shaderapi/ishaderapi.h"
#include "IGameUIFuncs.h"
#include "cdll_int.h"
#include "igameevents.h"
#include "tier1/utlpair.h"

#include "rocketrender.h"

#include "RmlUi/Core/ElementDocument.h"

class DeviceCallbacks;
class RocketUIImpl : public CTier3AppSystem<IRocketUI>
{
    typedef CTier3AppSystem<IRocketUI> BaseClass;

    /** singleton support **/
public:
    static RocketUIImpl m_Instance;

protected:
    IDirect3DDevice9    *m_pDevice;
    DeviceCallbacks     *m_pDeviceCallbacks;

#if defined( USE_SDL ) || defined( OSX )
    ILauncherMgr        *m_pLauncherMgr;
#endif
    IShaderDeviceMgr    *m_pShaderDeviceMgr;
    IShaderAPI		    *m_pShaderAPI;
    IGameUIFuncs        *m_pGameUIFuncs;
    IVEngineClient      *m_pEngine;
    IGameEventManager2  *m_pGameEventManager;

    // Fonts need to stay for the lifetime of the program. Used directly by freetype. Freed on shutdown.
    CUtlVector<unsigned char*>   m_fontAllocs;
    CUtlVector<CUtlString>       m_inputConsumers;
    float               m_fTime;
    bool                m_bCursorVisible;

    // Contexts
    Rml::Context        *m_ctxMenu;
    Rml::Context        *m_ctxHud;
    Rml::Context        *m_ctxCurrent; //Pointer to Hud or Menu

    bool                m_isDebuggerOpen;

    TogglePauseMenuFn     m_togglePauseMenuFunc;

    // List of Document Reload functions for hot-reloading.
    CUtlVector< CUtlPair<LoadDocumentFn, UnloadDocumentFn> > m_documentReloadFuncs;

    // if > 0, we are stealing input from the game.
    int                 m_numInputConsumers;
    /** IAppSystem **/
public:
    virtual bool Connect( CreateInterfaceFn factory );
    virtual void Disconnect( void );

    // Here's where systems can access other interfaces implemented by this m_pObject
    // Returns NULL if it doesn't implement the requested interface
    virtual void *QueryInterface( const char *pInterfaceName );

    // Init, shutdown
    virtual InitReturnVal_t Init( void );
    virtual void Shutdown( void );

    // Returns all dependent libraries
    virtual const AppSystemInfo_t* GetDependencies( void );

    // Returns the tier
    virtual AppSystemTier_t GetTier( void )
    {
        return APP_SYSTEM_TIER3;
    }

    // Reconnect to a particular interface
    virtual void Reconnect( CreateInterfaceFn factory, const char *pInterfaceName )
    {
        BaseClass::Reconnect( factory, pInterfaceName );
    }

    /** IRocketUI Interface Methods **/
public:
    // Updates time mostly
    virtual void RunFrame( float time );

    // Reload from Disk
    virtual bool ReloadDocuments();

    // Feed input into UI
    virtual bool HandleInputEvent( const InputEvent_t &event );
    // Start consuming inputs
    virtual void DenyInputToGame( bool value, const char *why );
    virtual bool IsConsumingInput( void );

    virtual void EnableCursor( bool state );

    // Document manipulation.
    virtual Rml::ElementDocument *LoadDocumentFile( RocketDesinationContext_t ctx, const char *filepath,
                                                    LoadDocumentFn loadDocumentFunc = nullptr, UnloadDocumentFn unloadDocumentFunc = nullptr );

    // Rendering
    virtual void RenderHUDFrame();
    virtual void RenderMenuFrame();

    // Access to the actual contexts.
    virtual Rml::Context* AccessHudContext();
    virtual Rml::Context* AccessMenuContext();

    virtual void RegisterPauseMenu( TogglePauseMenuFn showPauseMenuFunc )
    {
        m_togglePauseMenuFunc = showPauseMenuFunc;
    }

    void AddDeviceDependentObject( IShaderDeviceDependentObject * pObject );
    void RemoveDeviceDependentObject( IShaderDeviceDependentObject * pObject );

    /** Local Class Methods **/
    RocketUIImpl( void );
    void SetRenderingDevice( IDirect3DDevice9 *pDevice, D3DPRESENT_PARAMETERS *pPresentParameters, HWND hWnd );
    void ToggleDebugger( void );
    Rml::Context *GetCurrentContext()
    {
        return m_ctxCurrent;
    }
private:
    bool LoadFont( const char *filepath, const char *path );
    bool LoadFonts();

public:
    inline const float GetTime()
    {
        return m_fTime;
    }
    void SetScreenSize( int width, int height );
};

class DeviceCallbacks: public IShaderDeviceDependentObject
{
public:
    int m_iRefCount;
    RocketUIImpl* m_pRocketUI;

    DeviceCallbacks( void ) :
            m_iRefCount( 1 ), m_pRocketUI( NULL )
    {
    }

    virtual void DeviceLost( void )
    {
        //TODO: Windows version wants this
        //m_pScaleform->NotifyRenderingDeviceLost();
    }

    virtual void DeviceReset( void *pDevice, void *pPresentParameters, void *pHWnd )
    {
        m_pRocketUI->SetRenderingDevice( ( IDirect3DDevice9* )pDevice, ( D3DPRESENT_PARAMETERS* )pPresentParameters, ( HWND )pHWnd );
    }

    virtual void ScreenSizeChanged( int width, int height )
    {
        m_pRocketUI->SetScreenSize( width, height );
    }
};




#endif //KISAKSTRIKE_ROCKETUI_H