#if !defined( __ROCKETUI_H__ )
#define __ROCKETUI_H__

#include "appframework/iappsystem.h"
#include "shaderapi/IShaderDevice.h"
#include "inputsystem/InputEnums.h"

#define ROCKETUI_INTERFACE_VERSION "RocketUI001"

namespace Rml
{
    class Element;
    class ElementDocument;
    class Context;
    class EventListener;
}

enum RocketDesinationContext_t
{
    ROCKET_CONTEXT_MENU,
    ROCKET_CONTEXT_HUD,
    ROCKET_CONTEXT_CURRENT, // whichever context is active.
};

inline IRocketUI* RocketUI()
{
    extern IRocketUI* g_pRocketUI;
    return g_pRocketUI;
}

typedef void ( *LoadDocumentFn )( void );
typedef void ( *UnloadDocumentFn )( void );
typedef void ( *TogglePauseMenuFn )( void );

class IRocketUI: public IAppSystem
{
public:
    // Updates time mostly
    virtual void RunFrame( float time ) = 0;
    // Reload from Disk
    virtual bool ReloadDocuments() = 0;

    // Feed input into UI
    virtual bool HandleInputEvent( const InputEvent_t &event ) = 0;

    // Start consuming inputs, tell us why so we can debug input layers/leaks
    virtual void DenyInputToGame( bool value, const char *why = "Unknown" ) = 0;
    virtual bool IsConsumingInput( void ) = 0;

    virtual void EnableCursor( bool state ) = 0;

    // Document manipulation
    // The Load/Unload functions are for hot-reloading, they will be called on rocket_reload.
    // If you want reloading to work with your element, they must be set.
    virtual Rml::ElementDocument *LoadDocumentFile( RocketDesinationContext_t ctx, const char *filepath,
            LoadDocumentFn loadDocumentFunc = nullptr, UnloadDocumentFn unloadDocumentFunc = nullptr ) = 0;

    // The actual rendering
    virtual void RenderHUDFrame() = 0;
    virtual void RenderMenuFrame() = 0;

    // Access to the actual contexts in case you need something specific like data-bindings.
    virtual Rml::Context* AccessHudContext() = 0;
    virtual Rml::Context* AccessMenuContext() = 0;

    // The pause menu will register itself here so the library can open/close it via ESC
    virtual void RegisterPauseMenu( TogglePauseMenuFn showPauseMenuFunc ) = 0;

    virtual void AddDeviceDependentObject( IShaderDeviceDependentObject * pObject ) = 0;
    virtual void RemoveDeviceDependentObject( IShaderDeviceDependentObject * pObject ) = 0;
};
























#endif