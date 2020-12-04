#include "rocketuiimpl.h"

#ifdef Debugger
#undef Debugger
#endif

#include "rocketsystem.h"
#include "rocketrender.h"
#include "rocketfilesystem.h"

#include <RmlUi/Core.h>
#include <RmlUi/Debugger.h>

#include "rocketkeys.h"
#include "customelements/KisakConvarSetting.h"
#include "customelements/KisakURL.h"

RocketUIImpl RocketUIImpl::m_Instance;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR( RocketUIImpl, IRocketUI, ROCKETUI_INTERFACE_VERSION, RocketUIImpl::m_Instance )

ConVar rocket_enable( "rocket_enable", "1", 0, "Enables RocketUI" );
ConVar rocket_verbose( "rocket_verbose", "0", 0, "Enables more logging" );

CON_COMMAND_F( rocket_reload, "Reloads all RocketUI Documents", FCVAR_NONE )
{
    if( RocketUIImpl::m_Instance.ReloadDocuments() )
    {
        ConMsg("[RocketUI]Documents Reloaded.\n");
    }
    else
    {
        ConMsg("[RocketUI]Error reloading Documents!\n");
    }
}

CON_COMMAND_F( rocket_debug, "Open/Close the RocketUI Debugger", FCVAR_NONE )
{
    RocketUIImpl::m_Instance.ToggleDebugger();
}



RocketUIImpl::RocketUIImpl() { }

bool RocketUIImpl::Connect(CreateInterfaceFn factory)
{
    if ( !factory )
    {
        return false;
    }

    if ( !BaseClass::Connect( factory ) )
    {
        return false;
    }

#if defined( USE_SDL )
    m_pLauncherMgr = (ILauncherMgr *)factory( SDLMGR_INTERFACE_VERSION, NULL);
#elif defined( OSX )
    m_pLauncherMgr = (ILauncherMgr *)factory( COCOAMGR_INTERFACE_VERSION, NULL);
#else
    #error fixme
#endif

    m_pShaderDeviceMgr = ( IShaderDeviceMgr* ) factory( SHADER_DEVICE_MGR_INTERFACE_VERSION, NULL );
    m_pGameUIFuncs = ( IGameUIFuncs* ) factory( VENGINE_GAMEUIFUNCS_VERSION, NULL );
    m_pEngine = ( IVEngineClient* )factory( VENGINE_CLIENT_INTERFACE_VERSION, NULL );
    m_pGameEventManager = ( IGameEventManager2* )factory ( INTERFACEVERSION_GAMEEVENTSMANAGER2, NULL );
    m_pShaderAPI = ( IShaderAPI * )factory( SHADERAPI_INTERFACE_VERSION, NULL );

    if ( !m_pShaderDeviceMgr || !m_pGameUIFuncs || !m_pEngine || !m_pGameEventManager || !m_pShaderAPI )
    {
        Warning( "RocketUI: missing expected interface\n" );
        return false;
    }

    return true;
}

void RocketUIImpl::Disconnect()
{
    if ( m_pShaderDeviceMgr )
    {
        if ( m_pDeviceCallbacks )
        {
            m_pShaderDeviceMgr->RemoveDeviceDependentObject( m_pDeviceCallbacks );
            delete m_pDeviceCallbacks;
            m_pDeviceCallbacks = NULL;
        }
    }
#if defined( USE_SDL )
    m_pLauncherMgr = NULL;
#elif defined( OSX )
    m_pLauncherMgr = NULL;
#else
    #error fixme
#endif
    m_pShaderDeviceMgr = NULL;
    m_pGameUIFuncs = NULL;
    m_pEngine = NULL;
    m_pGameEventManager = NULL;
    m_pShaderAPI = NULL;

    BaseClass::Disconnect();
}

void* RocketUIImpl::QueryInterface( const char *pInterfaceName )
{
    if ( !Q_strncmp( pInterfaceName, ROCKETUI_INTERFACE_VERSION, Q_strlen( ROCKETUI_INTERFACE_VERSION ) + 1 ) )
    {
        return ( IRocketUI* ) &RocketUIImpl::m_Instance;
    }

    return BaseClass::QueryInterface( pInterfaceName );
}

const AppSystemInfo_t* RocketUIImpl::GetDependencies( void )
{
    return BaseClass::GetDependencies();
}

Rml::Context* RocketUIImpl::AccessHudContext()
{
    return m_ctxHud;
}

Rml::Context* RocketUIImpl::AccessMenuContext()
{
    return m_ctxMenu;
}
bool RocketUIImpl::LoadFont( const char *filepath, const char *path )
{
    unsigned char *fontBuffer = NULL;
    CUtlBuffer font;
    unsigned int fontLen;

    if( !g_pFullFileSystem->ReadFile( filepath, path, font ) )
    {
        fprintf(stderr, "[RocketUI]Failed to read %s font.", filepath );
        return false;
    }

    fontLen = font.Size() - 1;

    if( fontLen >= ( 8 * 1024 * 1024 ) )
    {
        fprintf(stderr, "[RocketUI]Font (%s) is over 8MB!(%d). Not Loading.\n", filepath, fontLen );
        return false;
    }

    fprintf(stderr, "[RocketUI]Font size (%d)\n", fontLen );

    fontBuffer = new unsigned char[ fontLen + 1 ];
    // Add to list of alloc'd fonts. Freetype will use this memory until we Shutdown.
    m_fontAllocs.AddToTail( fontBuffer );

    font.Get( fontBuffer, fontLen );

    if( !Rml::LoadFontFace( fontBuffer, fontLen, "Lato", Rml::Style::FontStyle::Normal, Rml::Style::FontWeight::Normal, false ) )
    {
        fprintf(stderr,  "[RocketUI]Failed to Initialize Lato font\n" );
        return false;
    }

    return true;
}

bool RocketUIImpl::LoadFonts()
{
    bool fontsOK = true;
    fontsOK &= LoadFont( "rocketui/fonts/Lato-Black.ttf", "GAME" );

    return fontsOK;
}

Rml::ElementDocument *RocketUIImpl::LoadDocumentFile(RocketDesinationContext_t ctx, const char *filepath,
        LoadDocumentFn loadDocumentFunc, UnloadDocumentFn unloadDocumentFunc)
{
    Rml::ElementDocument *document;
    Rml::Context *destinationCtx;

    switch( ctx )
    {
        case ROCKET_CONTEXT_MENU:
            destinationCtx = m_ctxMenu;
            break;
        case ROCKET_CONTEXT_HUD:
            destinationCtx = m_ctxHud;
            break;
        case ROCKET_CONTEXT_CURRENT:
            if( !m_ctxCurrent )
            {
                Error("Couldn't load document: %s - loaded before 1st frame.\n", filepath );
                return nullptr;
            }
            destinationCtx = m_ctxCurrent;
            break;
        default:
            return nullptr;
    }

    document = destinationCtx->LoadDocument( filepath );

    // Need both
    if( loadDocumentFunc && unloadDocumentFunc )
    {
        CUtlPair<LoadDocumentFn, UnloadDocumentFn> documentFuncPair( loadDocumentFunc, unloadDocumentFunc );
        m_documentReloadFuncs.AddToTail( documentFuncPair );
    }

    return document;
}

InitReturnVal_t RocketUIImpl::Init( void )
{
    InitReturnVal_t nRetVal = BaseClass::Init();
    if ( nRetVal != INIT_OK )
    {
        return nRetVal;
    }

    // Register a callback with the ShaderDeviceMgr
    m_pDeviceCallbacks = new DeviceCallbacks();
    m_pDeviceCallbacks->m_pRocketUI = this;
    m_pShaderDeviceMgr->AddDeviceDependentObject( m_pDeviceCallbacks );

    // Create/Init the Rocket UI Library
    // Default width/height, these get updated in the DeviceCallbacks
    int width = 1920;
    int height = 1080;
    RocketRender::m_Instance.SetScreenSize( width, height );
    RocketRender::m_Instance.SetContext( m_pLauncherMgr->GetMainContext() );

    // Allocate and store system cursors so we can swap to them on the fly
    RocketSystem::m_Instance.InitCursors();

    Rml::SetFileInterface( &RocketFileSystem::m_Instance );
    Rml::SetRenderInterface( &RocketRender::m_Instance );
    Rml::SetSystemInterface( &RocketSystem::m_Instance );

    if ( !Rml::Initialise() )
    {
        Warning( "RocketUI: Initialise() failed!\n");
        return INIT_FAILED;
    }

    if( !LoadFonts() )
    {
        Warning( "RocketUI: Failed to load fonts.\n" );
        return INIT_FAILED;
    }

    m_ctxMenu = Rml::CreateContext("menu", Rml::Vector2i(width, height));
    m_ctxHud = Rml::CreateContext("hud", Rml::Vector2i(width, height));

    if ( !m_ctxMenu || !m_ctxHud )
    {
        Warning( "RocketUI: Failed to create Hud/Menu context\n" );
        Rml::Shutdown();
        return INIT_FAILED;
    }

    m_ctxMenu->SetDensityIndependentPixelRatio(1.0f );
    m_ctxHud->SetDensityIndependentPixelRatio(1.0f );

    // Register Custom Elements with the factory.
    Rml::Factory::RegisterElementInstancer( "KisakConvarSettingEnum", new KisakConvarSettingEnumInstancer() );
    Rml::Factory::RegisterElementInstancer( "KisakConvarSettingSlider", new KisakConvarSettingSliderInstancer() );
    Rml::Factory::RegisterElementInstancer( "KisakURL", new KisakURLInstancer() );
    return INIT_OK;
}

void RocketUIImpl::Shutdown()
{
    // Shutdown RocketUI. All contexts are destroyed on shutdown.
    Rml::Shutdown();

    // freetype FT_Done_Face has been called. Time to free fonts.
    for( int i = 0; i < m_fontAllocs.Count(); i++ )
    {
        unsigned char *fontAlloc = m_fontAllocs[i];
        delete[] fontAlloc;
    }

    // Free Cursors
    RocketSystem::m_Instance.FreeCursors();

    if ( m_pShaderDeviceMgr )
    {
        if ( m_pDeviceCallbacks )
        {
            m_pShaderDeviceMgr->RemoveDeviceDependentObject( m_pDeviceCallbacks );
            delete m_pDeviceCallbacks;
            m_pDeviceCallbacks = NULL;
        }
    }

    m_ctxCurrent = NULL;

    BaseClass::Shutdown();
}

void RocketUIImpl::RunFrame(float time)
{
    // We dont have the device yet..
    if( !m_pDevice )
        return;

    m_fTime = time;

    // This is important. Update the current context 1x per frame.
    // This basically needs to be called whenever elements are added/changed/removed
    // I am calling it 1x per frame here instead of all over the place for simplicity and no overlap.
    if( m_ctxCurrent )
        m_ctxCurrent->Update();
}

void RocketUIImpl::DenyInputToGame( bool value, const char *why )
{
    if( value )
    {
        m_numInputConsumers++;
        m_inputConsumers.AddToTail( CUtlString( why ) );
    }
    else
    {
        m_numInputConsumers--;
        m_inputConsumers.FindAndRemove( CUtlString( why ) );
    }

    EnableCursor( (m_numInputConsumers > 0) );

    if( rocket_verbose.GetBool() )
    {
        ConMsg("input Consumers[%d]: ", m_numInputConsumers);
        for( int i = 0; i < m_inputConsumers.Count(); i++ )
        {
            ConMsg("(%s) ", m_inputConsumers[i].Get() );
        }
        ConMsg("\n");
    }
}

bool RocketUIImpl::IsConsumingInput()
{
    return ( m_numInputConsumers > 0 );
}

void RocketUIImpl::EnableCursor(bool state)
{
    ConVarRef cl_mouseenable( "cl_mouseenable" );

    cl_mouseenable.SetValue( !state );

    if( state )
        m_pLauncherMgr->ForceSystemCursorVisible();
    else
        m_pLauncherMgr->UnforceSystemCursorVisible();

    m_bCursorVisible = state;
}

// This function is an input hook.
// return true if we want to deny the game the input.
bool RocketUIImpl::HandleInputEvent(const InputEvent_t &event)
{
    // Haven't rendered our very first frame ever yet.
    if( !m_ctxCurrent )
        return false;

    if( !rocket_enable.GetBool() )
        return false;

    // Always get the mouse location.
    if( event.m_nType == IE_AnalogValueChanged && event.m_nData == MOUSE_XY )
    {
        // TODO update this with keymodifiers
        m_ctxCurrent->ProcessMouseMove( event.m_nData2, event.m_nData3, 0 );
    }

    // Some edge cases
    if( event.m_nType == IE_ButtonPressed )
    {
        // Check for debugger. Toggle on F8.
        if( event.m_nData == KEY_F8 )
        {
            ToggleDebugger();
            return true;
        }
        // The magical ESC key for the pause menu. The game handles this in an awful way
        // CSGO will open the pause menu for us the 1st time, but after that it fubars
        // In order to minimize this component from reaching into the gamecode,
        // The pause menu will register itself via RegisterPauseMenu while loading.
        // Kinda Hacky, but it is direct from keys.cpp and prevents the VGUI code from messing with it too much.
        if( event.m_nData == KEY_ESCAPE )
        {
            if( m_togglePauseMenuFunc && m_pEngine->IsInGame() )
            {
                m_togglePauseMenuFunc();
            }
        }
    }

    // Nothing wants input, skip.
    if( !IsConsumingInput() )
        return false;

    // The console is open, skip
    if( m_pEngine->Con_IsVisible() )
        return false;

    Rml::Input::KeyIdentifier key;
    char ascii;

    switch( event.m_nType )
    {
        case IE_ButtonDoubleClicked:
        case IE_ButtonPressed:
            //TODO add key modifiers
            if( IsMouseCode( (ButtonCode_t)event.m_nData ) )
            {
                switch( (ButtonCode_t)event.m_nData )
                {
                    case MOUSE_LEFT:
                        m_ctxCurrent->ProcessMouseButtonDown( 0, 0 );
                        break;
                    case MOUSE_RIGHT:
                        m_ctxCurrent->ProcessMouseButtonDown( 1, 0 );
                        break;
                    case MOUSE_MIDDLE:
                        m_ctxCurrent->ProcessMouseButtonDown( 2, 0 );
                        break;
                    case MOUSE_4:
                        m_ctxCurrent->ProcessMouseButtonDown( 3, 0 );
                        break;
                    case MOUSE_5:
                        m_ctxCurrent->ProcessMouseButtonDown( 4, 0 );
                        break;
                    case MOUSE_WHEEL_UP:
                        m_ctxCurrent->ProcessMouseWheel( -1, 0 );
                        break;
                    case MOUSE_WHEEL_DOWN:
                        m_ctxCurrent->ProcessMouseWheel( 1, 0 );
                        break;
                }
            }
            else
            {
                m_ctxCurrent->ProcessKeyDown( ButtonToRocketKey( (ButtonCode_t)event.m_nData ), 0 );
            }
            break;
        case IE_ButtonReleased:
            //TODO add key modifiers
            if( IsMouseCode( (ButtonCode_t)event.m_nData ) )
            {
                switch( (ButtonCode_t)event.m_nData )
                {
                    case MOUSE_LEFT:
                        m_ctxCurrent->ProcessMouseButtonUp( 0, 0 );
                        break;
                    case MOUSE_RIGHT:
                        m_ctxCurrent->ProcessMouseButtonUp( 1, 0 );
                        break;
                    case MOUSE_MIDDLE:
                        m_ctxCurrent->ProcessMouseButtonUp( 2, 0 );
                        break;
                    case MOUSE_4:
                        m_ctxCurrent->ProcessMouseButtonUp( 3, 0 );
                        break;
                    case MOUSE_5:
                        m_ctxCurrent->ProcessMouseButtonUp( 4, 0 );
                        break;
                }
            }
            else
            {
                m_ctxCurrent->ProcessKeyUp( ButtonToRocketKey( (ButtonCode_t)event.m_nData ), 0 );
            }
            break;
        case IE_KeyTyped:
            ascii = (char)((wchar_t)event.m_nData);
            if( ascii != 8 ){ // Rocketui doesn't like the backspace for some reason.
                m_ctxCurrent->ProcessTextInput( ascii );
            }
            break;
        case IE_AnalogValueChanged:
            // Mouse/Joystick changes. Mouse changes are recorded above
            break;

        default:
            return false;
    }

    return IsConsumingInput();
}

void RocketUIImpl::RenderHUDFrame()
{
    if( !rocket_enable.GetBool() )
        return;

    m_ctxCurrent = m_ctxHud;

#if defined ( DX_TO_GL_ABSTRACTION )
    m_pDevice->SaveGLState();
    RocketRender::m_Instance.PrepareGLState();
#endif

    //CMatRenderContextPtr pRenderContext( g_pMaterialSystem );
    //ShaderStencilState_t state;

    //// Setup the Matrix/Ortho
    //StartDrawing();
//
    //// Clear z + stencil buffer
    //pRenderContext->ClearBuffers( false, true, true );
//
    //state.m_bEnable = true;
    //state.m_FailOp = SHADER_STENCILOP_KEEP;
    //state.m_ZFailOp = SHADER_STENCILOP_KEEP;
    //state.m_PassOp = SHADER_STENCILOP_SET_TO_REFERENCE;
    //state.m_CompareFunc = SHADER_STENCILFUNC_GEQUAL;
    //state.m_nReferenceValue = 0;
    //state.m_nTestMask = 0xFFFFFFFF;
    //state.m_nWriteMask = 0xFFFFFFFF;
    //pRenderContext->SetStencilState( state );

    //TODO: don't update here. update only after input or new elements
    //m_ctxHud->Update();
    //m_ctxMenu->Update();

    m_ctxHud->Render();
    //m_ctxMenu->Render();

    // Reset stencil to normal
    //state.m_bEnable = false;
    //pRenderContext->SetStencilState( state );
//
    //FinishDrawing();

#if defined ( DX_TO_GL_ABSTRACTION )
    m_pDevice->RestoreGLState();
#endif
}

void RocketUIImpl::RenderMenuFrame()
{
    if( !rocket_enable.GetBool() )
        return;

    m_ctxCurrent = m_ctxMenu;

#if defined ( DX_TO_GL_ABSTRACTION )
    m_pDevice->SaveGLState();
    RocketRender::m_Instance.PrepareGLState();
#endif

    //TODO: don't update here. update only after input or new elements
    //m_ctxMenu->Update();

    m_ctxMenu->Render();

#if defined ( DX_TO_GL_ABSTRACTION )
    m_pDevice->RestoreGLState();
#endif
}

bool RocketUIImpl::ReloadDocuments()
{
    rocket_enable.SetValue( false );
    // Hacky, sleep for 100ms after disabling UI.
    // I dont feel like adding a mutex check every frame for something rarely used by devs
    ThreadSleep( 100 );

    CUtlVector<CUtlPair<LoadDocumentFn, UnloadDocumentFn>> copyOfPairs;

    // Copy the pairs into a local Vector( grug, copy constructor no work )
    // We want a copy because the loading functions will mess with our Vector when we call them.
    for( int i = 0; i < m_documentReloadFuncs.Count(); i++ )
    {
        copyOfPairs.AddToTail( m_documentReloadFuncs[i] );
    }

    // We can now empty the Main Vector since we are about to reload.
    m_documentReloadFuncs.Purge();

    // Go through the copy and reload
    for( int i = 0; i < copyOfPairs.Count(); i++ )
    {
        CUtlPair<LoadDocumentFn, UnloadDocumentFn> documentPair( copyOfPairs[i] );
        // Unload...
        documentPair.second();
        // Load...
        documentPair.first();
    }

    rocket_enable.SetValue( true );
    return true;
}

void RocketUIImpl::AddDeviceDependentObject(IShaderDeviceDependentObject *pObject)
{
    if( m_pShaderDeviceMgr )
    {
        m_pShaderDeviceMgr->AddDeviceDependentObject( pObject );
    }
}

void RocketUIImpl::RemoveDeviceDependentObject(IShaderDeviceDependentObject *pObject)
{
    if( m_pShaderDeviceMgr )
    {
        m_pShaderDeviceMgr->RemoveDeviceDependentObject( pObject );
    }
}

void RocketUIImpl::SetRenderingDevice(IDirect3DDevice9 *pDevice, D3DPRESENT_PARAMETERS *pPresentParameters, HWND hWnd)
{
    // This is how pDevice gets initialized :)
    if( !pDevice || m_pDevice != pDevice )
    {
        m_pDevice = pDevice;
    }

    //TODO: Do we need to reset rocketui here?

    D3DVIEWPORT9 viewport;
    pDevice->GetViewport( &viewport );
    SetScreenSize( viewport.Width, viewport.Height );
}

void RocketUIImpl::SetScreenSize(int width, int height)
{
    m_ctxCurrent = nullptr;

    m_ctxHud->SetDimensions( Rml::Vector2i( width, height ) );
    m_ctxMenu->SetDimensions( Rml::Vector2i( width, height ) );

    RocketRender::m_Instance.SetScreenSize( width, height );
}

void RocketUIImpl::ToggleDebugger()
{
    static bool open = false;
    static bool firstTime = true;

    open = !open;

    if( !m_ctxCurrent )
        return;

    if( open )
    {
        if( firstTime )
        {
            if( Rml::Debugger::Initialise( m_ctxCurrent ) )
            {
                firstTime = false;
            }
            else
            {
                ConMsg("[RocketUI]Error Initializing Debugger\n");
                return;
            }
        }
        ConMsg("[RocketUI]Opening Debugger\n");
        if( !Rml::Debugger::SetContext( m_ctxCurrent ) )
        {
            ConMsg("[RocketUI]Error setting context!\n");
            return;
        }
        m_isDebuggerOpen = true;
        Rml::Debugger::SetVisible( true );
        DenyInputToGame( true, "RocketUI Debugger" );
    }
    else
    {
        ConMsg("[RocketUI]Closing Debugger\n");
        Rml::Debugger::SetVisible( false );
        m_isDebuggerOpen = false;
        DenyInputToGame( false, "RocketUI Debugger" );
    }
}