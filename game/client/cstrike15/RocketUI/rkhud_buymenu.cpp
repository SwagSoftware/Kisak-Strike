#include "rkhud_buymenu.h"

#include "cbase.h"
#include "hud_macros.h"
#include "c_cs_player.h"

#include <tier0/valve_minmax_off.h> // included to fix an error with min/max and rocketui
#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"

DECLARE_HUDELEMENT( RkHudBuyMenu );

struct ItemListEntry
{
    Rml::String itemName;
    int itemPrice;
};

// struct layout for data-binding model.
struct BuyMenuData
{
    int playerCash;
    int playerTeamNum;
    int buyTimeLeft; // seconds
    Rml::Vector<ItemListEntry> gearList;
    Rml::Vector<ItemListEntry> grenadeList;
    Rml::Vector<ItemListEntry> pistolList;
    Rml::Vector<ItemListEntry> rifleList;
    Rml::Vector<ItemListEntry> smgList;
    Rml::Vector<ItemListEntry> heavyList;
} buyMenuData;

class RkBuyMenuListener : public Rml::EventListener
{
public:
    void ProcessEvent(Rml::Event &keyevent) override
    {
        Rml::EventId eventType = keyevent.GetId();

        RkHudBuyMenu *pBuyMenu = GET_HUDELEMENT( RkHudBuyMenu );
        if( !pBuyMenu )
            return;

        if( eventType == Rml::EventId::Keyup )
        {
            C_BasePlayer *localPlayer = C_BasePlayer::GetLocalPlayer();
            IGameEvent *pEvent = gameeventmanager->CreateEvent( "buymenu_close" );
            if ( localPlayer && pEvent )
            {
                pEvent->SetInt("userid", localPlayer->GetUserID() );
                gameeventmanager->FireEventClientSide( pEvent );
            }
        }
        else if( eventType == Rml::EventId::Mousedown )
        {
            const Rml::Element *target = keyevent.GetTargetElement();
            if( target->IsClassSet( "item" ) )
            {
                // for some reason this shit wont work with "weapon_ak47", just "ak47"
                char buyBuffer[512];
                V_snprintf( buyBuffer, sizeof( buyBuffer ), "buy %s", ( V_strstr( target->GetChild(2)->GetInnerRML().c_str(), "_" ) + 1 ) );
                DevMsg("buying [%s]\n", buyBuffer );
                engine->ExecuteClientCmd( buyBuffer );
                pBuyMenu->UpdateBuyMenu();
            }
        }
    }
};

static RkBuyMenuListener buyMenuListener;

// Updates the buy menu options via the game's weapon database.
// Called sparingly when buymenu is opened/interacted.
void RkHudBuyMenu::UpdateBuyMenu()
{
    C_CSPlayer *localPlayer = C_CSPlayer::GetLocalCSPlayer();
    if( !localPlayer )
        return;

    buyMenuData.gearList.clear();
    buyMenuData.grenadeList.clear();
    buyMenuData.pistolList.clear();
    buyMenuData.rifleList.clear();
    buyMenuData.smgList.clear();
    buyMenuData.heavyList.clear();

    for( int i = WEAPON_FIRST ; i < WEAPON_MAX; i++ )
    {
        if( localPlayer->CanAcquire( (CSWeaponID)i, AcquireMethod::Buy ) != AcquireResult::Allowed )
            continue;

        const CCSWeaponInfo* info = GetWeaponInfo( (CSWeaponID)i );
        if( !info )
            continue;

        ItemListEntry entry;
        entry.itemPrice = info->GetWeaponPrice();
        entry.itemName = WeaponIdAsString( (CSWeaponID)i );

        int weaponType = info->GetWeaponType();

        switch( weaponType )
        {
            case WEAPONTYPE_EQUIPMENT:
                buyMenuData.gearList.push_back( entry );
                break;
            case WEAPONTYPE_GRENADE:
                buyMenuData.grenadeList.push_back( entry );
                break;
            case WEAPONTYPE_PISTOL:
                buyMenuData.pistolList.push_back( entry );
                break;
            case WEAPONTYPE_RIFLE:
            case WEAPONTYPE_SNIPER_RIFLE:
                buyMenuData.rifleList.push_back( entry );
                break;
            case WEAPONTYPE_SUBMACHINEGUN:
                buyMenuData.smgList.push_back( entry );
                break;
            case WEAPONTYPE_SHOTGUN:
            case WEAPONTYPE_MACHINEGUN:
                buyMenuData.heavyList.push_back( entry );
                break;
            default:
            {
                continue;
            }
        }
    }

    m_dataModel.DirtyVariable( "gear_list" );
    m_dataModel.DirtyVariable( "grenade_list" );
    m_dataModel.DirtyVariable( "pistol_list" );
    m_dataModel.DirtyVariable( "rifle_list" );
    m_dataModel.DirtyVariable( "smg_list" );
    m_dataModel.DirtyVariable( "heavy_list" );

    m_dataModel.Update();
}

// called 1x per frame
void RkHudBuyMenu::OnNewFrameBuyMenu()
{
    float timeIntoRound = CSGameRules()->GetRoundElapsedTime();
    float buyTime = CSGameRules()->GetBuyTimeLength();

    buyMenuData.buyTimeLeft = int(buyTime - timeIntoRound);

    C_CSPlayer *localPlayer = C_CSPlayer::GetLocalCSPlayer();
    if( localPlayer )
    {
        buyMenuData.playerCash = localPlayer->GetAccount();
        buyMenuData.playerTeamNum = localPlayer->GetTeamNumber();
        m_dataModel.DirtyVariable( "player_cash" );
        m_dataModel.DirtyVariable( "player_teamnum" );
    }

    m_dataModel.DirtyVariable("buy_time_left");

    m_dataModel.Update();
}

static void UnloadRkBuyMenu()
{
    RkHudBuyMenu *pBuyMenu = GET_HUDELEMENT( RkHudBuyMenu );
    if( !pBuyMenu )
    {
        Warning( "Couldn't grab RkBuyMenu to unload!\n" );
        return;
    }

    if( !pBuyMenu->m_pInstance )
        return;

    Rml::Context *hudCtx = RocketUI()->AccessHudContext();
    if( hudCtx )
    {
        hudCtx->RemoveDataModel( "buymenu_model" );
        pBuyMenu->m_dataModel = nullptr;
    }
    else
    {
        Warning( "Couldn't access hudctx to unload buymenu datamodel!\n" );
    }

    pBuyMenu->m_pInstance->RemoveEventListener( Rml::EventId::Mousedown, &buyMenuListener );
    pBuyMenu->m_pInstance->RemoveEventListener( Rml::EventId::Keyup, &buyMenuListener );
    pBuyMenu->m_pInstance->Close();
    pBuyMenu->m_pInstance = nullptr;
    pBuyMenu->m_bVisible = false;

    if( pBuyMenu->m_bGrabbingInput )
    {
        RocketUI()->DenyInputToGame( false, "BuyMenu" );
        pBuyMenu->m_bGrabbingInput = false;
    }
}

static void LoadRkBuyMenu()
{
    RkHudBuyMenu *pBuyMenu = GET_HUDELEMENT( RkHudBuyMenu );
    if( !pBuyMenu )
    {
        Warning( "Couldn't grab hud buymenu to load!\n" );
        return;
    }

    Rml::Context *hudCtx = RocketUI()->AccessHudContext();
    if( !hudCtx )
    {
        Error( "Couldn't access hudctx!\n" );
        /* Exit */
    }

    if( pBuyMenu->m_pInstance || pBuyMenu->m_dataModel )
    {
        Warning( "RkBuyMenu already loaded! call unload first!\n" );
        return;
    }

    // Create the data binding, this will sync data between rocketui and the game.
    Rml::DataModelConstructor constructor = hudCtx->CreateDataModel( "buymenu_model" );
    if( !constructor )
    {
        Error( "Couldn't create datamodel for buymenu!\n" );
        /* Exit */
    }

    // Register ItemListEntry struct
    if( auto itemlist_handle = constructor.RegisterStruct<ItemListEntry>() )
    {
        itemlist_handle.RegisterMember( "item_name", &ItemListEntry::itemName );
        itemlist_handle.RegisterMember( "item_price", &ItemListEntry::itemPrice );
    }

    // Register the array type
    constructor.RegisterArray<Rml::Vector<ItemListEntry>>();

    // Bind the Lists of items
    constructor.Bind( "gear_list", &buyMenuData.gearList );
    constructor.Bind( "grenade_list", &buyMenuData.grenadeList );
    constructor.Bind( "pistol_list", &buyMenuData.pistolList );
    constructor.Bind( "rifle_list", &buyMenuData.rifleList );
    constructor.Bind( "smg_list", &buyMenuData.smgList );
    constructor.Bind( "heavy_list", &buyMenuData.heavyList );

    // Bind regular data
    constructor.Bind( "player_cash", &buyMenuData.playerCash );
    constructor.Bind( "player_teamnum", &buyMenuData.playerTeamNum );
    constructor.Bind( "buy_time_left", &buyMenuData.buyTimeLeft );

    pBuyMenu->m_dataModel = constructor.GetModelHandle();

    // Load document from file.
    pBuyMenu->m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_HUD, "hud_buymenu.rml", &LoadRkBuyMenu, &UnloadRkBuyMenu );
    if( !pBuyMenu->m_pInstance )
    {
        Error( "Couldn't create hud_buymenu document!\n" );
        /* Exit */
    }

    // Add event listener for input
    pBuyMenu->m_pInstance->AddEventListener( Rml::EventId::Mousedown, &buyMenuListener );
    pBuyMenu->m_pInstance->AddEventListener( Rml::EventId::Keyup, &buyMenuListener );
}

RkHudBuyMenu::RkHudBuyMenu(const char *value) : CHudElement( value ),
                                                m_bVisible( false ),
                                                m_bGrabbingInput( false ),
                                                m_pInstance( nullptr )
{
    SetHiddenBits( /* HIDEHUD_MISCSTATUS */ 0 );
}

RkHudBuyMenu::~RkHudBuyMenu() noexcept
{
    StopListeningForAllEvents();

    UnloadRkBuyMenu();
}

void RkHudBuyMenu::LevelInit()
{
    ListenForGameEvent( "buymenu_open" );
    ListenForGameEvent( "buymenu_close" );

    LoadRkBuyMenu();
}

void RkHudBuyMenu::LevelShutdown()
{
    UnloadRkBuyMenu();
}

void RkHudBuyMenu::ShowPanel(bool bShow, bool force)
{
    if( !m_pInstance )
        return;

    if( bShow )
    {
        if( !m_bVisible )
        {
            UpdateBuyMenu();
            RocketUI()->DenyInputToGame( true, "BuyMenu" );
            m_pInstance->Show();
        }
        OnNewFrameBuyMenu();
    }
    else
    {
        if( m_bVisible )
        {
            RocketUI()->DenyInputToGame( false, "BuyMenu" );
            m_pInstance->Hide();
        }
    }

    m_bVisible = bShow;
}

void RkHudBuyMenu::SetActive(bool bActive)
{
    ShowPanel( bActive, false );
    CHudElement::SetActive( bActive );
}

bool RkHudBuyMenu::ShouldDraw()
{
    // This element is opened/closed by clientside events
    // that we listen for and set m_bVisible manually via showpanel(true)

    return cl_drawhud.GetBool() && CSGameRules() && !CSGameRules()->IsBuyTimeElapsed() && m_bVisible && CHudElement::ShouldDraw();
}

void RkHudBuyMenu::FireGameEvent(IGameEvent *event)
{
    const char *type = event->GetName();

    if( !V_strcmp( "buymenu_open", type ) )
    {
        ShowPanel( true, false );
    }
    else if( !V_strcmp( "buymenu_close", type ) )
    {
        ShowPanel( false, false );
    }
}