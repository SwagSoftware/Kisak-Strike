#include "rkhud_infobar.h"

#include "cbase.h"
#include "hud_macros.h"
#include "c_cs_player.h"

#include <tier0/valve_minmax_off.h> // included to fix an error with min/max and rocketui

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"

DECLARE_HUDELEMENT( RkHudInfoBar );

// Struct layout for data-binding model.
struct InfoBarData
{
    int hp;
    int armor;
    bool hasHelmet;
    int ammo;
    int ammoReserve;
    Rml::String fireModeString;
    Rml::String primaryString;
    Rml::String secondaryString;
    Rml::String knifeString;
    bool hasGrenade;
    bool hasFlash;
    bool hasFlashPair;
    bool hasDecoy;
    bool hasSmoke;
    bool hasFire;
    bool hasC4;
} infoBarData;

static void UpdateInfoFromPlayer( const C_CSPlayer &pPlayer )
{
    infoBarData.hp = pPlayer.GetHealth();
    infoBarData.armor = pPlayer.ArmorValue();
    infoBarData.hasHelmet = false;
    if( pPlayer.HasHelmet() )
        infoBarData.hasHelmet = true;

    infoBarData.fireModeString = " ";
    infoBarData.primaryString = " ";
    infoBarData.secondaryString = " ";
    infoBarData.knifeString = " ";
    infoBarData.hasGrenade = false;
    //infoBarData.hasFlash = false;
    //infoBarData.hasFlashPair = false;
    infoBarData.hasDecoy = false;
    infoBarData.hasSmoke = false;
    infoBarData.hasFire = false;
    infoBarData.hasC4 = false;

    int flashbangAmount = 0;
    for( int i = 0; i < MAX_WEAPONS; i++ )
    {
        CWeaponCSBase *weapon = (CWeaponCSBase*)pPlayer.GetWeapon(i);
        if( !weapon )
            continue;

        int slot = weapon->GetSlot();
        const char *name;

        switch( slot )
        {
            case WEAPON_SLOT_RIFLE:
                name = V_strstr(weapon->GetName(), "_");
                if( name && name[0] )
                    infoBarData.primaryString = name+1;
                break;
            case WEAPON_SLOT_PISTOL:
                name = V_strstr(weapon->GetName(), "_");
                if( name && name[0] )
                    infoBarData.secondaryString = name+1;
                break;
            case WEAPON_SLOT_KNIFE:
                name = V_strstr(weapon->GetName(), "_");
                if( name && name[0] )
                    infoBarData.knifeString = name+1;
                break;
            case WEAPON_SLOT_GRENADES:
            {
                int weaponID = weapon->GetCSWeaponID();
                switch( weaponID )
                {
                    case WEAPON_HEGRENADE:
                        infoBarData.hasGrenade = true;
                        break;
                    case WEAPON_FLASHBANG:
                        flashbangAmount++;
                        break;
                    case WEAPON_SMOKEGRENADE:
                        infoBarData.hasSmoke = true;
                        break;
                    case WEAPON_MOLOTOV:
                        infoBarData.hasFire = true;
                        break;
                    case WEAPON_INCGRENADE:
                        infoBarData.hasFire = true;
                        break;
                    case WEAPON_DECOY:
                        infoBarData.hasDecoy = true;
                    default:
                        break;
                }
                break;
            }
            case WEAPON_SLOT_C4:
                infoBarData.hasC4 = true;
                break;
            default:
                break;
        }
    }
    infoBarData.hasFlash = ( flashbangAmount == 1 );
    infoBarData.hasFlashPair = ( flashbangAmount == 2 );

    CWeaponCSBase *activeWeapon = pPlayer.GetActiveCSWeapon();
    if( activeWeapon )
    {
        infoBarData.ammo = activeWeapon->Clip1();
        infoBarData.ammoReserve = activeWeapon->GetReserveAmmoCount( AMMO_POSITION_PRIMARY );
        if( activeWeapon->IsFullAuto() )
            infoBarData.fireModeString = "AUTO";
        else if( activeWeapon->IsInBurstMode() )
            infoBarData.fireModeString = "BURST";
        else
            infoBarData.fireModeString = "SINGLE";
    }
}

static void UnloadRkInfoBar()
{
    RkHudInfoBar *pInfoBar = GET_HUDELEMENT( RkHudInfoBar );
    if( !pInfoBar )
    {
        Warning( "Couldn't grab RkHudInfoBar element to unload!\n");
        return;
    }

    // Not loaded
    if( !pInfoBar->m_pInstance )
        return;

    Rml::Context *hudCtx = RocketUI()->AccessHudContext();
    if( hudCtx )
    {
        hudCtx->RemoveDataModel("infobar_model");
        pInfoBar->m_dataModel = nullptr;
    }
    else
    {
        Warning("Couldn't access hudCtx to unload infobar datamodel\n");
    }

    pInfoBar->m_pInstance->Close();
    pInfoBar->m_pInstance = nullptr;
}

static void LoadRkInfoBar()
{
    RkHudInfoBar *pInfoBar = GET_HUDELEMENT( RkHudInfoBar );
    if( !pInfoBar )
    {
        Warning( "Couldn't grab hud infobar to load!\n");
        return;
    }

    Rml::Context *hudCtx = RocketUI()->AccessHudContext();
    if( !hudCtx )
    {
        Error("Couldn't access hudctx!\n");
        /* Exit */
    }

    if( pInfoBar->m_pInstance || pInfoBar->m_dataModel )
    {
        Warning("RkInfoBar already loaded, call unload first!\n");
        return;
    }

    // Create the data binding, this will sync data between rocketui and the game.
    Rml::DataModelConstructor constructor = hudCtx->CreateDataModel("infobar_model");
    if( !constructor )
    {
        Error( "Couldn't create datamodel for infobar!\n");
        /* Exit */
    }

    constructor.Bind("hp", &infoBarData.hp);
    constructor.Bind("armor", &infoBarData.armor);
    constructor.Bind("ammo", &infoBarData.ammo);
    constructor.Bind("ammo_reserve", &infoBarData.ammoReserve);
    constructor.Bind("fire_mode_string", &infoBarData.fireModeString);
    constructor.Bind("has_helmet", &infoBarData.hasHelmet);
    constructor.Bind("primary_string", &infoBarData.primaryString);
    constructor.Bind("secondary_string", &infoBarData.secondaryString);
    constructor.Bind("knife_string", &infoBarData.knifeString);
    constructor.Bind("has_grenade", &infoBarData.hasGrenade);
    constructor.Bind("has_decoy", &infoBarData.hasDecoy);
    constructor.Bind("has_flash", &infoBarData.hasFlash);
    constructor.Bind("has_flash_pair", &infoBarData.hasFlashPair);
    constructor.Bind("has_smoke", &infoBarData.hasSmoke);
    constructor.Bind("has_fire", &infoBarData.hasFire);
    constructor.Bind("has_c4", &infoBarData.hasC4);

    pInfoBar->m_dataModel = constructor.GetModelHandle();

    pInfoBar->m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_HUD, "hud_infobar.rml", LoadRkInfoBar, UnloadRkInfoBar );

    if( !pInfoBar->m_pInstance )
    {
        Error("Couldn't create hud_infobar document!\n");
        /* Exit */
    }

    //pInfoBar->m_pInstance->Show();
    pInfoBar->ShowPanel( false, false );
}

RkHudInfoBar::RkHudInfoBar(const char *value) : CHudElement( value ),
                                                m_bVisible( false ),
                                                m_pInstance( nullptr )
{
    SetHiddenBits( /* HIDEHUD_MISCSTATUS */ 0 );
}

RkHudInfoBar::~RkHudInfoBar() noexcept
{
    UnloadRkInfoBar();
}

void RkHudInfoBar::LevelInit()
{
    LoadRkInfoBar();
}

void RkHudInfoBar::LevelShutdown()
{
    UnloadRkInfoBar();
}

// this is called every frame, keep that in mind.
void RkHudInfoBar::ShowPanel(bool bShow, bool force)
{
    if( !m_pInstance )
        return;

    if( bShow )
    {
        if( !m_bVisible )
        {
            m_pInstance->Show();
        }
        C_CSPlayer *pPlayer = C_CSPlayer::GetLocalCSPlayer();

        if( !pPlayer )
            goto end;

        // observing someone? switch to that player.
        if( pPlayer->IsObserver() && (pPlayer->GetObserverMode() == OBS_MODE_IN_EYE || pPlayer->GetObserverMode() == OBS_MODE_CHASE) )
            pPlayer = ToCSPlayer(pPlayer->GetObserverTarget());

        if( !pPlayer )
            goto end;

        UpdateInfoFromPlayer( *pPlayer );

        m_dataModel.DirtyVariable( "hp" );
        m_dataModel.DirtyVariable( "ammo" );
        m_dataModel.DirtyVariable( "ammo_reserve" );
        m_dataModel.DirtyVariable( "fire_mode_string" );
        m_dataModel.DirtyVariable( "armor" );
        m_dataModel.DirtyVariable( "has_helmet" );
        m_dataModel.DirtyVariable( "primary_string" );
        m_dataModel.DirtyVariable( "secondary_string" );
        m_dataModel.DirtyVariable( "knife_string" );
        m_dataModel.DirtyVariable( "has_grenade" );
        m_dataModel.DirtyVariable( "has_decoy" );
        m_dataModel.DirtyVariable( "has_flash" );
        m_dataModel.DirtyVariable( "has_flash_pair" );
        m_dataModel.DirtyVariable( "has_smoke" );
        m_dataModel.DirtyVariable( "has_fire" );
        m_dataModel.DirtyVariable( "has_c4" );

        m_dataModel.Update();
    }
    else
    {
        if( m_bVisible )
        {
            m_pInstance->Hide();
        }
    }

end:
    m_bVisible = bShow;
}

void RkHudInfoBar::SetActive(bool bActive)
{
    ShowPanel( bActive, false );
    CHudElement::SetActive( bActive );
}

bool RkHudInfoBar::ShouldDraw()
{
    C_CSPlayer *localPlayer = C_CSPlayer::GetLocalCSPlayer();

    return localPlayer &&
    cl_drawhud.GetBool() &&
    ( localPlayer->IsAlive() || ( localPlayer->IsObserver() && localPlayer->GetObserverMode() == OBS_MODE_IN_EYE || localPlayer->GetObserverMode() == OBS_MODE_CHASE ) ) &&
    CHudElement::ShouldDraw();
}