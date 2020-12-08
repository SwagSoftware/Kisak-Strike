#include "rkhud_chat.h"

#include "cbase.h"
#include "cs_gamerules.h"
#include "hud_macros.h"
#include "text_message.h"
#include "c_cs_playerresource.h"

#include <tier0/valve_minmax_off.h> // included to fix an error with min/max and rocketui

#include "../../../../thirdparty/RmlUi/Include/RmlUi/Core.h"

#include "rkpanel_popup.h"

DECLARE_HUDELEMENT( RkHudChat );

ConVar rocket_hud_chat_idle_opacity( "rocket_hud_chat_idle_opacity", "0.2", 0, "The Opacity of the Chat while it is not active" );
ConVar rocket_hud_chat_active_opacity( "rocket_hud_chat_active_opacity", "0.7", 0, "The Opacity of the Chat while typing/new message" );
ConVar rocket_hud_chat_max_entries( "rocket_hud_chat_max_entries", "1000", 0, "Chat History Length" );

CON_COMMAND_F( rocket_hud_chat_clear, "Clears the Chat History", FCVAR_NONE )
{
    RkHudChat* pChat = GET_HUDELEMENT( RkHudChat );
    if( !pChat )
        return;
    pChat->ClearChatHistory();
}

static bool __MsgFunc_SayText2( const CCSUsrMsg_SayText2 &msg )
{
    int paramSize = msg.params_size();
    int entID = msg.ent_idx();

    RkHudChat* pChat = GET_HUDELEMENT( RkHudChat );

    if( !pChat )
        return true;

    // from server
    if( entID == 0 && paramSize > 1 )
    {
        pChat->AddChatString( nullptr, msg.params(1).c_str(), RkHudChat::SERVER );
        return true;
    }

    CBasePlayer *speaker = UTIL_PlayerByIndex( entID );
    CBasePlayer *localPlayer = C_BasePlayer::GetLocalPlayer( );

    if( !speaker || !localPlayer )
        return true;

    // params(0) = Name of player
    // params(1) = Message
    if( paramSize > 1 )
    {
        if( (speaker->GetTeamNumber() == localPlayer->GetTeamNumber()) )
            pChat->AddChatString( msg.params(0).c_str(), msg.params(1).c_str(),RkHudChat::FRIEND);
        else
            pChat->AddChatString( msg.params(0).c_str(), msg.params(1).c_str(),RkHudChat::FOE);
    }

    return true;
}
// converts all '\r' characters to '\n', so that the engine can deal with the properly
// returns a pointer to str
static char* ConvertCRtoNL( char *str )
{
    for ( char *ch = str; *ch != 0; ch++ )
        if ( *ch == '\r' )
            *ch = '\n';
    return str;
}

static void NullLastNewlineFromString( char *str )
{
    int s = strlen( str ) - 1;
    if ( s >= 0 )
    {
        if ( str[s] == '\n' || str[s] == '\r' )
            str[s] = 0;
    }
}

//-----------------------------------------------------------------------------
// Message handler for text messages
// displays a string, looking them up from the titles.txt file, which can be localised
// parameters:
//   byte:   message destination  ( HUD_PRINTCONSOLE, HUD_PRINTNOTIFY, HUD_PRINTCENTER, HUD_PRINTTALK )
//   string: message
// optional parameters:
//   string: message parameter 1
//   string: message parameter 2
//   string: message parameter 3
//   string: message parameter 4
// any string that starts with the character '#' is a message name, and is used to look up the real message in titles.txt
// the next ( optional) one to four strings are parameters for that string ( which can also be message names if they begin with '#')
//-----------------------------------------------------------------------------
static bool __MsgFunc_TextMsg( const CCSUsrMsg_TextMsg &msg )
{
    char szString[2048] = {0};
    int dest = msg.msg_dst();

    wchar_t szBuf[5][256] = {};
    wchar_t outputBuf[256] = {};

    if( !cl_showtextmsg.GetBool() )
        return true;

    for( int i = 0; i < 4; i++ )
    {
        // Allow localizing player names
        if ( const char *pszEntIndex = StringAfterPrefix( msg.params(i).c_str(), "#ENTNAME[" ) )
        {
            int iEntIndex = V_atoi( pszEntIndex );
            wchar_t wszPlayerName[MAX_DECORATED_PLAYER_NAME_LENGTH] = {};
            if ( C_CS_PlayerResource *pCSPR = ( C_CS_PlayerResource* ) GameResources() )
            {
                pCSPR->GetDecoratedPlayerName( iEntIndex, wszPlayerName, sizeof( wszPlayerName ), ( EDecoratedPlayerNameFlag_t ) ( k_EDecoratedPlayerNameFlag_DontUseNameOfControllingPlayer | k_EDecoratedPlayerNameFlag_DontUseAssassinationTargetName ) );
            }
            if ( wszPlayerName[0] )
            {
                szString[0] = 0;
                V_wcscpy_safe( szBuf[ i ], wszPlayerName );
            }
            else if ( const char *pszEndBracket = V_strnchr( pszEntIndex, ']', 64 ) )
            {
                V_strcpy_safe( szString, pszEndBracket + 1 );
            }
            else
            {
                V_strcpy_safe( szString, msg.params(i).c_str() );
            }
        }
        else
        {
            V_strcpy_safe( szString, msg.params(i).c_str() );
        }

        if( szString[0] )
        {
            static char tmpStrBuf[1024];
            V_strncpy( tmpStrBuf, hudtextmessage->LookupString( szString, &dest ), sizeof(tmpStrBuf) );
            bool bTranslated = false;
            if( tmpStrBuf[0] == '#' ) // only translate parameters intended as localization tokens
            {
                const wchar_t *pBuf = g_pVGuiLocalize->Find( tmpStrBuf );
                if( pBuf )
                {
                    // copy pBuf into szBuf[i]
                    int nMaxChars = sizeof( szBuf[ i ] ) / sizeof( wchar_t );
                    wcsncpy( szBuf[ i ], pBuf, nMaxChars );
                    szBuf[ i ][ nMaxChars - 1 ] = 0;
                    bTranslated = true;
                }
            }

            if( !bTranslated )
            {
                if( i > 0 )
                {
                    NullLastNewlineFromString( tmpStrBuf );  // these strings are meant for substitution into the main strings, so cull the automatic end newlines
                }
                g_pVGuiLocalize->ConvertANSIToUnicode( tmpStrBuf, szBuf[ i ], sizeof( szBuf[ i ] ) );
            }
        }
    }

    //TODO: right now this is just ascii
    int len;
    RkHudChat* pChat;
    g_pVGuiLocalize->ConstructString( outputBuf, sizeof( outputBuf), szBuf[0], 4, szBuf[1], szBuf[2], szBuf[3], szBuf[4] );
    g_pVGuiLocalize->ConvertUnicodeToANSI( outputBuf, szString, sizeof( szString) );
    len = strlen( szString );
    if ( len && szString[len-1] != '\n' && szString[len-1] != '\r' )
    {
        Q_strncat( szString, "\n", sizeof( szString), 1 );
    }

    switch( dest )
    {
        case HUD_PRINTCENTER:
            // Center popup with RocketUI that deletes itself after 4.5 seconds
            new RocketPopupDocument( ConvertCRtoNL( szString ), 4.5f );
            break;
        case HUD_PRINTTALK:
            pChat = GET_HUDELEMENT( RkHudChat );
            if( !pChat )
                return true;

            pChat->AddChatString( nullptr, ConvertCRtoNL( szString ), RkHudChat::SERVER );
            break;
        case HUD_PRINTNOTIFY:
        case HUD_PRINTCONSOLE:
            Msg( "%s", ConvertCRtoNL( szString ) );
            break;
    }

    return true;
}

class RkHudChatEventListener : public Rml::EventListener
{
public:
    void ProcessEvent(Rml::Event& keyevent) override
    {
        char sayBuffer[1024];
        switch (keyevent.GetId())
        {
            case Rml::EventId::Keydown:
            {
                const Rml::Dictionary& params = keyevent.GetParameters();
                Rml::Input::KeyIdentifier key_identifier = (Rml::Input::KeyIdentifier) keyevent.GetParameter< int >("key_identifier", 0);
                if (key_identifier == Rml::Input::KI_ESCAPE)
                {
                    // Close the chat.
                    RkHudChat* pChat = GET_HUDELEMENT( RkHudChat );
                    if( !pChat )
                        break;

                    pChat->StopMessageMode();
                }
                else if (key_identifier == Rml::Input::KI_RETURN)
                {
                    // Submit the Chat
                    RkHudChat* pChat = GET_HUDELEMENT( RkHudChat );
                    if( !pChat )
                        break;

                    Rml::ElementFormControl *input = static_cast<Rml::ElementFormControl*>(pChat->m_elemChatInput);

                    V_snprintf( sayBuffer, sizeof( sayBuffer ), "%s \"%s\"", pChat->GetMessageMode() == MM_SAY ? "say" : "say_team", input->GetValue().c_str() );

                    engine->ClientCmd_Unrestricted( sayBuffer );
                    input->SetValue("");

                    pChat->StopMessageMode();
                }
            }
                break;

            default:
                break;
        }
    }
};

static RkHudChatEventListener chatEventListener;

static void UnloadRkChat()
{
    RkHudChat* pChat = GET_HUDELEMENT( RkHudChat );
    if( !pChat )
    {
        Warning("Couldn't grab RkHudChat element to Unload\n");
        return;
    }

    // Not loaded
    if( !pChat->m_pInstance )
        return;

    pChat->m_pInstance->Close();
    pChat->m_pInstance = nullptr;
}

static void LoadRkChat()
{
    RkHudChat* pChat = GET_HUDELEMENT( RkHudChat );
    if( !pChat )
    {
        Warning("Couldn't grab RkHudChat element to load\n");
        return;
    }

    pChat->m_pInstance = RocketUI()->LoadDocumentFile( ROCKET_CONTEXT_HUD, "hud_chat.rml", LoadRkChat, UnloadRkChat );

    if( !pChat->m_pInstance )
    {
        Error("Couldn't create hud_chat document!\n");
        /* Exit */
    }

    pChat->m_elemChatLines = pChat->m_pInstance->GetElementById( "chat_lines" );
    if( !pChat->m_elemChatLines )
    {
        Error( "Couldn't find required element id: 'chat_lines' in hud_chat\n" );
        /* Exit */
    }

    pChat->m_elemChatInput = pChat->m_pInstance->GetElementById( "chat_input" );
    if( !pChat->m_elemChatInput )
    {
        Error( "Couldn't find required element id: 'chat_input' in hud_chat\n" );
        /* Exit */
    }

    // Add a listener to both the Chat panel and the text-input element in case it changes focus.
    pChat->m_pInstance->AddEventListener( Rml::EventId::Keydown, &chatEventListener );
    pChat->m_elemChatInput->AddEventListener( Rml::EventId::Keydown, &chatEventListener );

    pChat->m_pInstance->Show();
}

// Called on program startup by a Macro with the HUD UI system
// HudElementHelper::CreateAllElements
RkHudChat::RkHudChat(const char *value) : CHudElement( value ),
                                          m_pInstance( nullptr ),
                                          m_bVisible( false ),
                                          m_iMode( MM_NONE ),
                                          m_bGrabbingInput( false ),
                                          m_iNumEntries( 0 )
{
    SetHiddenBits( /* HIDEHUD_MISCSTATUS */ 0 );
}

RkHudChat::~RkHudChat() noexcept
{
    UnloadRkChat();
}

void RkHudChat::LevelInit( void )
{
    HOOK_MESSAGE( SayText2 );
    HOOK_MESSAGE( TextMsg );

    LoadRkChat();
}

void RkHudChat::LevelShutdown( void )
{
    m_iMode = MM_NONE;

    if( m_bGrabbingInput )
        RocketUI()->DenyInputToGame( false, "Hud_Chat" );

    UnloadRkChat();
}

// this is called every frame, keep that in mind.
void RkHudChat::ShowPanel(bool bShow, bool force)
{
    if( !m_pInstance )
        return;

    if( bShow )
    {
        if( !m_bVisible )
        {
            m_bVisible = true;
            m_bGrabbingInput = true;
            RocketUI()->DenyInputToGame( true, "Hud_Chat" );
        }

        m_pInstance->SetProperty( "opacity", std::to_string(rocket_hud_chat_active_opacity.GetFloat()) );
    }
    else
    {
        if( m_bVisible )
        {
            m_bVisible = false;
            m_bGrabbingInput = false;
            RocketUI()->DenyInputToGame( false, "Hud_Chat" );
        }

        // Do a fade out to the idle opacity level.
        float currentOpacity = m_pInstance->GetProperty("opacity")->Get<float>();
        if( currentOpacity > rocket_hud_chat_idle_opacity.GetFloat() )
            m_pInstance->SetProperty("opacity", std::to_string(currentOpacity - 0.0075f) );
    }
}

// Called every Frame by the hudsystem if ShouldDraw()
void RkHudChat::SetActive( bool bActive )
{
    ShowPanel( bActive, false );
    CHudElement::SetActive( bActive );
}

bool RkHudChat::ShouldDraw( void )
{
    if ( //IsTakingAFreezecamScreenshot() ||
            (CSGameRules() && CSGameRules()->IsPlayingTraining()) )
        return false;

    return cl_drawhud.GetBool() && (m_iMode != MM_NONE ) && CHudElement::ShouldDraw();
}

void RkHudChat::StartMessageMode( int mode )
{
    // Already in chat mode.
    if( ChatRaised() )
        return;
    
    if( GetHud().HudDisabled() )
        return;

    m_iMode = mode;

    m_elemChatInput->Focus();
}

void RkHudChat::StopMessageMode()
{
    m_iMode = MM_NONE;

    m_elemChatInput->Blur();
}

bool RkHudChat::ChatRaised()
{
    return m_iMode != MM_NONE;
}

void RkHudChat::ClearChatHistory()
{
    if( !m_pInstance )
        return;

    Rml::ElementList chatEntries;
    m_pInstance->GetElementsByClassName( chatEntries, "chat_line" );
    for( Rml::Element *elem : chatEntries )
    {
        elem->GetParentNode()->RemoveChild( elem );
    }
    m_iNumEntries = 0;
}

void RkHudChat::AddChatString( const char *username, const char *message, MessageSender sender )
{
    // Max Number of Entries, delete old ones if needed.
    m_iNumEntries++;
    if( m_iNumEntries > rocket_hud_chat_max_entries.GetInt() )
    {
        ClearChatHistory();
    }

    Rml::ElementPtr chatLine = m_pInstance->CreateElement("#text");
    Rml::ElementPtr br = m_pInstance->CreateElement("br");

    if( !chatLine || !br )
        return;

    chatLine->SetClass("chat_line", true);
    chatLine->AppendChild( std::move(br) );

    if( username )
    {
        Rml::ElementPtr chatUsername = m_pInstance->CreateElement("#text");
        Rml::String usernameText = username + Rml::String(": ");

        if( !chatUsername )
            return;

        Rml::ElementText *chatUsernameElement = static_cast< Rml::ElementText* >( chatUsername.get() );
        switch( sender )
        {
            case RkHudChat::SERVER:
                chatUsernameElement->SetClass("chat_username_server", true);
                break;
            case RkHudChat::FRIEND:
                chatUsernameElement->SetClass("chat_username_friend", true);
                break;
            case RkHudChat::FOE:
                chatUsernameElement->SetClass("chat_username_foe", true);
                break;
        }
        chatUsernameElement->SetText( usernameText );
        chatLine->AppendChild( std::move(chatUsername) );
    }

    if( message )
    {
        Rml::ElementPtr chatMessage = m_pInstance->CreateElement("#text");
        if( !chatMessage )
            return;

        Rml::ElementText *chatMessageElement = static_cast< Rml::ElementText* >( chatMessage.get() );
        chatMessageElement->SetClass("chat_message", true);
        chatMessageElement->SetText( message );
        chatLine->AppendChild( std::move( chatMessage ) );

        m_elemChatLines->AppendChild( std::move(chatLine) );
    }


    // Update the document so the scrollbar will take the new elements into account.
    m_pInstance->UpdateDocument();
    // Scroll to the bottom (1.0)
    m_pInstance->SetScrollTop( 1.0f * ( m_pInstance->GetScrollHeight() ) - m_pInstance->GetClientHeight() );
}

void RkHudChat::AddChatString( const wchar_t *username, const wchar_t *message, MessageSender sender )
{
    //TODO: wide strings.
}
