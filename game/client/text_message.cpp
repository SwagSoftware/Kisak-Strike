//========= Copyright � 1996-2005, Valve Corporation, All rights reserved. ============//
//
// Purpose: 
//
// $NoKeywords: $
//
//=============================================================================//
//
// text_message.cpp
//
// implementation of CHudTextMessage class
//
// this class routes messages through titles.txt for localisation
//
#include "cbase.h"
#include "text_message.h"
#include "client_textmessage.h"
#include "vgui_controls/Controls.h"
#include "vgui/ILocalize.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-----------------------------------------------------------------------------
// Purpose: Implements IHudTextMessage
//-----------------------------------------------------------------------------
class CHudTextMessage : public IHudTextMessage
{
public:
	virtual char *LocaliseTextString( const char *msg, char *dst_buffer, int buffer_size );
	virtual const char * const BufferedLocaliseTextString( const char *msg );
	virtual const char *LookupString( const char *msg_name, int *msg_dest = NULL );
};

// Singleton
static CHudTextMessage g_HudTextMessage;
IHudTextMessage *hudtextmessage = &g_HudTextMessage;

//-----------------------------------------------------------------------------
// Purpose:  Searches through the string for any msg names (indicated by a '#')
// any found are looked up in titles.txt and the new message substituted
// the new value is pushed into dst_buffer
// Input  : *msg - 
//			*dst_buffer - 
//			buffer_size - 
// Output : char
//-----------------------------------------------------------------------------
char *CHudTextMessage::LocaliseTextString( const char *msg, char *dst_buffer, int buffer_size )
{
	char *dst = dst_buffer;
	for ( const char *src = msg; *src != 0 && buffer_size > 0; buffer_size-- )
	{
		if ( *src == '#' )
		{
			// cut msg name out of string
            static const int max_word_len = 255;
            static char word_buf[max_word_len];
			const char *word_start = src;
            char *word = word_buf;
            int word_len = 0;
            for ( ++src ; *src >= 'A' && *src <= 'z'; word++, src++ )
			{
				*word = *src;
				if( word_len++ >= ( max_word_len - 1 ) )
                {
				    // word is too long, return empty string.
				    Warning( "#Token too long in LocaliseTextString!\n" );
				    dst_buffer[0] = 0;
				    return dst_buffer;
                }
			}
			*word = 0;

			// lookup msg name in titles.txt
			client_textmessage_t *clmsg = TextMessageGet( word_buf );
			if ( !clmsg || !(clmsg->pMessage) )
			{
				src = word_start;
				*dst = *src;
				dst++, src++;
				continue;
			}

			// Does titles.txt want to lookup into cstrike_<language>.txt?
			wchar_t *pLocalizedStr;
			if ( clmsg->pMessage[0] == '#' && ((pLocalizedStr = g_pVGuiLocalize->Find( clmsg->pMessage )) != NULL ) )
			{
				g_pVGuiLocalize->ConvertUnicodeToANSI( pLocalizedStr, dst, buffer_size );
			}
			else
			{
				// copy string into message over the msg name
				for ( char *wsrc = (char*)clmsg->pMessage; *wsrc != 0; wsrc++, dst++ )
				{
					*dst = *wsrc;
				}
				*dst = 0;
			}
		}
		else
		{
			*dst = *src;
			dst++, src++;
			*dst = 0;
		}
	}

    // ensure null termination
	dst_buffer[buffer_size-1] = 0; 
	return dst_buffer;
}

//-----------------------------------------------------------------------------
// Purpose: As above, but with a local static buffer
// Input  : *msg - 
// Output : char pointer to local static buffer that shouldn't be changed.
//-----------------------------------------------------------------------------
const char * const CHudTextMessage::BufferedLocaliseTextString( const char *msg )
{
	static char dst_buffer[1024];
	return LocaliseTextString( msg, dst_buffer, 1024 );
}

//-----------------------------------------------------------------------------
// Purpose:  Simplified version of LocaliseTextString;  assumes string is only one word
// Input  : *msg - 
//			*msg_dest - 
// Output : char
//-----------------------------------------------------------------------------
const char *CHudTextMessage::LookupString( const char *msg, int *msg_dest )
{
	if ( !msg )
		return "";

	// '#' character indicates this is a reference to a string in titles.txt, and not the string itself
	if ( msg[0] == '#' ) 
	{
		// this is a message name, so look up the real message
		client_textmessage_t *clmsg = TextMessageGet( msg+1 );

		if ( !clmsg || !(clmsg->pMessage) )
			return msg; // lookup failed, so return the original string
		
		if ( msg_dest )
		{
			// check to see if titles.txt info overrides msg destination
			// if clmsg->effect is less than 0, then clmsg->effect holds -1 * message_destination
			if ( clmsg->effect < 0 )  // 
				*msg_dest = -clmsg->effect;
		}

		return clmsg->pMessage;
	}
	else
	{  // nothing special about this message, so just return the same string
		return msg;
	}
}