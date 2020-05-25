//========== Copyright (c) 2008, Valve Corporation, All rights reserved. ========
//
// Purpose:
//
//=============================================================================

#include "cbase.h"
#include "vscript_client.h"
#include "icommandline.h"
#include "tier1/utlbuffer.h"
#include "tier1/fmtstr.h"
#include "filesystem.h"
#include "characterset.h"
#include "isaverestore.h"
#include "gamerules.h"

//lwss - hardcode this generated nut file here. It will never change and there's no point in building it.
//#include "vscript_client_nut.h"
// Generated with texttoarray.pl - original file: vscript_client.nut
/*
function UniqueString( string = "" )
{
	return DoUniqueString( string.tostring() );
}

function IncludeScript( name, scope = null )
{
	if ( scope == null )
	{
		scope = this;
	}
	return ::DoIncludeScript( name, scope );
}
*/
static const unsigned char g_Script_vscript_client[] = {
        0x2f,0x2f,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x20,0x43,0x6f,0x70,0x79,0x72,0x69,0x67,
        0x68,0x74,0x20,0xa9,0x20,0x32,0x30,0x30,0x38,0x2c,0x20,0x56,0x61,0x6c,0x76,0x65,0x20,0x43,0x6f,0x72,
        0x70,0x6f,0x72,0x61,0x74,0x69,0x6f,0x6e,0x2c,0x20,0x41,0x6c,0x6c,0x20,0x72,0x69,0x67,0x68,0x74,0x73,
        0x20,0x72,0x65,0x73,0x65,0x72,0x76,0x65,0x64,0x2e,0x20,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x0a,
        0x2f,0x2f,0x0a,0x2f,0x2f,0x20,0x50,0x75,0x72,0x70,0x6f,0x73,0x65,0x3a,0x0a,0x2f,0x2f,0x0a,0x2f,0x2f,
        0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,
        0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,
        0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,
        0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x3d,0x0a,0x0a,0x66,
        0x75,0x6e,0x63,0x74,0x69,0x6f,0x6e,0x20,0x55,0x6e,0x69,0x71,0x75,0x65,0x53,0x74,0x72,0x69,0x6e,0x67,
        0x28,0x20,0x73,0x74,0x72,0x69,0x6e,0x67,0x20,0x3d,0x20,0x22,0x22,0x20,0x29,0x0a,0x7b,0x0a,0x09,0x72,
        0x65,0x74,0x75,0x72,0x6e,0x20,0x44,0x6f,0x55,0x6e,0x69,0x71,0x75,0x65,0x53,0x74,0x72,0x69,0x6e,0x67,
        0x28,0x20,0x73,0x74,0x72,0x69,0x6e,0x67,0x2e,0x74,0x6f,0x73,0x74,0x72,0x69,0x6e,0x67,0x28,0x29,0x20,
        0x29,0x3b,0x0a,0x7d,0x0a,0x0a,0x66,0x75,0x6e,0x63,0x74,0x69,0x6f,0x6e,0x20,0x49,0x6e,0x63,0x6c,0x75,
        0x64,0x65,0x53,0x63,0x72,0x69,0x70,0x74,0x28,0x20,0x6e,0x61,0x6d,0x65,0x2c,0x20,0x73,0x63,0x6f,0x70,
        0x65,0x20,0x3d,0x20,0x6e,0x75,0x6c,0x6c,0x20,0x29,0x0a,0x7b,0x0a,0x09,0x69,0x66,0x20,0x28,0x20,0x73,
        0x63,0x6f,0x70,0x65,0x20,0x3d,0x3d,0x20,0x6e,0x75,0x6c,0x6c,0x20,0x29,0x0a,0x09,0x7b,0x0a,0x09,0x09,
        0x73,0x63,0x6f,0x70,0x65,0x20,0x3d,0x20,0x74,0x68,0x69,0x73,0x3b,0x0a,0x09,0x7d,0x0a,0x09,0x72,0x65,
        0x74,0x75,0x72,0x6e,0x20,0x3a,0x3a,0x44,0x6f,0x49,0x6e,0x63,0x6c,0x75,0x64,0x65,0x53,0x63,0x72,0x69,
        0x70,0x74,0x28,0x20,0x6e,0x61,0x6d,0x65,0x2c,0x20,0x73,0x63,0x6f,0x70,0x65,0x20,0x29,0x3b,0x0a,0x7d,
        0x0a,0x00
};
//lwss end

#if defined ( PORTAL2 )
#include "usermessages.h"
#include "hud_macros.h"
#endif

extern IScriptManager *scriptmanager;
extern ScriptClassDesc_t * GetScriptDesc( CBaseEntity * );

// #define VMPROFILE 1

#ifdef VMPROFILE

#define VMPROF_START double debugStartTime = Plat_FloatTime();
#define VMPROF_SHOW( funcname, funcdesc  ) DevMsg("***VSCRIPT PROFILE***: %s %s: %6.4f milliseconds\n", (##funcname), (##funcdesc), (Plat_FloatTime() - debugStartTime)*1000.0 );

#else // !VMPROFILE

#define VMPROF_START
#define VMPROF_SHOW

#endif // VMPROFILE

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
static float Time()
{
	return gpGlobals->curtime;
}

static const char *GetMapName()
{
	return engine->GetLevelName();
}

static const char *DoUniqueString( const char *pszBase )
{
	static char szBuf[512];
	g_pScriptVM->GenerateUniqueKey( pszBase, szBuf, ARRAYSIZE(szBuf) );
	return szBuf;
}

bool DoIncludeScript( const char *pszScript, HSCRIPT hScope )
{
	if ( !VScriptRunScript( pszScript, hScope, true ) )
	{
		g_pScriptVM->RaiseException( CFmtStr( "Failed to include script \"%s\"", ( pszScript ) ? pszScript : "unknown" ) );
		return false;
	}
	return true;
}

int GetDeveloperLevel()
{
	return developer.GetInt();
}

bool VScriptClientInit()
{
	VMPROF_START

	if( scriptmanager != NULL )
	{
		ScriptLanguage_t scriptLanguage = SL_DEFAULT;

		char const *pszScriptLanguage;
		if ( CommandLine()->CheckParm( "-scriptlang", &pszScriptLanguage ) )
		{
			if( !Q_stricmp(pszScriptLanguage, "gamemonkey") )
			{
				scriptLanguage = SL_GAMEMONKEY;
			}
			else if( !Q_stricmp(pszScriptLanguage, "squirrel") )
			{
				scriptLanguage = SL_SQUIRREL;
			}
			else if( !Q_stricmp(pszScriptLanguage, "python") )
			{
				scriptLanguage = SL_PYTHON;
			}
			else
			{
				DevWarning("-scriptlang does not recognize a language named '%s'. virtual machine did NOT start.\n", pszScriptLanguage );
				scriptLanguage = SL_NONE;
			}

		}
		if( scriptLanguage != SL_NONE )
		{
			if ( g_pScriptVM == NULL )
				g_pScriptVM = scriptmanager->CreateVM( scriptLanguage );

			if( g_pScriptVM )
			{
				Log_Msg( LOG_VScript, "VSCRIPT: Started VScript virtual machine using script language '%s'\n", g_pScriptVM->GetLanguageName() );
				ScriptRegisterFunction( g_pScriptVM, GetMapName, "Get the name of the map.");
				ScriptRegisterFunction( g_pScriptVM, Time, "Get the current server time" );
				ScriptRegisterFunction( g_pScriptVM, DoIncludeScript, "Execute a script (internal)" );
				ScriptRegisterFunction( g_pScriptVM, GetDeveloperLevel, "Gets the level of 'develoer'" );
				
				if ( GameRules() )
				{
					GameRules()->RegisterScriptFunctions();
				}

				//g_pScriptVM->RegisterInstance( &g_ScriptEntityIterator, "Entities" );

				if ( scriptLanguage == SL_SQUIRREL )
				{
					g_pScriptVM->Run( g_Script_vscript_client );
				}

				VScriptRunScript( "mapspawn", false );

				VMPROF_SHOW( pszScriptLanguage, "virtual machine startup" );

				return true;
			}
			else
			{
				DevWarning("VM Did not start!\n");
			}
		}
	}
	else
	{
		Log_Msg( LOG_VScript, "\nVSCRIPT: Scripting is disabled.\n" );
	}
	g_pScriptVM = NULL;
	return false;
}

void VScriptClientTerm()
{
	if( g_pScriptVM != NULL )
	{
		if( g_pScriptVM )
		{
			scriptmanager->DestroyVM( g_pScriptVM );
			g_pScriptVM = NULL;
		}
	}
}


class CVScriptGameSystem : public CAutoGameSystemPerFrame
{
public:
	// Inherited from IAutoServerSystem
	virtual void LevelInitPreEntity( void )
	{
		// <sergiy> Note: we may need script VM garbage collection at this point in the future. Currently, VM does not persist 
		//          across level boundaries. GC is not necessary because our scripts are supposed to never create circular references
		//          and everything else is handled with ref counting. For the case of bugs creating circular references, the plan is to add
		//          diagnostics that detects such loops and warns the developer.

		m_bAllowEntityCreationInScripts = true;
		VScriptClientInit();
	}

	virtual void LevelInitPostEntity( void )
	{
		m_bAllowEntityCreationInScripts = false;
	}

	virtual void LevelShutdownPostEntity( void )
	{
		VScriptClientTerm();
	}

	virtual void FrameUpdatePostEntityThink() 
	{ 
		if ( g_pScriptVM )
			g_pScriptVM->Frame( gpGlobals->frametime );
	}

	bool m_bAllowEntityCreationInScripts;
};

CVScriptGameSystem g_VScriptGameSystem;

bool IsEntityCreationAllowedInScripts( void )
{
	return g_VScriptGameSystem.m_bAllowEntityCreationInScripts;
}

#if defined ( PORTAL2 )
void __MsgFunc_SetMixLayerTriggerFactor( bf_read &msg )
{
	char buf[MAX_PATH];

	msg.ReadString( buf, ARRAYSIZE( buf ), false );
	int iLayerID = engine->GetMixLayerIndex( buf );
	if ( iLayerID < 0 )
	{
		Warning( "Invalid mix layer passed to SetMixLayerTriggerFactor: '%s'\n", buf ); 
		return;
	}
	msg.ReadString( buf, ARRAYSIZE( buf ), false );
	int iGroupID = engine->GetMixGroupIndex( buf );
	if ( iGroupID < 0 )
	{
		Warning( "Invalid mix group passed to SetMixLayerTriggerFactor: '%s'\n", buf ); 
		return;
	}

	engine->SetMixLayerTriggerFactor( iLayerID, iGroupID, msg.ReadFloat() );
}

class CSetMixLayerTriggerHelper : public CAutoGameSystem 
{
	virtual bool Init()
	{
		for( int i = 0; i < MAX_SPLITSCREEN_PLAYERS; ++i )
		{
			ACTIVE_SPLITSCREEN_PLAYER_GUARD( i );
			HOOK_MESSAGE( SetMixLayerTriggerFactor );
		}
		return true;
	}
};

static CSetMixLayerTriggerHelper g_SetMixLayerTriggerHelper;
#endif