project "vphysics"

language "C++"

kind "SharedLib"

-- Visual studio specific copy command
configuration { "windows", "vs*" }
	if GEN_POSTBUILDCOMMANDS then
		postbuildcommands {
			   'if defined VPHYSICS_GAME_PATH (\n'
			.. ' if exist "%VPHYSICS_GAME_PATH%\bin\$(TargetFileName)" (\n'
			.. '  attrib -r "%VPHYSICS_GAME_PATH%\bin\$(TargetFileName)"\n'
			.. '  del "%VPHYSICS_GAME_PATH%\bin\$(TargetFileName)"\n'
			.. ' )\n'
			.. ' \n'
			.. ' copy "$(TargetPath)" "%VPHYSICS_GAME_PATH%\bin\$(TargetFileName)"\n'
			.. ')\n'
		}
	end

	linkoptions { "/NODEFAULTLIB:\"LIBCMT\"" }

-- Part of a rediculous hack to get source sdk to build
configuration { "linux", "gmake" }
	targetname "vphysics_srv"
	targetprefix ""
	buildoptions { "-w", "-fpermissive" }
	defines { "sprintf_s=snprintf", "strcmpi=strcasecmp", "_alloca=alloca", "stricmp=strcasecmp", "_stricmp=strcasecmp", "strcpy_s=strncpy", "_strnicmp=strncasecmp", "strnicmp=strncasecmp", "_snprintf=snprintf", "_vsnprintf=vsnprintf", "_alloca=alloca", "strcmpi=strcasecmp", "NO_MALLOC_OVERRIDE" }

configuration {}

links { "BulletCollision", "BulletDynamics", "LinearMath", "BulletSoftBody", "BulletMultiThreaded" }

if _PREMAKE_VERSION == "4.4" then
	vpaths {
		["Header Files"] = "**.h",
		["Source Files"] = "**.cpp",
		["Resource Files"] = {"**.rc", "resource.h"},
	}
end

includedirs {
	SDK_DIR,
	SDK_DIR .. "/public",
	SDK_DIR .. "/public/tier0",
	SDK_DIR .. "/public/tier1",
	"../bullet/src",
	"../include"
}

configuration { "windows" }
	libdirs {
		SDK_DIR .. "/lib/public",
	}
	
	links { "tier0", "tier1", "tier2", "vstdlib", "mathlib" }
	
configuration { "linux", "gmake" }
	-- SRCDS_BIN_DIR is nil on windows, and this code runs anyways :(
	if os.is("linux") then
		libdirs {
			SDK_DIR .. "/lib/linux",
			SRCDS_BIN_DIR,
		}
		
		-- GCC is terrible with static vs. dynamic
		-- FIXME: This doesn't work at all!
		linkoptions {
			"-static",
			"\"" .. path.getabsolute(SDK_DIR) .. "/lib/linux/libtier1_i486.a\"",
			"\"" .. path.getabsolute(SDK_DIR) .. "/lib/linux/libmathlib_i486.a\"",
			"\"" .. path.getabsolute(SRCDS_BIN_DIR) .. "/libtier0_srv.so\"",
			"\"" .. path.getabsolute(SRCDS_BIN_DIR) .. "/libvstdlib_srv.so\"",
		}
	end

configuration {}
	
files {
	"**.cpp",
	"**.h"
}