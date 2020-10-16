# Microsoft Developer Studio Project File - Name="hull_editor" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Application" 0x0101

CFG=hull_editor - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "hull_editor.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "hull_editor.mak" CFG="hull_editor - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "hull_editor - Win32 Debug" (based on "Win32 (x86) Application")
!MESSAGE "hull_editor - Win32 SDK SingleThreaded" (based on "Win32 (x86) Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName "hull_editor"
# PROP Scc_LocalPath "..\.."
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "hull_editor - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\obj\win32_debug\hull_editor"
# PROP Intermediate_Dir "..\..\obj\win32_debug\hull_editor"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /Zi /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /ZI /Od /I "../../ivp_geompack" /I "../../ivp_tools_internal" /I "../../ivp_collision" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_qhull" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /I "../../ivp_tools" /I "../../ivp_utility" /I "../../p_main" /I "../../p_object" /I "../../p_renderer" /I "../../p_directx" /I "../../p_hardware" /D "_WINDOWS" /D "DEBUG" /D "WIN32" /D "IVP_VERSION_EVAL" /FR /YX /FD /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /o "NUL" /win32
# ADD BASE RSC /l 0x407 /d "_DEBUG"
# ADD RSC /l 0x407 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /debug /machine:I386 /pdbtype:sept
# ADD LINK32 ddraw.lib d3drm.lib dsound.lib dinput.lib dxguid.lib gdi32.lib user32.lib winmm.lib /nologo /subsystem:windows /profile /map /debug /machine:I386 /out:"../../hull_editor_d.exe"

!ELSEIF  "$(CFG)" == "hull_editor - Win32 SDK SingleThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "hull_editor___Win32_SDK_SingleThreaded"
# PROP BASE Intermediate_Dir "hull_editor___Win32_SDK_SingleThreaded"
# PROP BASE Ignore_Export_Lib 0
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\obj\win32_sdk_singlethreaded\hull_editor"
# PROP Intermediate_Dir "..\..\obj\win32_sdk_singlethreaded\hull_editor"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /Ob2 /I "../../ivp_tools_internal" /I "../../ivp_collision" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_qhull" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /I "../../ivp_tools" /I "../../ivp_utility" /I "../../p_main" /I "../../p_object" /I "../../p_renderer" /I "../../p_directx" /I "../../p_hardware" /D "_WINDOWS" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /Fr /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /Ob2 /I "../../ivp_tools_internal" /I "../../ivp_collision" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_qhull" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /I "../../ivp_tools" /I "../../ivp_utility" /I "../../p_main" /I "../../p_object" /I "../../p_renderer" /I "../../p_directx" /I "../../p_hardware" /D "_WINDOWS" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /Fr /YX /FD /c
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /o "NUL" /win32
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 ddraw.lib d3drm.lib dsound.lib dinput.lib dxguid.lib gdi32.lib user32.lib winmm.lib /nologo /subsystem:windows /pdb:"Debug/hull_editor.pdb" /machine:I386 /out:"../../hull_editor.exe" /pdbtype:sept
# SUBTRACT BASE LINK32 /pdb:none /nodefaultlib
# ADD LINK32 ddraw.lib d3drm.lib dsound.lib dinput.lib dxguid.lib gdi32.lib user32.lib winmm.lib /nologo /subsystem:windows /pdb:"Debug/hull_editor.pdb" /machine:I386 /out:"../../hull_editor.exe" /pdbtype:sept
# SUBTRACT LINK32 /pdb:none /nodefaultlib

!ENDIF 

# Begin Target

# Name "hull_editor - Win32 Debug"
# Name "hull_editor - Win32 SDK SingleThreaded"
# Begin Group "IVP_TOOLS"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Ivp_tools\hull_editor.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_tools\ivp_convex_hull_splitter.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_tools\ivp_hull_editor.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_tools\ivp_hull_splitter_manager.cxx
# End Source File
# End Group
# Begin Source File

SOURCE=..\..\IVP_SAMPLE_SOURCES\ive_example_common.cxx
# End Source File
# End Target
# End Project
