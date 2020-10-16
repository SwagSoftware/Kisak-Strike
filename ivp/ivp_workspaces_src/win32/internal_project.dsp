# Microsoft Developer Studio Project File - Name="internal_project" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Application" 0x0101

CFG=internal_project - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "internal_project.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "internal_project.mak" CFG="internal_project - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "internal_project - Win32 EVAL SingleThreaded" (based on "Win32 (x86) Application")
!MESSAGE "internal_project - Win32 SDK SingleThreaded" (based on "Win32 (x86) Application")
!MESSAGE "internal_project - Win32 SDK MultiThreaded" (based on "Win32 (x86) Application")
!MESSAGE "internal_project - Win32 Debug" (based on "Win32 (x86) Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName "internal_project"
# PROP Scc_LocalPath "..\.."
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "internal_project - Win32 EVAL SingleThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "c:\TEMP\IpionEngine\internal_project\EVAL_SingleThreaded\"
# PROP Intermediate_Dir "c:\TEMP\IpionEngine\internal_project\EVAL_SingleThreaded\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /Ob2 /I "../../ivp_collision" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_qhull" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /I "../../ivp_tools" /I "../../ivp_utility" /I "../../p_main" /I "../../p_object" /I "../../p_renderer" /I "../../p_directx" /I "../../p_hardware" /I "../../../havana/havok" /D "_WINDOWS" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_EVAL" /Fr /Yu"ivp_physics.hxx" /FD /c
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /o "NUL" /win32
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib /nologo /subsystem:windows /machine:I386
# ADD LINK32 ddraw.lib d3drm.lib dsound.lib dinput.lib dxguid.lib gdi32.lib user32.lib winmm.lib /nologo /subsystem:windows /pdb:"Debug/internal_project.pdb" /machine:I386 /out:"../../ipion_demo_internal_eval.exe" /pdbtype:sept
# SUBTRACT LINK32 /verbose /pdb:none /incremental:yes /nodefaultlib

!ELSEIF  "$(CFG)" == "internal_project - Win32 SDK SingleThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "c:\TEMP\IpionEngine\internal_project\SDK_SingleThreaded\"
# PROP Intermediate_Dir "c:\TEMP\IpionEngine\internal_project\SDK_SingleThreaded\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /Ob2 /I "../../ivp_collision" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_qhull" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /I "../../ivp_tools" /I "../../ivp_utility" /I "../../p_main" /I "../../p_object" /I "../../p_renderer" /I "../../p_directx" /I "../../p_hardware" /I "../../../havana/havok" /I "../../havok_bvm" /D "_WINDOWS" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVANA" /Fr /Yu"ivp_physics.hxx" /FD /c
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /o "NUL" /win32
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib /nologo /subsystem:windows /machine:I386
# ADD LINK32 ddraw.lib d3drm.lib dsound.lib dinput.lib dxguid.lib gdi32.lib user32.lib winmm.lib /nologo /subsystem:windows /pdb:"Debug/internal_project.pdb" /machine:I386 /out:"../../ipion_demo_internal.exe" /pdbtype:sept
# SUBTRACT LINK32 /verbose /profile /pdb:none /incremental:yes /nodefaultlib

!ELSEIF  "$(CFG)" == "internal_project - Win32 SDK MultiThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "c:\TEMP\IpionEngine\internal_project\SDK_MultiThreaded\"
# PROP Intermediate_Dir "c:\TEMP\IpionEngine\internal_project\SDK_MultiThreaded\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /MD /W3 /GX /O2 /Ob2 /I "../../ivp_collision" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_qhull" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /I "../../ivp_tools" /I "../../ivp_utility" /I "../../p_main" /I "../../p_object" /I "../../p_renderer" /I "../../p_directx" /I "../../p_hardware" /I "../../../havana/havok" /D "_WINDOWS" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /Yu"ivp_physics.hxx" /FD /c
# SUBTRACT CPP /Fr
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /o "NUL" /win32
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib /nologo /subsystem:windows /machine:I386
# ADD LINK32 ddraw.lib d3drm.lib dsound.lib dinput.lib dxguid.lib gdi32.lib user32.lib winmm.lib /nologo /subsystem:windows /pdb:"Debug/internal_project.pdb" /machine:I386 /nodefaultlib:"libc.lib" /out:"../../ipion_demo_internal_mt.exe" /pdbtype:sept
# SUBTRACT LINK32 /verbose /pdb:none /incremental:yes /nodefaultlib

!ELSEIF  "$(CFG)" == "internal_project - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "c:\TEMP\IpionEngine\internal_project\Debug\"
# PROP Intermediate_Dir "c:\TEMP\IpionEngine\internal_project\Debug\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /Zi /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /ZI /Od /I "../../ivp_geompack" /I "../../ivp_collision" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_qhull" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /I "../../ivp_tools" /I "../../ivp_utility" /I "../../p_main" /I "../../p_object" /I "../../p_renderer" /I "../../p_directx" /I "../../p_hardware" /I "../../../havana/havok" /D "_WINDOWS" /D "DEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVANA" /FR /Yu"ivp_physics.hxx" /FD /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /o "NUL" /win32
# ADD BASE RSC /l 0x407 /d "_DEBUG"
# ADD RSC /l 0x407 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /debug /machine:I386 /pdbtype:sept
# ADD LINK32 ddraw.lib d3drm.lib dsound.lib dinput.lib dxguid.lib gdi32.lib user32.lib winmm.lib /nologo /subsystem:windows /profile /map /debug /machine:I386 /out:"c:\TEMP\IpionEngine\internal_project\Debug\ipion_demo_internal_DEBUG.exe"

!ENDIF 

# Begin Target

# Name "internal_project - Win32 EVAL SingleThreaded"
# Name "internal_project - Win32 SDK SingleThreaded"
# Name "internal_project - Win32 SDK MultiThreaded"
# Name "internal_project - Win32 Debug"
# Begin Group "IVP_SAMPLES"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_black_hole.cxx
# ADD CPP /Yc"ivp_physics.hxx"
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_blackhole2.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_boat.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_body.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_bridge.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_bsp2ics.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_bsptree.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_city.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_common.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_creature.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_ctrl_raycast.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_domino.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_floating.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_flyer.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_forcefld_funnel.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_objectpile.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_onthefly.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_paper.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_phantom1.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_phantom2.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_polygon_soup.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_shootingranch.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_snowflake.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_stiff_creature.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_tankwar.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_terrain.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_turn_up.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_universe.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_vehicle.cxx
# End Source File
# End Group
# Begin Group "IVP_SAMPLES_QUICKSTARTS"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_act_force.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_act_sprng.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_active_value.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_ball.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_buoyancy.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_callback.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_check_dist.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_coll_filter.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_collision.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_cube.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_cut.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_force.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_SAMPLE_SOURCES_QUICKSTARTS\ive_quickstart_golem_cntr.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_hinge.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_motion_cntrl.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_raycast.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_ring.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_rope.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_rotmot.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_ski.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_quickstarts\ive_quickstart_stiff_spring.cxx
# End Source File
# End Group
# End Target
# End Project
