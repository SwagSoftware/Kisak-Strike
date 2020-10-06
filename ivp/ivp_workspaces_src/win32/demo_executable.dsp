# Microsoft Developer Studio Project File - Name="demo_executable" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Application" 0x0101

CFG=demo_executable - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "demo_executable.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "demo_executable.mak" CFG="demo_executable - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "demo_executable - Win32 SDK SingleThreaded" (based on "Win32 (x86) Application")
!MESSAGE "demo_executable - Win32 Debug" (based on "Win32 (x86) Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName "demo_executable"
# PROP Scc_LocalPath "..\.."
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "demo_executable - Win32 SDK SingleThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\obj\win32_sdk_singlethreaded\demo_executable"
# PROP Intermediate_Dir "..\..\obj\win32_sdk_singlethreaded\demo_executable"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /Ob2 /I "../../ivp_tools" /I "../../ivp_utility" /I "../../ivp_physics" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_collision" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /I "../../p_object" /I "../../p_main" /I "../../p_hardware" /I "../../p_renderer" /D "_DEBUG" /D "_WINDOWS" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /Fr /FD /c
# SUBTRACT CPP /YX /Yc /Yu
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /o "NUL" /win32
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib /nologo /subsystem:windows /machine:I386
# ADD LINK32 ddraw.lib d3drm.lib dsound.lib dinput.lib dxguid.lib gdi32.lib user32.lib winmm.lib /nologo /subsystem:windows /machine:I386 /out:"../../ipion_demo.exe"
# SUBTRACT LINK32 /profile /debug /nodefaultlib

!ELSEIF  "$(CFG)" == "demo_executable - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\obj\win32_debug\demo_executable"
# PROP Intermediate_Dir "..\..\obj\win32_debug\demo_executable"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /Zi /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /ZI /Od /I "../../ivp_utility" /I "../../ivp_physics" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_collision" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /I "../../../havana/havok" /D "_WINDOWS" /D "DEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /FR /FD /c
# SUBTRACT CPP /YX /Yc /Yu
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /o "NUL" /win32
# ADD BASE RSC /l 0x407 /d "_DEBUG"
# ADD RSC /l 0x407 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:windows /debug /machine:I386 /pdbtype:sept
# ADD LINK32 ddraw.lib d3drm.lib dsound.lib dinput.lib dxguid.lib gdi32.lib user32.lib winmm.lib /nologo /subsystem:windows /profile /map /debug /machine:I386 /out:"..\..\ipion_demo_d.exe"

!ENDIF 

# Begin Target

# Name "demo_executable - Win32 SDK SingleThreaded"
# Name "demo_executable - Win32 Debug"
# Begin Group "IVP_SAMPLES"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Ivp_sample_sources\ive_example_black_hole.cxx
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

SOURCE=..\..\IVP_SAMPLE_SOURCES\ive_example_spacefield.cxx
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
