# Microsoft Developer Studio Project File - Name="internal_project_nogfx" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=internal_project_nogfx - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "internal_project_nogfx.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "internal_project_nogfx.mak" CFG="internal_project_nogfx - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "internal_project_nogfx - Win32 SDK SingleThreaded" (based on "Win32 (x86) Console Application")
!MESSAGE "internal_project_nogfx - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName "internal_project_nogfx"
# PROP Scc_LocalPath "..\.."
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "internal_project_nogfx - Win32 SDK SingleThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "c:\TEMP\IpionEngine\internal_project_nogfx\SDK_SingleThreaded\"
# PROP Intermediate_Dir "c:\TEMP\IpionEngine\internal_project_nogfx\SDK_SingleThreaded\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /I "../../ivp_qhull" /I "../../ivp_utility" /I "../../ivp_physics" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_collision" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /D "_WINDOWS" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /YX /FD /c
# SUBTRACT CPP /Fr
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386
# ADD LINK32 winmm.lib user32.lib gdi32.lib /nologo /subsystem:console /machine:I386 /out:"..\..\ipion_demo_internal_nogfx.exe"

!ELSEIF  "$(CFG)" == "internal_project_nogfx - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "c:\TEMP\IpionEngine\internal_project_nogfx\Debug\"
# PROP Intermediate_Dir "c:\TEMP\IpionEngine\internal_project_nogfx\Debug\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /GZ /c
# ADD CPP /nologo /W3 /Gm /GX /ZI /Od /I "../../ivp_utility" /I "../../ivp_physics" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_collision" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /I "../../../havana/havok" /D "_WINDOWS" /D "DEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /FR /Yu"ivp_physics.hxx" /FD /GZ /c
# ADD BASE RSC /l 0x407 /d "_DEBUG"
# ADD RSC /l 0x407 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# ADD LINK32 winmm.lib user32.lib gdi32.lib /nologo /subsystem:console /debug /machine:I386 /out:"c:\TEMP\IpionEngine\internal_project_nogfx\Debug\ipion_internal_nogfx_DEBUG.exe" /pdbtype:sept
# SUBTRACT LINK32 /pdb:none

!ENDIF 

# Begin Target

# Name "internal_project_nogfx - Win32 SDK SingleThreaded"
# Name "internal_project_nogfx - Win32 Debug"
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
# Begin Group "IVP_SAMPLES_PROTECTED"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_protected\ive_example_asteroids.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_protected\ive_example_subparts.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_sample_sources_protected\ive_quickstart_attach.cxx
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
# Begin Source File

SOURCE=..\..\IVP_SAMPLE_SOURCES\ive_nogfx.cxx
# End Source File
# End Target
# End Project
