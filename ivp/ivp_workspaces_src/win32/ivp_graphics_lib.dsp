# Microsoft Developer Studio Project File - Name="ivp_graphics.lib" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=IVP_GRAPHICS.LIB - WIN32 DEBUG
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "ivp_graphics_lib.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "ivp_graphics_lib.mak" CFG="IVP_GRAPHICS.LIB - WIN32 DEBUG"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "ivp_graphics.lib - Win32 SDK SingleThreaded" (based on "Win32 (x86) Static Library")
!MESSAGE "ivp_graphics.lib - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName "ivp_graphics_lib"
# PROP Scc_LocalPath "..\.."
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "ivp_graphics.lib - Win32 SDK SingleThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\obj\win32_sdk_singlethreaded\ivp_graphics"
# PROP Intermediate_Dir "..\..\obj\win32_sdk_singlethreaded\ivp_graphics"
# PROP Target_Dir ""
MTL=midl.exe
LINK32=link.exe
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /I "../../ivp_collision" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_qhull" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /I "../../ivp_tools" /I "../../ivp_utility" /I "../../p_main" /I "../../p_object" /I "../../p_renderer" /I "../../p_directx" /I "../../p_hardware" /D "_MBCS" /D "_LIB" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /Yu"ivp_physics.hxx" /FD /c
# SUBTRACT CPP /Fr
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"../../ivp_library/win32_sdk_singlethreaded/ivp_graphics.lib"

!ELSEIF  "$(CFG)" == "ivp_graphics.lib - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\obj\win32_debug\ivp_graphics"
# PROP Intermediate_Dir "..\..\obj\win32_debug\ivp_graphics"
# PROP Target_Dir ""
MTL=midl.exe
LINK32=link.exe
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /W3 /Gm /GX /ZI /Od /I "../../ivp_collision" /I "../../ivp_compact_builder" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_qhull" /I "../../ivp_sample_sources" /I "../../ivp_surface_manager" /I "../../ivp_tools" /I "../../ivp_utility" /I "../../p_main" /I "../../p_object" /I "../../p_renderer" /I "../../p_directx" /I "../../p_hardware" /D "_MBCS" /D "_LIB" /D "DEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /FR /FD /GZ /c
# SUBTRACT CPP /YX /Yc /Yu
# ADD BASE RSC /l 0x407 /d "_DEBUG"
# ADD RSC /l 0x407 /i "../../dxguid.lib dsound.lib d3drm.lib ddraw.lib" /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"../../ivp_library/win32_debug/ivp_graphics_d.lib"

!ENDIF 

# Begin Target

# Name "ivp_graphics.lib - Win32 SDK SingleThreaded"
# Name "ivp_graphics.lib - Win32 Debug"
# Begin Group "P_SYSTEM"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\P_main\P_2d_memory_manager.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_actuator.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_actuator_samples.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_blur.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_camera.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_coll_event.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_color.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_control.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\p_custom_camera.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_main\P_directory.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_main\P_etc.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\p_float.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_main\P_font.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_main\P_game.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_main\p_graphlib_old_demos.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_levels.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_material.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_main\P_menu.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_main\P_modules.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object_after_display.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object_ball.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object_edit.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object_func.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object_func2.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object_func3.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object_light.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object_polygon.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object_polygon_check.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object_polygon_display.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object_polygon_edit.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_object_sweep.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\p_os_dep.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_main\P_own_menus.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_parse.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_main\P_properties.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\p_public_graphics_intf.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_scan.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_main\P_smooth.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_stapel.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_sweep.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_sweep_debug.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_sweep_fast.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_sweep_shadow.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_main\P_task.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_texture.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_texture_manager.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_trafo_ball.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_trafo_texture.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_trafo_texture_fast.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_object\P_worm.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\P_xstruktur.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\readGIF.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_renderer\writeGIF.cxx
# End Source File
# End Group
# Begin Group "P_HARDWARE_ABSTRACT"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\P_hardware\P_hardware.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_hardware\P_hardware_pic.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_hardware\P_in_dev.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_hardware\P_sound.cxx
# End Source File
# End Group
# Begin Group "P_HARDWARE_DIRECTX"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\P_directx\P_directx.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_directx\P_directx_listener.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_directx\P_directx_sound.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_directx\P_directx_utils.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_directx\p_dx_clustering_visua.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_directx\p_dx_stat_manager_ovl.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_directx\P_hardware_directx.cxx
# End Source File
# Begin Source File

SOURCE=..\..\P_directx\p_hardware_directx_window.cxx
# End Source File
# End Group
# Begin Source File

SOURCE=..\..\P_main\main.cxx
# ADD CPP /Yc"ivp_physics.hxx"
# End Source File
# End Target
# End Project
