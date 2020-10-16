# Microsoft Developer Studio Project File - Name="ivp_physics.lib" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=ivp_physics.lib - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "ivp_physics_lib.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "ivp_physics_lib.mak" CFG="ivp_physics.lib - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "ivp_physics.lib - Win32 SDK SingleThreaded" (based on "Win32 (x86) Static Library")
!MESSAGE "ivp_physics.lib - Win32 SDK MultiThreaded" (based on "Win32 (x86) Static Library")
!MESSAGE "ivp_physics.lib - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "ivp_physics.lib - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName "ivp_physics_lib"
# PROP Scc_LocalPath "..\.."
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "ivp_physics.lib - Win32 SDK SingleThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\obj\win32_sdk_singlethreaded\ivp_physics"
# PROP Intermediate_Dir "..\..\obj\win32_sdk_singlethreaded\ivp_physics"
# PROP Target_Dir ""
MTL=midl.exe
LINK32=link.exe
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /I "../../ivp_collision" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_surface_manager" /I "../../ivp_utility" /I "../../../havok/source" /I "../../../havana/havok" /D "NDEBUG" /D "_MBCS" /D "_LIB" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVANA_CONSTRAINTS" /D "HAVOK_MOPP" /Yu"ivp_physics.hxx" /FD /c
# SUBTRACT CPP /Fr
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\ivp_library\win32_sdk_singlethreaded\ivp_physics.lib"

!ELSEIF  "$(CFG)" == "ivp_physics.lib - Win32 SDK MultiThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\obj\win32_sdk_multithreaded\ivp_physics"
# PROP Intermediate_Dir "..\..\obj\win32_sdk_multithreaded\ivp_physics"
# PROP Target_Dir ""
MTL=midl.exe
LINK32=link.exe
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /MT /W3 /GX /O2 /I "../../ivp_collision" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_surface_manager" /I "../../ivp_utility" /D "NDEBUG" /D "_MBCS" /D "_LIB" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVANA_CONSTRAINTS" /D "HAVOK_MOPP" /Yu"ivp_physics.hxx" /FD /c
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\ivp_library\win32_sdk_multithreaded\ivp_physics_mt.lib"

!ELSEIF  "$(CFG)" == "ivp_physics.lib - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\obj\win32_debug\ivp_physics"
# PROP Intermediate_Dir "..\..\obj\win32_debug\ivp_physics"
# PROP Target_Dir ""
MTL=midl.exe
LINK32=link.exe
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /W3 /Gm /GX /ZI /Od /I "../../ivp_collision" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_surface_manager" /I "../../ivp_utility" /D "DEBUG" /D "HAVOK_MOPP" /D "_MBCS" /D "_LIB" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVANA_CONSTRAINTS" /FR /FD /GZ /c
# SUBTRACT CPP /YX /Yc /Yu
# ADD BASE RSC /l 0x407 /d "_DEBUG"
# ADD RSC /l 0x407 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\ivp_library\win32_debug\ivp_physics_d.lib"
# Begin Custom Build
TargetPath=\dev\src_main\ivp\ivp_library\win32_debug\ivp_physics_d.lib
InputPath=\dev\src_main\ivp\ivp_library\win32_debug\ivp_physics_d.lib
SOURCE="$(InputPath)"

"..\..\..\lib\common\ivp_physics.lib" : $(SOURCE) "$(INTDIR)" "$(OUTDIR)"
	if exist ..\..\..\lib\common\ivp_physics.lib attrib -r ..\..\..\lib\common\ivp_physics.lib 
	copy $(TargetPath) ..\..\..\lib\common\ivp_physics.lib 
	
# End Custom Build

!ELSEIF  "$(CFG)" == "ivp_physics.lib - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "ivp_physics_lib___Win32_Release"
# PROP BASE Intermediate_Dir "ivp_physics_lib___Win32_Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\obj\win32_release\ivp_physics"
# PROP Intermediate_Dir "..\..\obj\win32_release\ivp_physics"
# PROP Target_Dir ""
MTL=midl.exe
LINK32=link.exe
# ADD BASE CPP /nologo /W3 /GX /O2 /I "../../ivp_collision" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_surface_manager" /I "../../ivp_utility" /D "_MBCS" /D "_LIB" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /YX /FD /c
# SUBTRACT BASE CPP /Fr
# ADD CPP /nologo /W3 /GX /O2 /I "../../ivp_collision" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_surface_manager" /I "../../ivp_utility" /D "NDEBUG" /D "_MBCS" /D "_LIB" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVANA_CONSTRAINTS" /D "HAVOK_MOPP" /Fr /Yu"ivp_physics.hxx" /FD /c
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo /out:"../../IVP_Library/ivp_physics.lib"
# ADD LIB32 /nologo /out:"..\..\ivp_library\win32_release\ivp_physics.lib"
# Begin Custom Build
TargetPath=\dev\src_main\ivp\ivp_library\win32_release\ivp_physics.lib
InputPath=\dev\src_main\ivp\ivp_library\win32_release\ivp_physics.lib
SOURCE="$(InputPath)"

"..\..\..\lib\common\ivp_physics.lib" : $(SOURCE) "$(INTDIR)" "$(OUTDIR)"
	if exist ..\..\..\lib\common\ivp_physics.lib attrib -r ..\..\..\lib\common\ivp_physics.lib 
	copy $(TargetPath) ..\..\..\lib\common\ivp_physics.lib 
	
# End Custom Build

!ENDIF 

# Begin Target

# Name "ivp_physics.lib - Win32 SDK SingleThreaded"
# Name "ivp_physics.lib - Win32 SDK MultiThreaded"
# Name "ivp_physics.lib - Win32 Debug"
# Name "ivp_physics.lib - Win32 Release"
# Begin Group "IVP_COLLISION"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_3d_solver.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_3d_solver.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_cache_ledge_point.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_clustering_longrange.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_clustering_longrange.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_clustering_lrange_hash.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_clustering_lrange_hash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_clustering_visual_hash.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_clustering_visual_hash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_clustering_visualizer.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_clustering_visualizer.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_coll_del_root_mindist.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_collision.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_collision_filter.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_collision_filter.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_compact_ledge.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_compact_ledge.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_compact_ledge_solver.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_compact_ledge_solver.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_i_collision_vhash.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_i_collision_vhash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_listener_collision.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_mindist.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_mindist.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_mindist_debug.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_mindist_event.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_mindist_event.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_mindist_intern.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_mindist_macros.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_mindist_mcases.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_mindist_minimize.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_mindist_minimize.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_mindist_recursive.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_oo_watcher.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_range_manager.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_range_manager.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_collision\ivp_ray_solver.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_ray_solver.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COLLISION\ivp_universe_manager.hxx
# End Source File
# End Group
# Begin Group "IVP_CONTROLLER"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_actuator.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_actuator.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_actuator_info.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_actuator_spring.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_actuator_spring.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_attacher_to_cores.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_buoyancy_solver.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_buoyancy_solver.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_car_system.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_car_system.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_constraint.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_constraint.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_constraint_car.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_constraint_car.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_constraint_fixed_keyed.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_constraint_fixed_keyed.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_constraint_local.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_constraint_local.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_constraint_types.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_controller.hxx
# End Source File
# Begin Source File

SOURCE=..\..\ivp_controller\ivp_controller_airboat.cpp
# End Source File
# Begin Source File

SOURCE=..\..\ivp_controller\ivp_controller_airboat.h
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_controller_buoyancy.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_controller_buoyancy.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_controller_floating.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_controller_floating.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_controller_golem.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_controller_motion.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_controller_motion.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_controller_raycast_car.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_controller_raycast_car.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_controller_stiff_spring.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_controller_stiff_spring.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_controller_world_frict.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_controller_world_frict.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_forcefield.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_forcefield.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_multidimensional_interp.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_multidimensional_interp.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_controller\ivp_template_constraint.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_CONTROLLER\ivp_template_constraint.hxx
# End Source File
# End Group
# Begin Group "IVP_INTERN"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_ball.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_calc_next_psi_solver.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_INTERN\ivp_calc_next_psi_solver.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_controller_phantom.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_core.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_environment.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_friction.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_INTERN\ivp_friction.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_friction_gaps.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_INTERN\ivp_friction_solver.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_great_matrix.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_hull_manager.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_INTERN\ivp_hull_manager_macros.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_INTERN\ivp_i_controller_vhash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_i_friction_hash.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_INTERN\ivp_i_friction_hash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_i_object_vhash.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_INTERN\ivp_i_object_vhash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_impact.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_INTERN\ivp_impact.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_merge_core.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_INTERN\ivp_merge_core.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_mindist_friction.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_object.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_object_attach.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_physic.cxx
# ADD CPP /Yc"ivp_physics.hxx"
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_physic_private.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_INTERN\ivp_physic_private.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_polygon.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_sim_unit.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_INTERN\ivp_sim_unit.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_solver_core_reaction.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_intern\ivp_time.cxx
# End Source File
# End Group
# Begin Group "IVP_PHYSICS"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ive_graphics.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_physics\ivp_anomaly_manager.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_anomaly_manager.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_auth_empty.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_authenticity.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_ball.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_physics\ivp_betterdebugmanager.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_betterdebugmanager.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_physics\ivp_betterstatisticsmanager.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_betterstatisticsmanager.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_physics\ivp_cache_object.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_cache_object.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_contact_situation.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_core.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_core_macros.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_debug.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_debug_manager.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_environment.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_great_matrix.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_hull_manager.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_physics\ivp_liquid_surface_descript.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_liquid_surface_descript.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_listener_hull.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_listener_object.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_listener_psi.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_physics\ivp_material.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_material.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_object.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_object_attach.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_physics\ivp_performancecounter.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_performancecounter.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_phantom.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_physics.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_polygon.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_radar.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_radar_appl.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_real_object.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_solver_core_reaction.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_physics\ivp_stat_manager_cback_con.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_stat_manager_cback_con.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_physics\ivp_surface_manager.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_surface_manager.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_physics\ivp_templates.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_templates.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_time.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\ivp_time_event.hxx
# End Source File
# End Group
# Begin Group "IVP_SURFACEMANAGER"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\IVP_SURFACE_MANAGER\ivp_compact_grid.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_SURFACE_MANAGER\ivp_compact_surface.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_SURFACE_MANAGER\ivp_compact_surface.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_surface_manager\ivp_gridbuild_array.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_SURFACE_MANAGER\ivp_gridbuild_array.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_surface_manager\ivp_surman_grid.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_SURFACE_MANAGER\ivp_surman_grid.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_surface_manager\ivp_surman_polygon.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_SURFACE_MANAGER\ivp_surman_polygon.hxx
# End Source File
# End Group
# Begin Group "IVP_UTILITY"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_active_value.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_active_value.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_active_value_hash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_bigvector.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_bigvector.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_diff_hash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_float.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_fvector.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_geometry.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_geometry.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_hash.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_hash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_linear.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_linear.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_linear_double.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_linear_macros.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_linear_PIII.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_linear_ps2.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_linear_software.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_linear_willamette.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_list.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_mapping.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_matrix_macros.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_memory.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_memory.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_min_hash.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_min_hash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_min_list.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_min_list.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_os_dep.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_ps2.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_quat.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_quat.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_set.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_string.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_string.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_string_hash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_types.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_types.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_vector.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_vector.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_utility\ivu_vhash.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_UTILITY\ivu_vhash.hxx
# End Source File
# End Group
# End Target
# End Project
