# Microsoft Developer Studio Project File - Name="havana_constraints" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=havana_constraints - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "havana_constraints.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "havana_constraints.mak" CFG="havana_constraints - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "havana_constraints - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "havana_constraints - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "havana_constraints - Win32 SDK SingleThreaded" (based on "Win32 (x86) Static Library")
!MESSAGE "havana_constraints - Win32 SDK MultiThreaded" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName "havana_constraints"
# PROP Scc_LocalPath "."
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "havana_constraints - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\obj\win32_release\havana_constraints"
# PROP Intermediate_Dir "..\..\obj\win32_release\havana_constraints"
# PROP Target_Dir ""
LINK32=link.exe
MTL=midl.exe
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /W3 /O2 /I "../../ivp_collision" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_surface_manager" /I "../../ivp_utility" /I "../../havana/havok" /D "_WINDOWS" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVANA_CONSTRAINTS" /FR /FD /c
# SUBTRACT CPP /YX /Yc /Yu
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\ivp_library\win32_release\havana_constraints.lib"
# Begin Custom Build
TargetPath=\dev\src_main\ivp\ivp_library\win32_release\havana_constraints.lib
InputPath=\dev\src_main\ivp\ivp_library\win32_release\havana_constraints.lib
SOURCE="$(InputPath)"

"..\..\..\lib\common\havana_constraints.lib" : $(SOURCE) "$(INTDIR)" "$(OUTDIR)"
	if exist ..\..\..\lib\common\havana_constraints.lib attrib -r ..\..\..\lib\common\havana_constraints.lib 
	copy $(TargetPath) ..\..\..\lib\common\havana_constraints.lib 
	
# End Custom Build

!ELSEIF  "$(CFG)" == "havana_constraints - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\obj\win32_debug\havana_constraints"
# PROP Intermediate_Dir "..\..\obj\win32_debug\havana_constraints"
# PROP Target_Dir ""
LINK32=link.exe
MTL=midl.exe
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /W3 /Gm /ZI /Od /I "../../ivp_collision" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_surface_manager" /I "../../ivp_utility" /I "../../havana/havok" /D "_WINDOWS" /D "DEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVANA_CONSTRAINTS" /FR /FD /GZ /c
# SUBTRACT CPP /YX /Yc /Yu
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\ivp_library\win32_debug\havana_constraints_d.lib"
# Begin Custom Build
TargetPath=\dev\src_main\ivp\ivp_library\win32_debug\havana_constraints_d.lib
InputPath=\dev\src_main\ivp\ivp_library\win32_debug\havana_constraints_d.lib
SOURCE="$(InputPath)"

"..\..\..\lib\common\havana_constraints.lib" : $(SOURCE) "$(INTDIR)" "$(OUTDIR)"
	if exist ..\..\..\lib\common\havana_constraints.lib attrib -r ..\..\..\lib\common\havana_constraints.lib 
	copy $(TargetPath) ..\..\..\lib\common\havana_constraints.lib 
	
# End Custom Build

!ELSEIF  "$(CFG)" == "havana_constraints - Win32 SDK SingleThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "havana_constraints___Win32_SDK_SingleThreaded"
# PROP BASE Intermediate_Dir "havana_constraints___Win32_SDK_SingleThreaded"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\obj\win32_sdk_singlethreaded\havana_constraints"
# PROP Intermediate_Dir "..\..\obj\win32_sdk_singlethreaded\havana_constraints"
# PROP Target_Dir ""
LINK32=link.exe
MTL=midl.exe
# ADD BASE CPP /nologo /W3 /GX /O2 /I "../../ivp_collision" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_surface_manager" /I "../../ivp_utility" /I "../../../havana/havok" /D "NDEBUG" /D "IVP_VERSION_SDK" /D "_MBCS" /D "_LIB" /D "WIN32" /D "IPION" /FD /c
# SUBTRACT BASE CPP /YX /Yc /Yu
# ADD CPP /nologo /G6 /W3 /GX /O2 /I "../../ivp_collision" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_surface_manager" /I "../../ivp_utility" /I "../../../havana/havok" /D "_WINDOWS" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVANA_CONSTRAINTS" /FD /c
# SUBTRACT CPP /YX /Yc /Yu
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo /out:"../../IVP_Library/hk_physics.lib"
# ADD LIB32 /nologo /out:"..\..\ivp_library\win32_sdk_singlethreaded\havana_constraints.lib"

!ELSEIF  "$(CFG)" == "havana_constraints - Win32 SDK MultiThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "havana_constraints___Win32_SDK_MultiThreaded"
# PROP BASE Intermediate_Dir "havana_constraints___Win32_SDK_MultiThreaded"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\obj\win32_sdk_multithreaded\havana_constraints"
# PROP Intermediate_Dir "..\..\obj\win32_sdk_multithreaded\havana_constraints"
# PROP Target_Dir ""
LINK32=link.exe
MTL=midl.exe
# ADD BASE CPP /nologo /G6 /W3 /GX /O2 /Ob2 /I "../../ivp_collision" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_surface_manager" /I "../../ivp_utility" /I "../../../havana/havok" /D "NDEBUG" /D "IVP_VERSION_SDK" /D "_MBCS" /D "_LIB" /D "WIN32" /D "HAVANA_CONSTRAINTS" /FD /c
# SUBTRACT BASE CPP /YX /Yc /Yu
# ADD CPP /nologo /G6 /MT /W3 /GX /O2 /Ob2 /I "../../ivp_collision" /I "../../ivp_controller" /I "../../ivp_intern" /I "../../ivp_physics" /I "../../ivp_surface_manager" /I "../../ivp_utility" /I "../../../havana/havok" /D "_WINDOWS" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVANA_CONSTRAINTS" /FD /c
# SUBTRACT CPP /YX /Yc /Yu
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo /out:"../../IVP_Library/hk_physics.lib"
# ADD LIB32 /nologo /out:"..\..\ivp_library\win32_sdk_multithreaded\havana_constraints_mt.lib"

!ENDIF 

# Begin Target

# Name "havana_constraints - Win32 Release"
# Name "havana_constraints - Win32 Debug"
# Name "havana_constraints - Win32 SDK SingleThreaded"
# Name "havana_constraints - Win32 SDK MultiThreaded"
# Begin Group "hk_physics (havana)"

# PROP Default_Filter ""
# Begin Group "core"

# PROP Default_Filter ""
# Begin Group "vm_query_builder"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\core\vm_query_builder\mass_relative_vector3.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\core\vm_query_builder\vm_query_builder.h
# End Source File
# End Group
# End Group
# Begin Group "constraint"

# PROP Default_Filter ""
# Begin Group "ball_socket"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\ball_socket\ball_socket_bp.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\ball_socket\ball_socket_constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\ball_socket\ball_socket_constraint.h
# End Source File
# End Group
# Begin Group "limited_ball_socket"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\limited_ball_socket\limited_ball_socket_bp.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\limited_ball_socket\limited_ball_socket_constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\limited_ball_socket\limited_ball_socket_constraint.h
# End Source File
# End Group
# Begin Group "ragdoll"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\ragdoll\ragdoll_constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\ragdoll\ragdoll_constraint.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\ragdoll\ragdoll_constraint_bp.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\ragdoll\ragdoll_constraint_bp_builder.cpp
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\ragdoll\ragdoll_constraint_bp_builder.h
# End Source File
# End Group
# Begin Group "local_constraint_system"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\local_constraint_system\local_constraint_system.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\local_constraint_system\local_constraint_system.inl
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\local_constraint_system\local_constraint_system_bp.h
# End Source File
# End Group
# Begin Group "util"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\util\constraint_limit_util.h
# End Source File
# End Group
# Begin Group "hinge"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\hinge\hinge_bp.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\hinge\hinge_bp_builder.cpp
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\hinge\hinge_bp_builder.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\hinge\hinge_constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\hinge\hinge_constraint.h
# End Source File
# End Group
# Begin Group "breakable_constraint"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\breakable_constraint\breakable_constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\breakable_constraint\breakable_constraint.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\breakable_constraint\breakable_constraint_bp.h
# End Source File
# End Group
# Begin Group "fixed"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\fixed\fixed_bp.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\fixed\fixed_constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\fixed\fixed_constraint.h
# End Source File
# End Group
# Begin Group "prismatic"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\prismatic\prismatic_bp.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\prismatic\prismatic_constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\prismatic\prismatic_constraint.h
# End Source File
# End Group
# Begin Group "pulley"

# PROP Default_Filter "*.h;*.cpp;*.cxx"
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\pulley\pulley_bp.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\pulley\pulley_constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\pulley\pulley_constraint.h
# End Source File
# End Group
# Begin Group "stiff_spring"

# PROP Default_Filter "cpp,h"
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\stiff_spring\stiff_spring_bp.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\stiff_spring\stiff_spring_constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\stiff_spring\stiff_spring_constraint.h
# End Source File
# End Group
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\constraint.cpp
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\constraint.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\constraint_bp.h
# End Source File
# Begin Source File

SOURCE=..\..\havana\havok\hk_physics\constraint\constraint_limit.h
# End Source File
# End Group
# End Group
# Begin Group "hk_physics (ipion)"

# PROP Default_Filter ""
# Begin Group "simunit"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\hk_physics\simunit\psi_info.h
# End Source File
# End Group
# Begin Group "constraint No. 1"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\hk_physics\constraint\local_constraint_system\local_constraint_system.cpp
# End Source File
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\hk_physics\constraint\local_constraint_system\local_constraint_system.h
# End Source File
# End Group
# Begin Group "core No. 1"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\hk_physics\core\rigid_body_core.cpp
# End Source File
# End Group
# Begin Group "effector"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\hk_physics\effector\rigid_body_binary_effector.cpp
# End Source File
# End Group
# Begin Source File

SOURCE=..\..\IVP_PHYSICS\hk_physics\physics.h
# End Source File
# End Group
# End Target
# End Project
