# Microsoft Developer Studio Project File - Name="ivp_compactbuilder.lib" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=ivp_compactbuilder.lib - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "ivp_compactbuilder_lib.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "ivp_compactbuilder_lib.mak" CFG="ivp_compactbuilder.lib - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "ivp_compactbuilder.lib - Win32 SDK SingleThreaded" (based on "Win32 (x86) Static Library")
!MESSAGE "ivp_compactbuilder.lib - Win32 SDK MultiThreaded" (based on "Win32 (x86) Static Library")
!MESSAGE "ivp_compactbuilder.lib - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE "ivp_compactbuilder.lib - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName "ivp_compactbuilder_lib"
# PROP Scc_LocalPath "..\.."
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "ivp_compactbuilder.lib - Win32 SDK SingleThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\obj\win32_sdk_singlethreaded\ivp_compactbuilder"
# PROP Intermediate_Dir "..\..\obj\win32_sdk_singlethreaded\ivp_compactbuilder"
# PROP Target_Dir ""
MTL=midl.exe
LINK32=link.exe
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /I "../../ivp_surface_manager" /I "../../ivp_physics" /I "../../ivp_compact_builder" /I "../../ivp_utility" /I "../../ivp_collision" /I "../../../havok/source" /D "NDEBUG" /D "_MBCS" /D "_LIB" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVOK_MOPP" /Yu"ivp_physics.hxx" /FD /c
# SUBTRACT CPP /Fr
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\ivp_library\win32_sdk_singlethreaded\ivp_compactbuilder.lib"

!ELSEIF  "$(CFG)" == "ivp_compactbuilder.lib - Win32 SDK MultiThreaded"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\obj\win32_sdk_multithreaded\ivp_compactbuilder"
# PROP Intermediate_Dir "..\..\obj\win32_sdk_multithreaded\ivp_compactbuilder"
# PROP Target_Dir ""
MTL=midl.exe
LINK32=link.exe
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_MBCS" /D "_LIB" /YX /FD /c
# ADD CPP /nologo /MT /W3 /GX /O2 /I "../../ivp_surface_manager" /I "../../ivp_physics" /I "../../ivp_compact_builder" /I "../../ivp_utility" /I "../../ivp_collision" /D "NDEBUG" /D "_MBCS" /D "_LIB" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVOK_MOPP" /Yu"ivp_physics.hxx" /FD /c
# SUBTRACT CPP /Fr
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\ivp_library\win32_sdk_multithreaded\ivp_compactbuilder_mt.lib"

!ELSEIF  "$(CFG)" == "ivp_compactbuilder.lib - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\obj\win32_debug\ivp_compactbuilder"
# PROP Intermediate_Dir "..\..\obj\win32_debug\ivp_compactbuilder"
# PROP Target_Dir ""
MTL=midl.exe
LINK32=link.exe
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_MBCS" /D "_LIB" /YX /FD /GZ /c
# ADD CPP /nologo /W3 /Gm /GX /ZI /Od /I "../../ivp_surface_manager" /I "../../ivp_physics" /I "../../ivp_compact_builder" /I "../../ivp_utility" /I "../../ivp_collision" /D "DEBUG" /D "_MBCS" /D "_LIB" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVOK_MOPP" /FR /FD /GZ /c
# SUBTRACT CPP /YX /Yc /Yu
# ADD BASE RSC /l 0x407 /d "_DEBUG"
# ADD RSC /l 0x407 /d "DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo /out:"..\..\ivp_library\win32_debug\ivp_compactbuilder_d.lib"
# Begin Custom Build
TargetPath=\DEV\src\ivp\ivp_library\win32_debug\ivp_compactbuilder_d.lib
InputPath=\DEV\src\ivp\ivp_library\win32_debug\ivp_compactbuilder_d.lib
SOURCE="$(InputPath)"

"..\..\..\lib\common\ivp_compactbuilder.lib" : $(SOURCE) "$(INTDIR)" "$(OUTDIR)"
	if exist ..\..\..\lib\common\ivp_compactbuilder.lib attrib -r ..\..\..\lib\common\ivp_compactbuilder.lib 
	copy $(TargetPath) ..\..\..\lib\common\ivp_compactbuilder.lib 
	
# End Custom Build

!ELSEIF  "$(CFG)" == "ivp_compactbuilder.lib - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "ivp_compactbuilder_lib___Win32_Release"
# PROP BASE Intermediate_Dir "ivp_compactbuilder_lib___Win32_Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\obj\win32_release\ivp_compactbuilder"
# PROP Intermediate_Dir "..\..\obj\win32_release\ivp_compactbuilder"
# PROP Target_Dir ""
MTL=midl.exe
LINK32=link.exe
# ADD BASE CPP /nologo /W3 /GX /O2 /I "../../ivp_surface_manager" /I "../../ivp_physics" /I "../../ivp_compact_builder" /I "../../ivp_utility" /I "../../ivp_collision" /D "_MBCS" /D "_LIB" /D "NDEBUG" /D "WIN32" /D "IVP_VERSION_SDK" /YX /FD /c
# SUBTRACT BASE CPP /Fr
# ADD CPP /nologo /W3 /GX /O2 /I "../../ivp_surface_manager" /I "../../ivp_physics" /I "../../ivp_compact_builder" /I "../../ivp_utility" /I "../../ivp_collision" /D "NDEBUG" /D "_MBCS" /D "_LIB" /D "WIN32" /D "IVP_VERSION_SDK" /D "HAVOK_MOPP" /Yu"ivp_physics.hxx" /FD /c
# SUBTRACT CPP /Fr
# ADD BASE RSC /l 0x407 /d "NDEBUG"
# ADD RSC /l 0x407 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo /out:"../../IVP_Library/ivp_compactbuilder.lib"
# ADD LIB32 /nologo /out:"..\..\ivp_library\win32_release\ivp_compactbuilder.lib"
# Begin Custom Build
TargetPath=\DEV\src\ivp\ivp_library\win32_release\ivp_compactbuilder.lib
InputPath=\DEV\src\ivp\ivp_library\win32_release\ivp_compactbuilder.lib
SOURCE="$(InputPath)"

"..\..\..\lib\common\ivp_compactbuilder.lib" : $(SOURCE) "$(INTDIR)" "$(OUTDIR)"
	if exist ..\..\..\lib\common\ivp_compactbuilder.lib attrib -r ..\..\..\lib\common\ivp_compactbuilder.lib 
	copy $(TargetPath) ..\..\..\lib\common\ivp_compactbuilder.lib 
	
# End Custom Build

!ENDIF 

# Begin Target

# Name "ivp_compactbuilder.lib - Win32 SDK SingleThreaded"
# Name "ivp_compactbuilder.lib - Win32 SDK MultiThreaded"
# Name "ivp_compactbuilder.lib - Win32 Debug"
# Name "ivp_compactbuilder.lib - Win32 Release"
# Begin Group "IVP_COMPACT_BUILDER"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_compact_ledge_gen.cxx
# ADD CPP /Yc"ivp_physics.hxx"
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_compact_ledge_gen.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_compact_modify.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_compact_modify.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_compact_recursive.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_compact_recursive.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_convex_decompositor.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_convex_decompositor.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_halfspacesoup.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_halfspacesoup.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_i_fpoint_vhash.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_i_fpoint_vhash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_i_point_vhash.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_i_point_vhash.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_object_polygon_tetra.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_object_polygon_tetra.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_rot_inertia_solver.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_rot_inertia_solver.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_surbuild_3ds.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_surbuild_halfspacesoup.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_surbuild_halfspacesoup.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_surbuild_ledge_soup.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_surbuild_ledge_soup.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_surbuild_pointsoup.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_surbuild_pointsoup.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_surbuild_polygon_convex.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_surbuild_polygon_convex.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_surbuild_polyhdrn_cncv.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_surbuild_polyhdrn_cncv.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_surbuild_q12.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_surbuild_q12.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_templates_intern.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_templates_intern.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivp_tetra_intrude.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_tetra_intrude.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivp_triangle_gen.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\ivv_cluster_min_hash.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\ivv_cluster_min_hash.hxx
# End Source File
# End Group
# Begin Group "IVP_QHULL"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\qhull.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\qhull.hxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\qhull_a.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\qhull_geom.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\qhull_geom.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\qhull_geom2.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\qhull_global.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\qhull_io.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\qhull_io.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\qhull_mem.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\qhull_mem.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\qhull_merge.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\qhull_merge.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\qhull_poly.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\qhull_poly.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\qhull_poly2.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\qhull_qset.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\qhull_qset.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\qhull_stat.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\qhull_stat.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\qhull_user.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\qhull_user.hxx
# End Source File
# End Group
# Begin Group "IVP_GEOMPACK"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\geompack.hxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\geompack_cutfac.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\geompack_cvdec3.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\geompack_drdec3.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\geompack_dsphdc.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\geompack_edght.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\geompack_initcb.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\geompack_insed3.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\geompack_insfac.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\geompack_insvr3.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\geompack_prime.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\geompack_ptpolg.cxx
# End Source File
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\geompack_resedg.cxx
# End Source File
# End Group
# Begin Source File

SOURCE=..\..\Ivp_compact_builder\3dsimport_co.cxx
# End Source File
# Begin Source File

SOURCE=..\..\IVP_COMPACT_BUILDER\3dsimport_load.hxx
# End Source File
# End Target
# End Project
