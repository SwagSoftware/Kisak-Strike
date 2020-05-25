include("${CMAKE_MODULE_PATH}/common_functions.cmake")
include("${CMAKE_MODULE_PATH}/platform_dirs.cmake") #defines PLATSUBDIR
include("${CMAKE_MODULE_PATH}/source_base.cmake")


add_definitions(-DCSTRIKE15)

MacroRequired(SRCDIR)

MacroRequired(OUTLIBDIR "${SRCDIR}/lib/public${PLATSUBDIR}")
MacroRequired(OUTLIBCOMMONDIR "${SRCDIR}/lib/common${PLATSUBDIR}")


if(POSIX)
    set( _STATICLIB_EXT ".a")
    include("${CMAKE_MODULE_PATH}/source_lib_posix_base.cmake")
elseif(WINDOWS)
    message(FATAL_ERROR "TODO Windows cmake support")
elseif(X360)
    message(FATAL_ERROR "TODO X360 cmake support")# lol like this will ever happen
elseif(PS3)
    message(FATAL_ERROR "TODO PS3 cmake support")
else()
    message(FATAL_ERROR "Couldn't find platform for library base")
endif()


include("${CMAKE_MODULE_PATH}/source_video_base.cmake")