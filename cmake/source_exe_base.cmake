include("${CMAKE_MODULE_PATH}/platform_dirs.cmake") #defines PLATSUBDIR
include("${CMAKE_MODULE_PATH}/source_base.cmake")

if(POSIX)
    include("${CMAKE_MODULE_PATH}/source_exe_posix_base.cmake")
elseif(WINDOWS)
    message(FATAL_ERROR "Cmake Windows Unsupported!")
elseif(X360)
    message(FATAL_ERROR "Cmake X360 Unsupported!")
elseif(PS3)
    message(FATAL_ERROR "Cmake PS3 Unsupported!")
endif()

include("${CMAKE_MODULE_PATH}/source_video_base.cmake")