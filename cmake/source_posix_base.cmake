include("${CMAKE_MODULE_PATH}/common_functions.cmake")

########source_lowest_base########
if(STATIC_LINK)
    add_definitions(-DBASE -DSTATIC_LINK)
endif()
##################################

MacroRequired(SRCDIR)
MacroRequired(_DLL_EXT)

set(LIBPUBLIC "${SRCDIR}/lib/public${PLATSUBDIR}") #this is where static libs are
link_directories(${LIBPUBLIC}) #add to search path for linker
set(LIBCOMMON "${SRCDIR}/lib/common${PLATSUBDIR}")
set(DEVTOOLS "${SRCDIR}/devtools")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "Build type not specified")
endif(NOT CMAKE_BUILD_TYPE)


#$Configuration "Debug"
if (CMAKE_BUILD_TYPE EQUAL "DEBUG")
    message(STATUS "Building in Debug mode")
    add_definitions(-DBASE -DDEBUG -D_DEBUG)
    if( OSXALL )
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -gdwarf-2 -g2 -O2 -march=native")
    elseif( LINUXALL )
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -gdwarf-4 -g2 -O2 -march=native")
    endif()
#$Configuration "Release"
else()
    message(STATUS "Building in Release mode")
    add_definitions(-DBASE -DNDEBUG)
    if( OSXALL )
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -gdwarf-2 -g2 -O2 -march=native")
    elseif( LINUXALL )
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -gdwarf-4 -g2 -O2 -march=native")
    endif()
endif()

#$Compiler
include_directories("${SRCDIR}/common")
include_directories("${SRCDIR}/public")
include_directories("${SRCDIR}/public/tier0")
include_directories("${SRCDIR}/public/tier1")
add_definitions(-DGNUC -DPOSIX -DCOMPILER_GCC -DMEMOVERRIDE_MODULE=${PROJECT_NAME} -D_DLL_EXT=${_DLL_EXT})
if(DEDICATED)
    add_definitions(-DDEDICATED)
endif()
if(OSXALL)
    add_definitions(-D_OSX -DOSX -D_DARWIN_UNLIMITED_SELECT -DFD_SETSIZE=10240)
endif()

if(LINUXALL)
    #add_definitions(-D_LINUX -DLINUX)
    if( DONT_DOWNGRADE_ABI )
        message(STATUS "KEEPING CXX11 ABI FOR PROJECT")
    else()
        message(STATUS "DOWNGRADING CXX11 ABI")
        #disable cpp11 ABI so libraries <gcc 5 will work
        add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
    endif()
endif()
if(POSIX)
    set(CMAKE_CXX_VISIBILITY_PRESET hidden) #$SymbolVisibility	"hidden"
    add_definitions(-DPOSIX -D_POSIX)
endif()
if(OSX64)
    add_definitions(-DPLATFORM_64BITS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -arch x86_64")
endif()

if(NOT IS_LIB_PROJECT)
    #set(ConfigurationType "Application (.exe)") #not used

    #$Linker
    if(OSX64)
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -arch x86_64")
    endif()

    #$Folder	"Link Libraries"
    if( NOSTINKYLINKIES )
        message(STATUS "skipping stinky linkie")
    else()
        link_libraries("libtier0_client")
        link_libraries("tier1_client")
        link_libraries("interfaces_client")
        #include_directories("vstdlib")
    endif()
endif()