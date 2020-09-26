if( GL AND NOT OSX32 )
    set(SDL "1")
endif()

if( OSXALL )

elseif( (WINDOWS AND NOT QUICKTIME_WINDOWS ) OR X360 OR PS3 )

elseif( WINDOWS AND QUICKTIME_WINDOWS )

endif()

if( GL )
    add_definitions(-DGL_GLEXT_PROTOTYPES -DDX_TO_GL_ABSTRACTION)
endif()
if( SDL )
    add_definitions(-DUSE_SDL)
    #Use system SDL2 for linux.
    if( LINUXALL )
        include_directories("/usr/include/SDL2")
    else()
        include_directories("${SRCDIR}/thirdparty/SDL2")
    endif()
endif()