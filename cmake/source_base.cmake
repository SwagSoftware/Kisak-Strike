#//-----------------------------------------------------------------------------
#//	source_base.cmake
#//
#//  This is the base VPC file that is included by all others, on all platforms.
#//
#//	Project Script
#//-----------------------------------------------------------------------------

# Set which branch we are building out of.
# This is one file we expect to be different between branches and so it must be merged carefully

#Rel Branch
set(CSTRIKE_TRUNK_BUILD "0")
set(CSTRIKE_STAGING_BUILD "0")
set(CSTRIKE_REL_BUILD "1")

add_definitions(-DCSTRIKE_REL_BUILD) #without this, some weird stuff is included like fatdemos
add_definitions(-DALLOW_DEVELOPMENT_CVARS) #let's allow development convars since this is an open source project.

#Rad telemetry profiler is not enabled with kisak-strike
set(RAD_TELEMETRY_DISABLED "1")
add_definitions(-DRAD_TELEMETRY_DISABLED)
#Tracy Profiler support
if( USE_TRACY )
    add_definitions(-DUSE_TRACY -DTRACY_ENABLE -DVPROF_LEVEL=2)
    if( TRACY_STORE_LOGS )
        # Tracy will store logs throughout the lifetime of the program. When the Tracy UI connects, it will send these logs over.
        # This will eat memory pretty quick and also will only allow 1 tracy session per runtime.
        # Use this if you want to profile the startup of the program.
    else()
        # Set Tracy to use On-Demand mode. This means you can disconnect/connect the UI whenever you want
        # and also can be used for long periods of time without running out of memory
        # **This mode is the Default**
        add_definitions(-DTRACY_ON_DEMAND)
    endif()
endif()

#CMAKE FILLS THESE IN BY DEFAULT. NUKE THEM!
set(CMAKE_CXX_FLAGS_RELEASE "")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "")
set(CMAKE_CXX_FLAGS_MINSIZEREL "")
set(CMAKE_CXX_FLAGS_DEBUG "")

#add_definitions(-DVPC) #lwss - might not be needed?
#if( ${CSTRIKE_TRUNK_BUILD} MATCHES "1" )
#    message("On Branch - Trunk.")
#elseif( ${CSTRIKE_STAGING_BUILD} MATCHES "1" )
#    message("On Branch - Staging.")
#elseif( ${CSTRIKE_REL_BUILD} MATCHES "1" )
#    message("On Branch - Release.")
#endif()