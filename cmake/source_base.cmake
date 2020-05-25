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

set(RAD_TELEMETRY_DISABLED "1")

#add_definitions(-DVPC) #lwss - might not be needed?
if( ${CSTRIKE_TRUNK_BUILD} MATCHES "1" )
    message("On Branch - Trunk.")
elseif( ${CSTRIKE_STAGING_BUILD} MATCHES "1" )
    message("On Branch - Staging.")
elseif( ${CSTRIKE_REL_BUILD} MATCHES "1" )
    message("On Branch - Release.")
endif()

if( ${RAD_TELEMETRY_DISABLED} )
    message( "Telemetry Disabled." )
    add_definitions(-DRAD_TELEMETRY_DISABLED)
else()
    message( "Telemetry Enabled." )
endif()