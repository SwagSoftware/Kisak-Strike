#ifndef KISAKSTRIKE_VPROF_TRACY_H
#define KISAKSTRIKE_VPROF_TRACY_H

#include "../../thirdparty/tracy-0.7.5/Tracy.hpp"

// AutoScoped profile marker with name
#define TRACY_ZONE( zoneName ) ZoneScopedN( zoneName )


// Sometimes valve uses TM_ZONE(telemetry macro) directly in the code.
//TODO: Tracy doesn't support format-strings - they are only used about 10% of the time though
#define TM_ZONE( level, flags, formatStr, ... ) ZoneScopedN( formatStr )
#define TM_ZONE_DEFAULT( context ) TM_ZONE( context, 0, __FUNCTION__ )
#define TM_ZONE_PLOT( context, name, slot ) TM_ZONE( context, 0, name )
#define TM_ZONE_FILTERED( context, kThreshold, kFlags, kpFormat, ... ) ZoneScopedN( kpFormat )

// stub out some telemetry stuff :((
#ifndef TMZF_NONE
#define TMZF_NONE 0
#endif

#ifndef TELEMETRY_LEVEL1
#define TELEMETRY_LEVEL1 (void)0
#endif

#define tmPlot (void)0

#endif //KISAKSTRIKE_VPROF_TRACY_H
