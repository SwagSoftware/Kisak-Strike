#pragma once

#include "platform.h"

#if defined(PLATFORM_E2K)
#include "tslist_alternative.h"
#else
#include "tslist_atomics.h"
#endif

PLATFORM_INTERFACE bool RunTSQueueTests( int nListSize = 10000, int nTests = 1 );
PLATFORM_INTERFACE bool RunTSListTests( int nListSize = 10000, int nTests = 1 );
