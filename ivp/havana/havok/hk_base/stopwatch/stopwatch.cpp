#include <hk_base/base.h>
#include <hk_base/stopwatch/stopwatch.h>

#ifdef HK_HAVE_QUERY_PERFORMANCE_TIMER
hk_uint64 hk_Stopwatch_qpt::s_ticks_per_second = 8000000;
#endif
