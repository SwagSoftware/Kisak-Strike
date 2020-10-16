#ifndef HK_BASE_STOPWATCH_H
#define HK_BASE_STOPWATCH_H

#include <hk_base/base_types.h>

#ifdef HK_HAVE_QUERY_PERFORMANCE_TIMER
#	include <hk_base/stopwatch/stopwatch_qpt.inl>
	typedef hk_Stopwatch_qpt hk_Stopwatch;
#else /* do nothing version */

class hk_Stopwatch
{
	public:

		hk_Stopwatch()	{ }
		~hk_Stopwatch()	{ }

		void start(){ }
		void stop() { }
		void reset(){ }

		hk_real get_elapsed_time()	{ return 0; }
		hk_real get_split_time()	{ return 0; }
};
#endif /* special stopwatch implementation */

#endif /* HK_BASE_STOPWATCH_H */

