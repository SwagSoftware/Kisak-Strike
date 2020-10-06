
#if defined(HK_HAVE_MSVC_INLINE_ASSEMBLY)

inline void hk_query_performance_timer(hk_uint64* ticks)
{
	__asm {
		mov edi, ticks
		rdtsc
		mov [edi  ], eax
		mov [edi+4], edx
	}
}

#elif defined(HK_HAVE_GNU_INLINE_ASSEMBLY)

inline void hk_query_performance_timer(hk_uint64* ticks)
{
	__asm__ __volatile__ (	"rdtsc\n\t"
							"movl %%eax,  (%0)\n\t"
							"movl %%edx, 4(%0)\n\t"
								: /* no output regs */
								: "D" (ticks)
								: "%eax", "%edx");
}

#else
#	error HK_HAVE_QUERY_PERFORMANCE_TIMER is defined, but no implementation.
#endif

inline void hk_query_performance_timer_frequency(hk_uint64* freq)
{
	// assume 800 Mhz for now
	*freq = 800000000;
}

////////////////////////

class hk_Stopwatch_qpt
{
	public:

		hk_Stopwatch_qpt();

		void start();
		void stop();
		void reset();

		hk_real get_elapsed_time();
		hk_real get_split_time();

	private:

		hk_uint64	m_ticks_at_start;
		hk_uint64	m_ticks_at_split;
		hk_uint64	m_ticks_total;
		bool		m_running_flag;
		int			m_num_timings;

		static hk_uint64	s_ticks_per_second;
};

inline hk_Stopwatch_qpt::hk_Stopwatch_qpt()
{
	if(s_ticks_per_second==0)
		hk_query_performance_timer_frequency(&s_ticks_per_second);
	reset();
}

inline void hk_Stopwatch_qpt::start()
{
	HK_ASSERT(! m_running_flag);
	m_running_flag = true;
	hk_query_performance_timer(&m_ticks_at_start);
	m_ticks_at_split = m_ticks_at_start;
}

inline void hk_Stopwatch_qpt::stop()
{
	HK_ASSERT(m_running_flag);

	m_running_flag = false;
	hk_uint64	ticks_now;
	hk_query_performance_timer(&ticks_now);
	m_ticks_total += ticks_now - m_ticks_at_start;
	++m_num_timings;
}

inline void hk_Stopwatch_qpt::reset()
{
	m_ticks_at_start = 0;
	m_ticks_at_split = 0;
	m_ticks_total = 0;
	m_running_flag = 0;
	m_num_timings = 0;
}

inline hk_real hk_Stopwatch_qpt::get_elapsed_time()
{
	return hk_real(hk_uint32(m_ticks_total)) / hk_real(hk_uint32(s_ticks_per_second));
}

inline hk_real hk_Stopwatch_qpt::get_split_time()
{
	hk_uint64 ticks_now;
	hk_query_performance_timer(&ticks_now);
	hk_uint32 sticks = hk_uint32(ticks_now - m_ticks_at_split);
	m_ticks_at_split = ticks_now;
	return hk_real(sticks) / hk_real(hk_uint32(s_ticks_per_second));
}

