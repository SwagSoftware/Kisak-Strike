#ifndef HK_MATH_INTERVAL_H
#define HK_MATH_INTERVAL_H

template <class T>
class hk_Interval
{
    public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Interval<T> )

		inline hk_Interval()
		{
		}


		inline hk_Interval(const T& a, const T& b)
			: m_min(a), m_max(b)
		{
		}

		void set(const T& a, const T& b)
		{
			m_min=a;
			m_max=b;
		}

		T m_min;
		T m_max;
};

#endif /* HK_MATH_INTERVAL_H */
