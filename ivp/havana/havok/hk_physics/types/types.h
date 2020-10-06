#ifndef HK_PHYSICS_TYPES_H
#define HK_PHYSICS_TYPES_H


typedef hk_uchar	hk_sim_freq; // 1.0f /  (x milliseconds)

typedef enum {
	HK_TYPE_RIGID_BODY_BP = 0x100
} hk_type;

typedef hk_int32 hk_sorted_set_index;
typedef hk_uint16 hk_sorted_set_store_index;

typedef hk_uint32	hk_timestamp;
#define HK_MAX_TIMESTAMP 0xffffffff

typedef hk_int32 hk_int; //@@@put in by sascha
typedef hk_int16 hk_short; //@@@put in by sascha

#define HK_DELETE(a)  { if (a) { delete(a); (a)=HK_NULL; } } //@@@put in by sascha

// make sure this is big enough to hold a pointer for every platform
typedef hk_int32 hk_Entity_Core_ID;

enum
{
    HK_MAX_FREQUENCY = 0x1,
    HK_MIN_FREQUENCY = 0xff
};

class hk_Clock_Time
{
	public:
		hk_Clock_Time() : m_seconds(0), m_subsecond(0) { }

		void set (hk_real t)
		{
			m_seconds = 0;
			m_subsecond = t;
			overflow_check();
		}

		void advance(hk_real dt)
		{
			m_subsecond += dt;
			overflow_check();
		}

		void overflow_check()
		{
			if(m_subsecond >= 1)
			{
				hk_real overflow = hk_Math::floor(m_subsecond);
				m_seconds += int(overflow);
				m_subsecond -= overflow;
			}
		}
		hk_real as_real()
		{
			return hk_real(m_seconds) + m_subsecond;
		}

		int m_seconds;
		hk_real m_subsecond;
};


#endif //HK_PHYSICS_TYPES_H
