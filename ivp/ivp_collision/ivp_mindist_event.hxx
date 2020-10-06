// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

class IVP_Cache_Ledge_Point;

class IVP_Mindist_Event_Solver {
public:	// to be set prior calc_time_of_next_event !!!
//    IVP_U_Point direction;	// object0 to object1
    IVP_DOUBLE sum_max_surface_rot_speed;
    IVP_DOUBLE projected_center_speed;
    IVP_DOUBLE max_coll_speed;
    IVP_DOUBLE worst_case_speed;
    IVP_Mindist *mindist;
    IVP_Environment *environment;
    IVP_Time t_now;
    IVP_Time t_max;

    // the result:
    IVP_COLL_TYPE event_type_out;
    IVP_Time event_time_out;
protected:
    static void next_event_illegal(IVP_Mindist_Event_Solver *);
    static void next_event_default_poly_poly(IVP_Mindist_Event_Solver *); // not all cases
    static void next_event_B_POLY(IVP_Mindist_Event_Solver *);

    static void next_event_BB(IVP_Mindist_Event_Solver *);
    
    static void  (*mim_function_table[IVP_ST_MAX_LEGAL][IVP_ST_MAX_LEGAL])(IVP_Mindist_Event_Solver *mms);
  
protected:
    /** calc the delta time of the next event, returns max_delta_time when no events found */
    void calc_next_event_PF(const IVP_Compact_Edge *P,
			      const IVP_Compact_Edge *F,
			      IVP_Cache_Ledge_Point *m_cache_P,
			      IVP_Cache_Ledge_Point *m_cache_F);
    
    void calc_next_event_PP(const IVP_Compact_Edge *P,
			      const IVP_Compact_Edge *P2,
			      IVP_Cache_Ledge_Point *m_cache_P,
			      IVP_Cache_Ledge_Point *m_cache_P2);
    
    void calc_next_event_PK(const IVP_Compact_Edge *P,
			      const IVP_Compact_Edge *K,
			      IVP_Cache_Ledge_Point *m_cache_P,
			      IVP_Cache_Ledge_Point *m_cache_K);

    void calc_next_event_KK(const IVP_Compact_Edge *P,
			      const IVP_Compact_Edge *K2,
			      IVP_Cache_Ledge_Point *m_cache_K,
			      IVP_Cache_Ledge_Point *m_cache_K2);

    void calc_next_event_BF(const IVP_Compact_Edge *F,
			      IVP_Cache_Object *m_cache_B, IVP_Cache_Ledge_Point *m_cache_F);
    void calc_next_event_BK(IVP_Ball *B, const IVP_Compact_Edge *K,
			      IVP_Cache_Object *m_cache_B, IVP_Cache_Ledge_Point *m_cache_K);
    void calc_next_event_BP(IVP_Ball *B, const IVP_Compact_Edge *P,
			      IVP_Cache_Object *m_cache_B, IVP_Cache_Ledge_Point *m_cache_P);
    void calc_next_event_BB(IVP_Cache_Object *m_cache_A, IVP_Cache_Object *m_cache_B);


public:
    static void init_mim_function_table();
    
// get the next interesting event e.g. collision ....
    void calc_time_of_next_event(){
	IVP_SYNAPSE_POLYGON_STATUS s0 = mindist->get_sorted_synapse(0)->get_status();
	IVP_SYNAPSE_POLYGON_STATUS s1 = mindist->get_sorted_synapse(1)->get_status();
	IVP_ASSERT(s0 < IVP_ST_MAX_LEGAL );
	IVP_ASSERT(s1 < IVP_ST_MAX_LEGAL );
	mim_function_table[s0][s1]( this );
    }
};


