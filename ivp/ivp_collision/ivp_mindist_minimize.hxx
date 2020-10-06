// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef IVP_HASH_INCLUDED
#	include <ivu_hash.hxx>
#endif

class IVP_Cache_Ball;
class IVP_Cache_Ledge_Point;

  // set IVP_LOOP_LIST_SIZE if you want to use a linear list for loop check instead of a HASH
#define IVP_LOOP_LIST_SIZE 256

#if defined(IVP_LOOP_LIST_SIZE)
    struct IVP_MM_Loop_Hash_Struct {
	int a;
	int b;
    };
#endif

class IVP_Mindist_Minimize_Solver {

protected:
    ////////// initialisation
    static IVP_MRC_TYPE minimize_illegal(IVP_Mindist_Minimize_Solver *);
    static IVP_MRC_TYPE minimize_default_poly_poly(IVP_Mindist_Minimize_Solver *); // not all cases
    static IVP_MRC_TYPE minimize_swapped_poly_poly(IVP_Mindist_Minimize_Solver *); // swap and call default
    static IVP_MRC_TYPE minimize_PB(IVP_Mindist_Minimize_Solver *);
    static IVP_MRC_TYPE minimize_KB(IVP_Mindist_Minimize_Solver *);
    static IVP_MRC_TYPE minimize_FB(IVP_Mindist_Minimize_Solver *);
    static IVP_MRC_TYPE minimize_B_POLY(IVP_Mindist_Minimize_Solver *);
    //static IVP_MRC_TYPE minimize_BK(IVP_Mindist_Minimize_Solver *);
    //static IVP_MRC_TYPE minimize_BF(IVP_Mindist_Minimize_Solver *);
    static IVP_MRC_TYPE minimize_BB(IVP_Mindist_Minimize_Solver *);

    void swap_synapses(){ mindist->synapse_sort_flag ^= 1; }

    void sort_synapses( IVP_Synapse *a, IVP_Synapse *){
	if ( a != mindist->get_sorted_synapse(0) ) swap_synapses();
    }

    static IVP_MRC_TYPE (*mms_function_table[IVP_ST_MAX_LEGAL][IVP_ST_MAX_LEGAL])(IVP_Mindist_Minimize_Solver *mms);

    /////////// real functions
    
    void init_loop_hash();
  IVP_BOOL check_loop_hash(IVP_SYNAPSE_POLYGON_STATUS s0, const IVP_Compact_Edge *e0,
			   IVP_SYNAPSE_POLYGON_STATUS s1, const IVP_Compact_Edge *e1);
    
  IVP_MRC_TYPE p_minimize_PF(const IVP_Compact_Edge *P, const IVP_Compact_Edge *F, IVP_Cache_Ledge_Point *m_cache_P, IVP_Cache_Ledge_Point *m_cache_F);
    
  IVP_MRC_TYPE p_minimize_Leave_PF(const IVP_Compact_Edge *P,const IVP_U_Point *P_Fos, const  IVP_Compact_Edge *F,
				    IVP_Cache_Ledge_Point *m_cache_P, IVP_Cache_Ledge_Point *m_cache_F);

  IVP_MRC_TYPE p_minimize_PK(const IVP_Compact_Edge *P, const IVP_Compact_Edge *K, IVP_Cache_Ledge_Point *m_cache_P, IVP_Cache_Ledge_Point *m_cache_K);
  IVP_MRC_TYPE p_minimize_Leave_PK(const IVP_Compact_Edge *P,const  IVP_Compact_Edge *K, IVP_Cache_Ledge_Point *m_cache_P, IVP_Cache_Ledge_Point *m_cache_K);    

  IVP_MRC_TYPE p_minimize_PP(const IVP_Compact_Edge *A, const IVP_Compact_Edge *B, IVP_Cache_Ledge_Point *m_cache_A, IVP_Cache_Ledge_Point *m_cache_B);

  IVP_MRC_TYPE p_minimize_KK(const IVP_Compact_Edge *K,const  IVP_Compact_Edge *L,
			      IVP_Cache_Ledge_Point *m_cache_K, IVP_Cache_Ledge_Point *m_cache_L);
  IVP_MRC_TYPE p_minimize_Leave_KK(const IVP_Compact_Edge *K,const  IVP_Compact_Edge *L,
				   const class IVP_KK_Input &kkin, const class IVP_Unscaled_KK_Result *kkr,
				    IVP_Cache_Ledge_Point *m_cache_K, IVP_Cache_Ledge_Point *m_cache_L);

  IVP_MRC_TYPE p_minimize_FF(const IVP_Compact_Edge *F, const IVP_Compact_Edge *G, IVP_Cache_Ledge_Point *m_cache_F, IVP_Cache_Ledge_Point *m_cache_G);


    IVP_MRC_TYPE p_minimize_BP(IVP_Cache_Ball *m_cache_ball, const IVP_Compact_Edge *P, IVP_Cache_Ledge_Point *m_cache_P);
    IVP_MRC_TYPE p_minimize_BK(IVP_Cache_Ball *m_cache_ball, const IVP_Compact_Edge *K, IVP_Cache_Ledge_Point *m_cache_K);
    IVP_MRC_TYPE p_minimize_Leave_BK(IVP_Cache_Ball *m_cache_ball, const IVP_Compact_Edge *K, IVP_Cache_Ledge_Point *m_cache_K);
    IVP_MRC_TYPE p_minimize_BF(IVP_Cache_Ball *m_cache_ball, const IVP_Compact_Edge *F, IVP_Cache_Ledge_Point *m_cache_F);

    
#ifdef DEBUG_CHECK_LEN
    IVP_RETURN_TYPE validate_termination_len(IVP_DOUBLE now_val); // used by check_len_xx()
    IVP_RETURN_TYPE check_len_PP(IVP_Compact_Edge *A, IVP_Compact_Edge *B, IVP_Cache_Ledge_Point *m_cache_A, IVP_Cache_Ledge_Point *m_cache_B);
    IVP_RETURN_TYPE check_len_PK(IVP_Compact_Edge *P, IVP_Compact_Edge *K, IVP_Cache_Ledge_Point *m_cache_P, IVP_Cache_Ledge_Point *m_cache_K);
    IVP_RETURN_TYPE check_len_PF(IVP_Compact_Edge *P, IVP_Compact_Edge *F, IVP_Cache_Ledge_Point *m_cache_P, IVP_Cache_Ledge_Point *m_cache_F);
    IVP_RETURN_TYPE check_len_KK(IVP_Compact_Edge *K, IVP_Compact_Edge *L, IVP_Cache_Ledge_Point *m_cache_K, IVP_Cache_Ledge_Point *m_cache_L);
    IVP_RETURN_TYPE check_len_FF(IVP_Compact_Edge *F, IVP_Compact_Edge *G, IVP_Cache_Ledge_Point *m_cache_F, IVP_Cache_Ledge_Point *m_cache_G);
#endif
    
    // for debug function proove()
    IVP_DOUBLE p_optimize_FF( const IVP_Compact_Edge *A, const IVP_Compact_Edge *B,
			  IVP_Cache_Ledge_Point *m_cache_A,  IVP_Cache_Ledge_Point *m_cache_B,
			  IVP_DOUBLE min_qdist);
#ifdef DEBUG
    IVP_RETURN_TYPE proove_polypoly();
    IVP_RETURN_TYPE proove_ballpoly();
#endif
  
public:
    IVP_Mindist *mindist;
    int P_Finish_Counter; 	// global for debug/termination purposes

#ifdef DEBUG
    IVP_DOUBLE termination_len; 	// used for termination function test
#endif

    void pierce_mindist();

  IVP_U_Point pos_opposite_BacksideOs;   // position of synapse opposite backside cases in backside space



#if !defined(IVP_LOOP_LIST_SIZE)
    IVP_Hash *loop_hash; 	// record situations for loop check
#else
    IVP_MM_Loop_Hash_Struct loop_hash[IVP_LOOP_LIST_SIZE];
    int loop_hash_size;
#endif

    IVP_Mindist_Minimize_Solver(IVP_Mindist *md) {
	mindist = md;
#if !defined(IVP_LOOP_LIST_SIZE)
	loop_hash = NULL;
#else
	loop_hash_size = 0;
#endif
	P_Finish_Counter = IVP_MAX_MINIMIZE_BEFORE_HASH_CHECK;
#ifdef DEBUG
	termination_len = P_DOUBLE_MAX;
#endif	
    };
    static void init_mms_function_table();
    
    ~IVP_Mindist_Minimize_Solver() {
#if !defined(IVP_LOOP_LIST_SIZE)
	P_DELETE(loop_hash);
#endif
    };
    
    IVP_MRC_TYPE recalc_mindist_sub(){
	IVP_IF_PREFETCH_ENABLED(IVP_TRUE){
	    IVP_PREFETCH( mindist->get_synapse(0)->edge,0);
	    IVP_PREFETCH( mindist->get_synapse(1)->edge,0);
	}
	IVP_SYNAPSE_POLYGON_STATUS s0 = mindist->get_sorted_synapse(0)->get_status();
	IVP_SYNAPSE_POLYGON_STATUS s1 = mindist->get_sorted_synapse(1)->get_status();
	IVP_ASSERT(s0 < IVP_ST_MAX_LEGAL );
	IVP_ASSERT(s1 < IVP_ST_MAX_LEGAL );

	IVP_MRC_TYPE ret = mms_function_table[s0][s1]( this );
	return ret;
    }
};
