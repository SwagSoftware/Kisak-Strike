// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

class IVP_Rot_z_Solver;

#define IVP_3D_SOLVER_MAX_STEPS_PER_PSI 20
#define IVP_3D_SOLVER_MIN_T_STEP_DPSI_FACTOR (1.0f/IVP_3D_SOLVER_MAX_STEPS_PER_PSI ) // factor of minimal t st= 1/500 second
#define IVP_3D_SOLVER_PSIS_PER_SECOND  ((int)(1.0f/IVP_MAX_DELTA_PSI_TIME))



#ifndef WIN32
#	pragma interface
#endif

class IVP_U_Matrix_Cache {
    static int hits;
    static int misses;
public:
    IVP_Real_Object *object;
    IVP_Core *core;
    IVP_Time base_time;
    IVP_Time_CODE base_time_code;
    
    IVP_U_Matrix *m_world_f_object[IVP_3D_SOLVER_MAX_STEPS_PER_PSI+1];
    IVP_U_Matrix matrizes[IVP_3D_SOLVER_MAX_STEPS_PER_PSI+1];

    IVP_U_Matrix *calc_matrix_at( IVP_Time t, int index){
	IVP_ASSERT(index<=IVP_3D_SOLVER_MAX_STEPS_PER_PSI);
	IVP_ASSERT(  ((int) ((t-base_time) * (IVP_3D_SOLVER_MAX_STEPS_PER_PSI * IVP_3D_SOLVER_PSIS_PER_SECOND)   + .5f)) == index);
	
	if (!m_world_f_object[index]){
	    m_world_f_object[index] = &matrizes[index];
	    object->calc_at_matrix(t, m_world_f_object[index]);
	    IVP_IF(0){ misses++;hits--;}
	}
	IVP_IF(0){
	    hits++;
	    if (misses %1000 == 0){
		printf("cache hit ratio %f\n", IVP_DOUBLE(hits)/IVP_DOUBLE(misses+hits));
	    }	
	}
	return m_world_f_object[index];
    }

    IVP_U_Matrix *calc_matrix_at_now( IVP_Time t, int index){
	IVP_ASSERT(index == 0);
	IVP_ASSERT(  t - base_time == 0.0f);
	return m_world_f_object[0];
    }


private:
    void p_init(IVP_Cache_Object *co){
	object = co->object;
	core = object->get_core();
	
	if ( !IVP_MTIS_SIMULATED(object->get_movement_state() ) ){
	    for (int i = IVP_3D_SOLVER_MAX_STEPS_PER_PSI; i>=0; i--){
		m_world_f_object[i] = &co->m_world_f_object;	// first value
//		m_world_f_core_next_PSI = &cc->current_m_world_f_core;
	    }
	}else{
	    for (int i = IVP_3D_SOLVER_MAX_STEPS_PER_PSI; i>0; i--){
		m_world_f_object[i] = 0;
	    }
	    m_world_f_object[0] = &co->m_world_f_object;	// first value
	}
	base_time_code = co->valid_until_time_code;
#ifdef DEBUG
	base_time = object->get_environment()->get_current_time();
	if ( IVP_MTIS_SIMULATED(object->get_movement_state() )){
	    IVP_ASSERT( base_time_code == co->valid_until_time_code );
	}
#endif
    }

    
public:

    IVP_U_Matrix_Cache(IVP_Cache_Object *co){
	p_init(co);
    }
    
    // if you want to reuse cache please call calc_calc for using it
    void calc_calc_matrix_cache( IVP_Cache_Object *co){
	if (base_time_code == co->valid_until_time_code) return;
	p_init(co);
    }
};



enum IVP_3D_SOLVER_TYPE {
    IVP_3D_SOLVER_TYPE_MAX_DEV,		// max deviation known
    IVP_3D_SOLVER_TYPE_MAX_DEV2,		// max second deviation + deviation known
    IVP_3D_SOLVER_TYPE_NO_ZERO_DEV	// max dev plus dev != 0 known
};

/* There are some basic classes of solvers !! */

class IVP_3D_Solver {
protected:
    IVP_Time calc_nullstelle(IVP_Time t0, IVP_Time t1, IVP_DOUBLE value,
			   IVP_DOUBLE v0, IVP_DOUBLE v1,
			   IVP_Real_Object *solver_a, IVP_Real_Object *solver_b);
    virtual IVP_DOUBLE get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c) = 0;

public:
    IVP_DOUBLE max_deviation;	// to be set by application
    IVP_DOUBLE inv_max_deviation;


    IVP_3D_SOLVER_TYPE		type;
    IVP_DOUBLE max_deviation2;	// to be set by application
    
    void print(char *name);

    void set_max_deviation( IVP_DOUBLE dev){ max_deviation = dev; inv_max_deviation = 1.0f/max_deviation;

}
    
    
    IVP_BOOL find_first_t_for_value_max_dev( IVP_DOUBLE value,	// returns true if found
					   IVP_Time t_now, IVP_Time t_max,
					   int t_now_cache_index,		// set to 0 if cache base is set to t_now
					   IVP_U_Matrix_Cache *mc_A, IVP_U_Matrix_Cache *mc_B,
					   IVP_DOUBLE *opt_val_at_t_now,
					   IVP_Time *t_out
	);
    IVP_BOOL find_first_t_for_value_max_dev2( IVP_DOUBLE value,	// returns true if found
					   IVP_Time t_now, IVP_Time t_max,
					   int t_now_cache_index,		// set to 0 if cache base is set to t_now
					   IVP_U_Matrix_Cache *mc_A, IVP_U_Matrix_Cache *mc_B,
					   IVP_DOUBLE *opt_val_at_t_now,
					   IVP_Time *t_out
	);
    IVP_BOOL find_first_t_for_value_no_zero_dev( IVP_DOUBLE value,	// returns true if found
					       IVP_Time t_now, IVP_Time t_max,
					       int t_now_cache_index,		// set to 0 if cache base is set to t_now
					       IVP_U_Matrix_Cache *mc_A, IVP_U_Matrix_Cache *mc_B,
					       IVP_DOUBLE *opt_val_at_t_now,
					       IVP_DOUBLE *opt_val_at_next_psi,
					       IVP_Time *t_out
	);
         // erste 'null'-stelle nach t_now bis zu t_max
         // (mit factor_1 * this^2 + factor_2 * term_2 = value)

    IVP_BOOL find_first_t_for_value_coll(	IVP_DOUBLE value,
					IVP_DOUBLE absolute_min_value,	// returns 0 when no value
					IVP_Time t_now, IVP_Time t_max,
					IVP_U_Matrix_Cache *mc_A, IVP_U_Matrix_Cache *mc_B,
					IVP_DOUBLE *opt_val_at_t_now, IVP_Time *t_out
					);


};


