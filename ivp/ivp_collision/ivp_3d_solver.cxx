// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PRIVATE

#include <ivp_physics.hxx>

#include <ivp_actuator_info.hxx>

#ifndef WIN32
#	pragma implementation "ivp_3d_solver.hxx"
#endif

#include <ivp_cache_object.hxx>
//#include <ivp_cache_ledge_point.hxx>
#include "ivp_3d_solver.hxx"

int IVP_U_Matrix_Cache::hits = 0;
int IVP_U_Matrix_Cache::misses = 0;

IVP_Time IVP_3D_Solver::calc_nullstelle(IVP_Time t0, IVP_Time t1, IVP_DOUBLE value, IVP_DOUBLE v0, IVP_DOUBLE v1,
				     IVP_Real_Object *solver_a, IVP_Real_Object *solver_b)
{
    // tests for 'value' rather than '0.0f'
    // uses 'sekantenverfahren' und modified mid point

    IVP_ASSERT(v0 >= value);
    IVP_ASSERT(v1 <= value);
    IVP_U_Matrix A,B;

    int counter = 0;
    while(1){
	
	IVP_DOUBLE dv = v1-v0;
	IVP_Time tt;
	// There are two algorithms to reduce the size of the interval:
	// 1. midpoint
	// 2. zero value of connecting straight

	// only alg. 1 ensures termination, alg 2 is better on the average so
	// mix both alg.
	tt = t0;
	tt += (value - v0) * (t1 - t0) / dv;
	
	// modified midpoint
	if ( (counter & 0x3) == 3){	// sometimes split interval int
	    if (counter > 64) return t0;	// endless loop, take left value
	    const IVP_DOUBLE ratio = 0.25f;		// int
	    const IVP_DOUBLE inv_ratio = 1.0f - ratio;
	    IVP_DOUBLE dt0 = t0 - tt;
	    IVP_DOUBLE dt1 = t1 - tt;
	    
	    IVP_DOUBLE mid2 = dt0 + dt1;
	    
	    tt += mid2 * (inv_ratio * 0.5f);
	}
	
	solver_a->calc_at_matrix(tt, &A);
	solver_b->calc_at_matrix(tt, &B);
	IVP_DOUBLE vv = get_value(&A,&B);
	// 	printf("%i %G	%G:%G	%G:%G	%G:%G    %G\n", counter,value,  t0.get_time(),v0, tt.get_time(),vv, t1.get_time(),v1, vv-value);
	
	if(IVP_Inline_Math::fabsd(vv-value) < IVP_3D_SOLVER_NULLSTELLE_EPS){
	    return tt;
	}
	counter++;
	    // take left or right interval?
	if(vv >= value){
	    v0 = vv;
	    t0 = tt;
	}else{
	    v1 = vv;
	    t1 = tt;
	}
    }
    return t0;
}


/** calc zero value when inly deviation is known */
IVP_BOOL IVP_3D_Solver::find_first_t_for_value_max_dev(IVP_DOUBLE value,
					    IVP_Time t_now, IVP_Time t_max, int t_now_cache_index, 
					    IVP_U_Matrix_Cache *mc_A, IVP_U_Matrix_Cache *mc_B,
					    IVP_DOUBLE *opt_val_at_t_now,
					    IVP_Time *t_out
					    )
{    
    /***** first 'null'-stelle from t_now to t_max *******/

    /*** calc max allowed t step (we want the FIRST nullstelle!) ***/
    IVP_DOUBLE v0;
    
    if (opt_val_at_t_now) {
	v0 = *opt_val_at_t_now;
    }else{
	IVP_ASSERT( mc_A->base_time - t_now == 0.0f);
	IVP_ASSERT( mc_B->base_time - t_now == 0.0f);
	IVP_ASSERT( t_now_cache_index == 0);
	
	IVP_U_Matrix *A = mc_A->calc_matrix_at_now( t_now, 0);
	IVP_U_Matrix *B = mc_B->calc_matrix_at_now( t_now, 0);
	v0 = get_value(A,B);
    }
    
    if(v0 <= value){
	// 'collision' already took place
	*t_out = t_now;
	return IVP_TRUE;
    }

        
    // find step interval with different signs
    IVP_Time t0=t_now;
    int cache1_index = t_now_cache_index;
    IVP_ASSERT (t_now - t_max <= 0.0f);
    
    while(1){
	IVP_DOUBLE t_step = (v0 - value) * inv_max_deviation; // max_dev is >> 0.0
	if ( t0 - t_max + t_step  > 0.0f ){
	    // assert value > 0 
	    return IVP_FALSE;
	}

	t_step += t_step;	// because of parabolic behavior @@@@@

	if ( t0 - t_max + t_step  > 0.0f ){
	    // assert value > 0 
	    t_step = t_max - t0 + P_RES_EPS;
	}
	
	// guarantee minimal/ maximal step + rasterize t_steps
	int int_t_step = (int) (t_step * ( IVP_3D_SOLVER_MAX_STEPS_PER_PSI * IVP_3D_SOLVER_PSIS_PER_SECOND));

	if (int_t_step <1){
	    int_t_step = 1;
	}
	cache1_index += int_t_step;
	
	IVP_ASSERT(cache1_index <= IVP_3D_SOLVER_MAX_STEPS_PER_PSI);

	IVP_DOUBLE d_tstep = int_t_step * ( 1.0f/ (IVP_3D_SOLVER_MAX_STEPS_PER_PSI * IVP_3D_SOLVER_PSIS_PER_SECOND));
	IVP_Time t1 = t0; t1 += d_tstep;
	IVP_U_Matrix *A = mc_A->calc_matrix_at( t1, cache1_index);
	IVP_U_Matrix *B = mc_B->calc_matrix_at( t1, cache1_index);
	IVP_DOUBLE v1 = get_value(A,B);
	
	if(v1 <= value){
	    // in this interval somewhere the first nullstelle is lurking
	    IVP_Time t = calc_nullstelle(t0, t1, value, v0, v1, mc_A->object, mc_B->object);
	    if ( t - t_max > 0.0f){
		return IVP_FALSE;
	    }
	    *t_out = t;
	    return IVP_TRUE;
	}
	if (cache1_index == IVP_3D_SOLVER_MAX_STEPS_PER_PSI) return IVP_FALSE;
	t0 = t1;
	v0 = v1;
    }
    // no nullstelle crossed our way
    return IVP_FALSE;
}

/** we know that there will be no zero value for first deviation !!! */
IVP_BOOL IVP_3D_Solver::find_first_t_for_value_max_dev2(IVP_DOUBLE value,
					    IVP_Time t_now, IVP_Time t_max, int t_now_cache_index, 
					    IVP_U_Matrix_Cache *mc_A, IVP_U_Matrix_Cache *mc_B,
					    IVP_DOUBLE *opt_val_at_t_now,
					    IVP_Time *t_out
					    )
{
    return find_first_t_for_value_max_dev(value, t_now, t_max, t_now_cache_index,
					  mc_A, mc_B, opt_val_at_t_now, t_out);
    IVP_ASSERT(0==1);
    return IVP_FALSE;
}

#if 0
/** we know that there will be no zero value for first deviation !!! */
IVP_BOOL IVP_3D_Solver::find_first_t_for_value_no_zero_dev(IVP_DOUBLE value,
							IVP_DOUBLE t_now, IVP_DOUBLE t_max, int t_now_cache_index, 
							IVP_U_Matrix_Cache *mc_A, IVP_U_Matrix_Cache *mc_B,
							IVP_DOUBLE *opt_val_at_t_now,
							IVP_DOUBLE *opt_val_at_next_psi,
							IVP_DOUBLE *t_out
    )
{    
    /***** first 'null'-stelle from t_now to t_max *******/

    /*** calc max allowed t step (we want the FIRST nullstelle!) ***/
    IVP_DOUBLE v0;	// value at left side of interval
    IVP_DOUBLE v1;	// value at right side of interval
    // first of all check right side
    if (opt_val_at_next_psi){
	v1 = *opt_val_at_next_psi;
	IVP_ASSERT(v1 <= value);	// right side should be checked already
    }else{
	IVP_U_Matrix *A = mc_A->m_world_f_core_next_PSI;
	IVP_U_Matrix *B = mc_B->m_world_f_core_next_PSI;	// maximum values 
	v1 = get_value(A,B);
	if (v1 >= value) return IVP_FALSE;		// no zero value
    }
    // ok we know that there will be a zero value

    if (t_max != mc_A->core->time_of_next_psi){	// check more in more detail
	IVP_U_Matrix A,B;
	IVP_Core *solver_a = mc_A->core;
	IVP_Core *solver_b = mc_B->core;
    
	solver_a->calc_at_matrix(t_max, &A);
	solver_b->calc_at_matrix(t_max, &B);
	v1 = get_value(&A,&B);
	if (v1 >= value) return IVP_FALSE;		// no zero value
    }

    /** now get first value */
    if (opt_val_at_t_now) {
	v0 = *opt_val_at_t_now;
    }else{
	IVP_ASSERT( mc_A->base_time == t_now);
	IVP_ASSERT( mc_B->base_time == t_now);
	IVP_ASSERT( t_now_cache_index == 0);
	
	IVP_U_Matrix *A = mc_A->calc_matrix_at_now( t_now, 0);
	IVP_U_Matrix *B = mc_B->calc_matrix_at( t_now, 0);
	v0 = get_value(A,B);
    }
    IVP_ASSERT( v0> value);
    {
	*t_out = calc_nullstelle(t_now, t_max, value, v0, v1, mc_A->core, mc_B->core);
	if ( *t_out < t_max) return IVP_TRUE;
    }
    // no nullstelle crossed our way
    return IVP_FALSE;
}
#endif


/** only max deviation ist known  */
  
IVP_BOOL IVP_3D_Solver::find_first_t_for_value_coll(IVP_DOUBLE value, IVP_DOUBLE value2,
						 IVP_Time t_now, IVP_Time t_max,
						 IVP_U_Matrix_Cache *mc_A, IVP_U_Matrix_Cache *mc_B,
						 IVP_DOUBLE *opt_val_at_t_now, IVP_Time *t_out
    )
{    
    /***** first 'null'-stelle from t_now to t_max *******/

    // spezial fuer kollisionsbestimmung:
    // ziel: nicht sofort collision ausloesen, wenn am start unterhalb coll_dist.
    // gucken, ob werte niedriger als startwert werden:
    // (dann war vector in richtung des objekts) -> erneute kollision.
    // wenn stets value(=coll_dist) > werte > value2(=echte oberflaeche), dann nix.
    // wenn ein wert > value, dann ab dort ganz normal den naechsten zeitpunkt ausrechnen.

    /*** calc max allowed t step (we don't want to hit the ground!) ***/
    IVP_DOUBLE tnow_val;
    {
	IVP_U_Matrix *A,*B;
	IVP_ASSERT( mc_A->base_time - t_now == 0.0f);
	IVP_ASSERT( mc_B->base_time - t_now == 0.0f);

	A = mc_A->calc_matrix_at_now( t_now, 0);
	B = mc_B->calc_matrix_at_now( t_now, 0);
	if (!opt_val_at_t_now){
	    tnow_val = get_value(A,B);
	}else{
	    tnow_val = *opt_val_at_t_now;
	}
    }
    
    if(tnow_val > value){
	// standard situation
	return find_first_t_for_value_max_dev(value, t_now, t_max, 0 /* cache_index*/, mc_A, mc_B, &tnow_val, t_out);
    }

    // we start in collision situation
    IVP_DOUBLE inv_max_dev = 1.0f / max_deviation;
    
    // step through time interval to find a t where val(t) > value
    // throw collision when val(t) becomes <= val(now)
    IVP_DOUBLE v0 =tnow_val;
    IVP_Time t0=t_now;
    int cache1_index = 0;
    
    while(t0 - t_max < 0.0f)    {
//printf("coll T0=%f\n",t0);

	IVP_DOUBLE t_step = (v0 - value2) * inv_max_dev; // max_dev is >> 0.0

	if ( t0 -t_max + t_step > 0.0f){
	    // assert value > 0 
	    t_step = t_max - t0 + P_RES_EPS;
	}else if ( t_step < 0.0f ){
		t_step = 0.0f;
	}
	
	// guarantee minimal/ maximal step + rasterize t_steps
	int int_t_step = (int) (t_step * ( IVP_3D_SOLVER_MAX_STEPS_PER_PSI * IVP_3D_SOLVER_PSIS_PER_SECOND));
	if (int_t_step <1) int_t_step = 1;

	IVP_U_Matrix *A;
	IVP_U_Matrix *B;
	IVP_Time t1;
	IVP_DOUBLE real_t_step = int_t_step * ( 1.0f/ (IVP_3D_SOLVER_MAX_STEPS_PER_PSI * IVP_3D_SOLVER_PSIS_PER_SECOND));
	t1 = t0 + real_t_step;
	cache1_index += int_t_step;	
	A = mc_A->calc_matrix_at( t1, cache1_index);
	B = mc_B->calc_matrix_at( t1, cache1_index);
	
	IVP_DOUBLE v1 = get_value(A,B);

	if(v1 > value){
	    if (t1 - t_max > 0.0f) return IVP_FALSE;
	    // now we reached standard situation
	    IVP_DOUBLE t1_val = v1;	
	    return find_first_t_for_value_max_dev(value, t1, t_max, cache1_index,
					  mc_A,mc_B, &t1_val,t_out);
	}
	if(v1 <= tnow_val){
	    // in t1 we are nearer to object than in t_now -> indicate another collision at t0
	    *t_out = t0;
	    return IVP_TRUE;
	}
	
	t0 = t1;
	v0 = v1;
    }

    // we still are below coll_dist but no real collision happened -> do nothing
    return IVP_FALSE;
}




