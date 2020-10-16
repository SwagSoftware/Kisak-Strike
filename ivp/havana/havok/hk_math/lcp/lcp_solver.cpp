#include <hk_math/vecmath.h>

#include <hk_math/densematrix.h>
#include <hk_math/gauss_elimination/gauss_elimination.h>
#include <hk_math/incr_lu/incr_lu.h>
#include <hk_math/vector_fpu/vector_fpu.h>
#include <hk_math/lcp/lcp_solver.h>

//#define HK_DEBUG
#ifdef HK_DEBUG
#	define HK_LCP_IF(flag)  if (flag)
#else
#	define HK_LCP_IF(flag)	if (0==1)
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

#define HK_LCP_SOLVER_EPS (1E-7f)
#define HK_LCP_TEST_EPS (HK_LCP_SOLVER_EPS*1000.0f)
#define HK_LCP_NUMERIC_TEST_EVERY_N 7
#define HK_LCP_MAX_STEP_LEN (10000.0f)
#define HK_LCP_REAL_MAX 1e20f
#define MAX_ERROR_BUFFER_LEN 1024


/* Algorithm explanation:
   The algorithm implemented here is basically the Algorithm described in
   David Baraff: "Fast Contact Force Computation for Nonpenetrating Rigid Bodies", SIGGRAPH '94
   The difference is we use here a incremental LU solution for the active variables
   In case the LU is invalid - e.g. numerical problems, Sub-Matrix is singular, ... - we use a
   gaussian elemination to get delta_x values of active variables per step (very costly :( )
   
   Optimization used here is to say what variables should be active at start. We than 
   try to setup the LU solver and start with an set of actives right from the start.

   The algorithm tries to solve full_A*full_x >= full_b, where full_x is the solution
   constraints: full_x >= 0.0 and if full_x[i]>0 than equation i has to hold
   Modification: with n_equations you can specify that the first part of A and x are treated as a system of
                 equations this means full_A*full_x = full_b for i<= n_equations. full_x is then allowed to be negative
   
   During all computations the most important variables are:
     vector int:  active_inactive_ignored (has information about what variables are active,inactive or not processed yet)
     integer      r_actives (number of active variables)
     integer      ignored_pos (number of next unknown to process)
   Given that information we can calculate all other information during a setup in case numerical errors accumulate.
   The most valuable thing to lose in that setup is the LU information. We can recompute it with
   lu_sub_solver::l_u_decomposition_with_pivoting() but that costs 3X a gaussian elemination
   
   lu_sub_solver: here is stored the LU decompostion of the sub Matrix A of active variables
                  in case an active variable is changed to inactive we call lu_sub_solver::decrement_l_u
		  in case an inactive/ignored variable becomes active we call lu_sub_solver::increment_l_u

   accel: stores vector values full_A*full_x - full_b 
          has to be zero for the active variables and >zero for inactives and some known value for ignored variables

   delta_x: during a step holds the changes of x when moving 1.0 with the latest ignored variable
            has to be zero for inactive variables (their x values are and shall stay zero)

   delta_accel: during a step holds the changes of accel when moving 1.0 with the latest ignored variable
                has to be zero for active variables (their accels are and shall stay zero)


   if accel > 0.0 for next ignored variable, we can move that variable to inactives. Else we have to do a standard step

   standard step:
     step begins with call to 'get_xdirection'. There the delta_x values are calculated. Delta_x for new ignored
     variable is 1.0 for the rest of the actives delta_x is unknown.
     So A*delta_x = delta_accel
        A* (? ? .. ? 1.0) = ( 0 0 0 .. 0 alpha )  sub matrix A is (r_actives+1)*(r_actives+1)
	A* delta_x + ai = ( 0 0 0 0 0 0 0)  sub matrix A is (r_actives*r_actives) and ai is ith column of A
	we have to solve:
	                   A * delta_x = -ai
        we solve this with our LU solver or with gauss-eleminiation

     After we have delta_x we calculate delta_accel: delta_accel = A * delta_x (and get value alpha)

     Then we calculate step size s:
     s = -accel[i]/delta_x[i] for current ignored variable to fullfill A*x>=b
     s = -full_x/delta_x for actives to avoid x-values getting negative
     s = -accel/delta_accel for inactives to avoid >=b violated
     Three possibilites: ignored variable becomes active, one active becomes inactive, one inactive becomes active
     Then we have to update accel and full_x and do next step


// */


//can not be vector optimized because of indexed access
void hk_LCP_Solver::mult_active_x_for_accel() {
    int i,j;
    for(i=r_actives;i<n_variables;i++) {
	hk_real sum=0.0f;
	int row_of_A=actives_inactives_ignored[i];
	hk_real *row_vals=&full_A[ row_of_A * aligned_size ];
	int real_pos;
	for(j=0;j<r_actives;j++) {
	    real_pos=actives_inactives_ignored[j];
	    sum+= delta_x[real_pos]*row_vals[real_pos];
	}
	real_pos=actives_inactives_ignored[ignored_pos];
	sum+=row_vals[real_pos];
	delta_accel[row_of_A]=sum;
    }
}

void hk_LCP_Solver::mult_x_with_full_A_minus_b() {
#if 0
    for(int i=0;i<n_variables;i++) {
	    IVP_DOUBLE *base_a=&full_A[i*aligned_size];
	    IVP_DOUBLE sum=IVP_VecFPU::fpu_large_dot_product(base_a,full_x,n_variables,HK_TRUE);
	    temp[i] = sum-full_b[i];
    }
#endif
    input_struct->m_A->mult_vector(full_x,temp);
    hk_VecFPU::fpu_add_multiple_row(temp,full_b,-1.0f,n_variables,HK_TRUE);
}

//quite costly, because we have to multiply with the full matrix
// we check wether the active variables match with the right side full_b
//    and we check wether the inactives are greater equal the right side full_b
hk_bool hk_LCP_Solver::numerical_stability_ok() {
    mult_x_with_full_A_minus_b();
    int k;
    hk_real val;
    for(k=0;k<r_actives;k++) {
	val=hk_Math::fabs(temp[actives_inactives_ignored[k]]);
	if( val > HK_LCP_TEST_EPS ) {
	    return HK_FALSE;
	}
    }
    for(k=r_actives; k<n_variables; k++) {
	val=hk_Math::fabs( temp[actives_inactives_ignored[k]] - accel[actives_inactives_ignored[k]] );
	if( val > HK_LCP_TEST_EPS ) {
	    HK_LCP_IF(debug_lcs) {
		hk_Console::get_instance()->printf("numerical_unstable %d %.10f should be %.10f\n",ignored_pos,temp[actives_inactives_ignored[k]],accel[actives_inactives_ignored[k]]);
	    }
	    return HK_FALSE;
	}
    }
    return HK_TRUE;
}

hk_result hk_LCP_Solver::solve_lcp(hk_LCP_Input &in,hk_LCP_Temp &tmp,hk_LCP_Output &out) {
    input_struct=&in;
    m_gauss_inp.m_A=(hk_gauss_real*)tmp.m_L_matrix; //aligned Matrix
    m_gauss_inp.m_b=(hk_gauss_real*)tmp.m_temp_vec; //aligned vector  
    m_gauss_out.m_result=(hk_gauss_real*)tmp.m_inout_vec; //aligned vector
    //Gauss solver shares all its memory with LU_Solver. Gauss is only needed when LU is invalid

    lu_sub_solver.m_L_matrix=tmp.m_L_matrix; //aligned Matrix
    lu_sub_solver.m_U_matrix=tmp.m_U_matrix; //aligned Matrix	
    lu_sub_solver.m_inout_vec=tmp.m_inout_vec; //aligned vector
    lu_sub_solver.m_temp_vec=tmp.m_temp_vec;  //aligned vector

    full_A=in.m_A->getRealPointer(); //aligned matrix, points to input
    full_b=in.m_b; //aligned vector, points to input
    full_x=out.m_result; //aligned vector, points to output
    n_variables=in.m_n_variables;
    n_equations=in.m_n_equations;
    aligned_size=hk_VecFPU::calc_aligned_row_len(n_variables,full_A);
    
    r_actives=0;
    ignored_pos=0;

    sub_solver_status=0;
    
    lu_sub_solver.m_aligned_row_len=aligned_size;
    lu_sub_solver.m_n_sub=0;
    
    m_gauss_inp.m_gauss_eps=HK_LCP_SOLVER_EPS;     // for gauss a more liberal EPS is enough (otherwise it may happen the unilateral solver to fail completely)
                                            // sorry, but for xdirection its not acceptable to have very large values
    
    m_max_steps_lcp=3*in.m_n_variables;

    actives_inactives_ignored=tmp.m_actives_inactives_ignored;
    accel =        tmp.m_accel;
    delta_accel =  tmp.m_delta_accel;
    delta_x     =  tmp.m_delta_f;
    temp  =        (hk_real*)tmp.m_temp_vec;

    int i;
    for(i=0;i<aligned_size;i++)
	{
		actives_inactives_ignored[i]=i;
		full_x[i] = 0.0f;
		accel[i] = -full_b[i];
    }

    first_permute_index=0;
    second_permute_index=0;
    first_permute_ignored=0;
    second_permute_ignored=0;
    
    debug_no_lu_count=0;
    debug_lcs = 0;

#if 0 //only for debug
    HK_LCP_IF(debug_lcs)
	{
		hk_Console::get_instance()->printf("\n***********************************************************\n");
		start_debug_lcs();
    }
#endif

    startup_setup( in.m_n_actives_at_start );
    hk_result ret_val = solve_lc();

    for(i=r_actives-1;i>=0;i--) {
	out.m_active[actives_inactives_ignored[i]]=HK_TRUE;
    }
    for(i=r_actives;i<n_variables;i++) {
	out.m_active[this->actives_inactives_ignored[i]]=HK_FALSE;
    }

    //only for debug
    HK_LCP_IF((debug_no_lu_count>0) && (debug_lcs)) {
	hk_Console::get_instance()->printf("sum_lu_failed %d\n",debug_no_lu_count);
    }
    
    HK_LCP_IF(ret_val==HK_OK) {
	input_struct->m_A->mult_vector( out.m_result, temp );

	int j;
	for(j=0;j<n_variables;j++) {
	    if(temp[j] + 0.0001f < full_b[j]) {
		hk_Console::get_instance()->printf("linear_constraint_failed %d %f should be greater %f\n",j,temp[j],full_b[j]);
	    }
	}
	for(j=0;j<r_actives;j++) {
	    int index_full=actives_inactives_ignored[j];
	    hk_real diff=hk_Math::fabs(temp[index_full] - full_b[index_full]);
	    if(diff>0.0001f) {
		hk_Console::get_instance()->printf("linear_constraint_failed %d %f should be equal %f\n",j,temp[index_full],full_b[index_full]);
	    }
	}
    }    
    
    return ret_val;
}


void hk_LCP_Solver::decrement_sub_solver( int sub_pos ) {
    if(sub_solver_status==0) {
	HK_ASSERT( lu_sub_solver.m_n_sub == r_actives + 1 );
	if( lu_sub_solver.decrement_l_u( sub_pos ) != HK_OK ) {
#if 0
	    HK_LCP_IF(1) {
		hk_Console::get_instance()->printf("\n\ndecrement_lu_failed_need_setup\n\n");
	    }
#endif
	    sub_solver_status=2;
	}
    } else {
	lu_sub_solver.m_n_sub--;
    }
}

void hk_LCP_Solver::increment_sub_solver() {
    int k;
    int new_var_orig = actives_inactives_ignored[ r_actives-1 ];
    if(sub_solver_status==0) {
	HK_ASSERT( lu_sub_solver.m_n_sub == r_actives - 1);
	for(k=0;k<r_actives;k++) {
	    lu_sub_solver.m_U_matrix[ lu_sub_solver.m_n_sub * lu_sub_solver.m_aligned_row_len + k ] = full_A[ new_var_orig*aligned_size + actives_inactives_ignored[ k ] ]; 
	}
	for(k=0;k<r_actives-1;k++) {
	    lu_sub_solver.m_inout_vec[k] = full_A[ aligned_size * actives_inactives_ignored[ k ] + new_var_orig ];
	}
	if( lu_sub_solver.increment_l_u() != HK_OK ) {
	    HK_LCP_IF(debug_lcs) {
		hk_Console::get_instance()->printf("\n\nincrement_lu_failed_need_setup\n\n");
	    }
	    sub_solver_status=2;
	}
    } else {
	lu_sub_solver.m_n_sub++;
    }
}

void hk_LCP_Solver::do_a_little_random_permutation() {
    // in case numeric gets unstable, neverending loops can occur
    // do some permutions to get fresh values ( in gauss the last variable gets zero, this "wrong" values are distributed alternatively )

    if( r_actives-n_equations < 2) {
	goto permute_free_variables;
    }

    first_permute_index +=1;
    second_permute_index +=2;

    while(first_permute_index >= (r_actives-n_equations)) {
	first_permute_index-=(r_actives-n_equations);
    }
    while(second_permute_index >= (r_actives-n_equations)) {
	second_permute_index-=(r_actives-n_equations);
    }
    exchange_lcs_variables( first_permute_index+n_equations, second_permute_index+n_equations );

    first_permute_ignored +=1;
    second_permute_ignored +=2;

permute_free_variables:
    int free_space=n_variables-ignored_pos-1;
    if(free_space<2) {
	return;
    }
    while(first_permute_ignored >= free_space) {
	first_permute_ignored-=free_space;
    }
    while(second_permute_ignored >= free_space) {
	second_permute_ignored-=free_space;
    }
    exchange_lcs_variables( first_permute_ignored+ignored_pos+1, second_permute_ignored+ignored_pos+1 );

}

void hk_LCP_Solver::move_not_necessary_actives_to_inactives() {
    int i;
    for( i=n_equations;i<r_actives;i++ ) {
	if( full_x[ actives_inactives_ignored[i] ] < HK_LCP_SOLVER_EPS ) {
	    //hk_Console::get_instance()->printf("juhu_removed_not_necessary_active %d full %d\n",i,actives_inactives_ignored[i]);
	    full_x[ actives_inactives_ignored[i] ]=0.0f;
	    exchange_lcs_variables( i, r_actives-1 );
	    r_actives--;
	    decrement_sub_solver( i );
	    i--;
	}
    }
}

// actives_inactives_ignored : array to transform index from small variable space (that is growing) to full variable space
// first are active variables: their accel - val is 0.0f and x val is > 0.0f
// then inactives: their x val is 0.0f and accel val is > 0.0f
// last ignored: x and accel unknown, first of ignored is processed

// doctrin:
// when step size and variables are near to 0.0f than try to move corresponding variable into inactive variables (makes algorithm faster (sub matrix small) and robust (sub matrix not singular)) 
// but moving variables with x=0.0f into active cannot be avoided: sometimes it is necessary when moving first ignored into actives (as second step)
hk_result hk_LCP_Solver::solve_lc()
{
    int index; //var to go from sub space to full space
    int step_count=0;
    int nr_zero_steps=0; //to avoid endless loops
    int did_real_step=0; //real step means that active variables have changed
    int next_numeric_stability_test = HK_LCP_NUMERIC_TEST_EVERY_N;

    //hk_Console::get_instance()->printf("\n\n\n\n\ndoing_lcs_ %d\n",n_variables);
    
    hk_incrlu_real solver_eps = HK_LCP_SOLVER_EPS;
    
    while(1)
	{
		if(did_real_step)
		{
			increase_step_count(&step_count);
		}
		if(step_count > m_max_steps_lcp)
		{
			HK_LCP_IF(1)
			{
				hk_Console::get_instance()->printf("\n\ngiveup_stepcount_linear_constraint_solver\n\n\n");
			}
			return HK_FAULT;
		}
		
		int limiting_var;
		
		if( next_numeric_stability_test == 0 )
		{
			if( (numerical_stability_ok() == HK_FALSE) )
			{
perform_full_setup:
				increase_step_count(&step_count);
    			if(step_count > m_max_steps_lcp)
				{
					HK_LCP_IF(1)
					{
						hk_Console::get_instance()->printf("\n\ngiveup_stepcount_linear_constraint_solver\n\n\n");
					}
					return HK_FAULT;
				}
				
				nr_zero_steps=0;
				if( full_setup() == HK_FAULT )
				{
					return HK_FAULT;
				}
			}
			next_numeric_stability_test = HK_LCP_NUMERIC_TEST_EVERY_N;
		}
		else
		{
			next_numeric_stability_test--;
		}
		
		if( ignored_pos >= n_variables )
		{
			return HK_OK;
		}
		
		int ignored_full_index = actives_inactives_ignored[ ignored_pos ];

		HK_LCP_IF(debug_lcs)
		{
			int k;
			for(k=n_equations;k<r_actives;k++)
			{
				int idx=actives_inactives_ignored[k];
				HK_LCP_IF( hk_Math::fabs( accel[idx] ) > 0.01f )
				{
					hk_Console::get_instance()->printf("incorrect_accel %f at %d  should be zero\n",accel[idx],idx);
				}
				HK_LCP_IF( hk_Math::fabs( full_x[idx] < solver_eps ))
				{
					if ( hk_Math::fabs( full_x[ignored_full_index] ) < solver_eps)
					{
						hk_Console::get_instance()->printf("active_var %d shouldnt be active but inactive\n",idx);
					}
				}
				if( full_x[idx]<0.0f )
				{
					HK_LCP_IF(1)
					{
						hk_Console::get_instance()->printf("active_var %d should be positive %f\n",idx,full_x[idx]);
					}
				}
			}
		}
		HK_LCP_IF(1)
		{
			int k;
			for(k=r_actives;k<ignored_pos;k++)
			{
				int idx=actives_inactives_ignored[k];
				HK_LCP_IF( hk_Math::fabs( full_x[idx] ) > 0.1f )
				{
					hk_Console::get_instance()->printf("incorrect_x %f at %d  should be zero\n",accel[idx],idx);
				}
				if(full_x[idx]<-0.001f)
				{
					hk_Console::get_instance()->printf("incorrect_negative_x %f at %d  should be zero\n",accel[idx],idx);
				}
				if( accel[idx] < 0.0f )
				{
					hk_Console::get_instance()->printf("warning_inactive_neg accel %d %f\n",idx,accel[idx]);
				}	
			}
		}
		
		HK_LCP_IF(debug_lcs)
		{
			debug_out_lcs();
		}

		HK_LCP_IF( hk_Math::fabs( full_x[ ignored_full_index ] ) < solver_eps )
		{
			debug_test_all_values();
		}
		
		if(( sub_solver_status > 0))
		{
			if( (sub_solver_status == 1) && (did_real_step) )
			{
				do_a_little_random_permutation();
				if(setup_l_u_solver()==HK_OK)
				{
					sub_solver_status=0; 
				}
				else
				{
					HK_LCP_IF(debug_lcs)
					{
						hk_Console::get_instance()->printf("setup_lu_notok\n");
					}
				}
			}
			else
			{
				sub_solver_status = 1; //ok then next time
			}
		}
		
		if( hk_Math::fabs( accel[ignored_full_index] ) < solver_eps )
		{
			// my processed variable has no acceleration and so could be an inactive var
			// except the force x is greater 0.0f than we must move it to actives
			limiting_var=ignored_pos;
			if( hk_Math::fabs( full_x[ignored_full_index] ) < solver_eps )
			{
				goto move_ignored_to_inactive;
			}
			else
			{
				goto move_ignored_to_active;
			}
		}
		
		if(accel[ignored_full_index] < 0.0f)
		{
			hk_real smallest_step;
			
			//first r vars are clamped
			//compute x with Acc x = b
			if(get_xdirection() == HK_FAULT)
			{
				HK_LCP_IF(debug_lcs)
				{
					hk_Console::get_instance()->printf("lcs_fdirection_total_fail\n");
				}
				goto perform_full_setup;
			}
			delta_x[ ignored_full_index ] = 1.0f;
			
			//delta_x is now filled in original var space
	    			
			//full_solver_mat.mult();  //delta_accel = A * delta_x
			//delta_accel is now filled in original var space
			mult_active_x_for_accel();

			int i;
			for(i=0;i<r_actives;i++)
			{
				int index_full = actives_inactives_ignored[i];
				delta_accel[index_full] = 0.0f; //just to overwrite rounding-errors
			}
			
			HK_LCP_IF(0)
			{
				for(int j=0;j<r_actives;j++)
				{
					int full_index = actives_inactives_ignored[ j ];
					hk_Console::get_instance()->printf("should be zero %f\n",delta_accel[full_index]);
				}
			}
			
			// now find smallest steps
			
			// step to make clamped active out of ignored_pos
			HK_LCP_IF( accel[ ignored_full_index ] > 0.0f )
			{
				hk_Console::get_instance()->printf("errror_accel_ignored_negativ\n");
			}
			if( -accel[ ignored_full_index ] < delta_accel[ ignored_full_index ] * HK_LCP_MAX_STEP_LEN )
			{
				smallest_step = - accel[ignored_full_index] / delta_accel[ ignored_full_index ];
				HK_ASSERT( smallest_step > 0.0f );
				limiting_var = ignored_pos;
			}
			else
			{
				limiting_var = -1;
				smallest_step = HK_LCP_REAL_MAX;
			}


			// smallest step of clamped (active)
			// if we increase value of ignored variable, equations will allways hold true whatever step size we choose
			// however the x-value of active variables are not allowed to pass through zero
			for(i=n_equations;i<r_actives;i++)
			{
				index = actives_inactives_ignored[i];
				hk_real del_f = delta_x[ index ];
				if( del_f < - solver_eps )
				{
					hk_real local_step = - full_x[ index ] / del_f;
					if( (hk_Math::fabs( local_step ) < solver_eps) && ( full_x[ index ] < solver_eps ) )
					{
						limiting_var = i;
						smallest_step = local_step;
						goto limiting_var_found;
					}
					if( local_step < smallest_step + solver_eps)
					{ //make it easier to move from active to inactive
						smallest_step = local_step;
						limiting_var = i;
					}
				}
			}

			// smallest step of unclamped
			for(i=r_actives;i<ignored_pos;i++)
			{
				index = actives_inactives_ignored[i];
				hk_real del_a = delta_accel[ index ];
				if( del_a < -HK_LCP_SOLVER_EPS )
				{
					hk_real local_step = - accel[ index ] / del_a;
					if( local_step < smallest_step - solver_eps )
					{ // make it harder to move from inactive to active
						HK_LCP_IF(debug_lcs)
						{
							hk_Console::get_instance()->printf("violate_inactive: accel %.4f  delta %.4f\n",accel[index],del_a);
						}			
						// we prefer making inactives out of actives instead other way round if both is possible ( both steps are nearly zero )
						// uuups this is all wrong
						if( (hk_Math::fabs( local_step ) > solver_eps ) || (limiting_var==ignored_pos) || 1)
						{
							smallest_step = local_step;
							limiting_var = i;
						}
					}
				}
			}

			if(smallest_step < 0.0f)
			{
				HK_LCP_IF(1)
				{
					hk_Console::get_instance()->printf("error_stepsize_negative %f\n",smallest_step);
				}
				smallest_step=0.0f;
			}	
			
			if( limiting_var < 0 )
			{
				return HK_FAULT;
			}

			HK_LCP_IF(debug_lcs)
			{
				hk_Console::get_instance()->printf("smallest_step_is %f\n",smallest_step);
			}

limiting_var_found:

			if(smallest_step > HK_LCP_MAX_STEP_LEN)
			{
				goto perform_full_setup;
			}
			if(smallest_step < HK_LCP_SOLVER_EPS)
			{
				nr_zero_steps++;
				if(nr_zero_steps > (n_variables>>1) +2 )
				{ //heuristic
					goto perform_full_setup; //does a little random permutation, we hope next time no endless loop
				}
			}
			else
			{
				nr_zero_steps=0;
			}
			did_real_step=1;
			update_step_vars( smallest_step );
			
			if(limiting_var < r_actives)
			{
				// one active var becomes inactive
				
				HK_LCP_IF(debug_lcs)
				{
					hk_Console::get_instance()->printf("\nactive %d becomes inactive (relp %d)\n\n",actives_inactives_ignored[limiting_var],limiting_var);
				}
				
				int indx=actives_inactives_ignored[limiting_var];
				HK_ASSERT( (hk_Math::fabs(full_x[indx]) < 0.001f) || (hk_Math::fabs( smallest_step ) < solver_eps) );
				full_x[indx]=0.0f;
				r_actives--;
				//exchange_activ_inactiv(limiting_var);
				exchange_lcs_variables(r_actives,limiting_var);
				decrement_sub_solver( limiting_var );
			}
			else
			{
				if(limiting_var < ignored_pos)
				{
					// one inactive var becomes active
					if(smallest_step > HK_LCP_SOLVER_EPS)
					{
						move_not_necessary_actives_to_inactives();
					}

					HK_LCP_IF(debug_lcs)
					{
						hk_Console::get_instance()->printf("\ninactive %d becomes active (relp %d)\n\n",actives_inactives_ignored[limiting_var],limiting_var);
					}
					int indx=actives_inactives_ignored[limiting_var];
					HK_ASSERT( hk_Math::fabs(accel[indx]) < 0.001f );
					accel[indx]=0.0f;
					//exchange_activ_inactiv(limiting_var);
					exchange_lcs_variables(r_actives,limiting_var);
					r_actives++;
					increment_sub_solver();
				}
				else
				{
	move_ignored_to_active:
					// ignored_pos comes to the active variables		    
					move_not_necessary_actives_to_inactives();		    
					HK_LCP_IF(debug_lcs)
					{
						hk_Console::get_instance()->printf("\nignored %d becomes active (relp %d)\n\n",actives_inactives_ignored[ignored_pos],ignored_pos);
					}
					int indx=actives_inactives_ignored[limiting_var];
					HK_ASSERT( hk_Math::fabs(accel[indx]) < 0.001f );
					accel[indx]=0.0f;
					if(full_x[indx]<0.0f)
					{ //really necessary ?
						full_x[indx]=0.0f;
					}
					//exchange_activ_inactiv(limiting_var);
					exchange_lcs_variables(r_actives,limiting_var);
					ignored_pos++;
					r_actives++;
					if(ignored_pos<n_variables)
					{
						increment_sub_solver();
					}
					else
					{
						next_numeric_stability_test=0;
					}
					HK_LCP_IF(0)
					{
						debug_test_all_values();
					}
				}
			}
		}
		else
		{
	move_ignored_to_inactive:	    
			// ignored_pos comes to the inactives variables
			did_real_step = 0; //active variables are unchanged
			HK_LCP_IF(debug_lcs)
			{
				hk_Console::get_instance()->printf("ignored %d becomes inactive (relp %d)\n\n",actives_inactives_ignored[ignored_pos],ignored_pos);
			}
			int indx = actives_inactives_ignored[ignored_pos];
			HK_LCP_IF( !(hk_Math::fabs(full_x[indx] ) < 0.001f) )
			{
				hk_Console::get_instance()->printf("error_move_ignored_to_inactive_x_not_zero\n");
			}
			
			full_x[indx] = 0.0f;
			if( accel[indx] < 0.0f)
			{ //really necessary ?
				accel[indx] = 0.0f;
			}
			ignored_pos++;
			if(ignored_pos >= n_variables)
			{
				next_numeric_stability_test=0;
			}	    
			//reset_values();
		}
	}

    HK_LCP_IF(debug_lcs)
	{
		debug_out_lcs();
    }
    
    return HK_OK;
}

//both numbers are in sub-space
void hk_LCP_Solver::exchange_lcs_variables( int first_nr, int second_nr ) {
    HK_ASSERT( first_nr >= 0 );
    HK_ASSERT( second_nr >= 0 );
    HK_ASSERT( first_nr < n_variables );
    HK_ASSERT( second_nr < n_variables );
    int full_space_0,full_space_1;
    full_space_0=actives_inactives_ignored[second_nr];
    full_space_1=actives_inactives_ignored[first_nr];
    actives_inactives_ignored[first_nr]=full_space_0;
    actives_inactives_ignored[second_nr]=full_space_1;
}
//please merge these two functions !!!!
void hk_LCP_Solver::exchange_activ_inactiv( int change_var ) {
    int first_inactiv=r_actives;
    int full_space_0,full_space_1;
    full_space_0=actives_inactives_ignored[change_var];
    full_space_1=actives_inactives_ignored[first_inactiv];
    actives_inactives_ignored[first_inactiv]=full_space_0;
    actives_inactives_ignored[change_var]=full_space_1;
}

//optimization: do not go from 0 to n_variables
//              make two loops: one for actives, one for others
void hk_LCP_Solver::update_step_vars( hk_real step_size ) {
    int i;
    HK_LCP_IF(debug_lcs) {
	hk_Console::get_instance()->printf("deltaa  ");
    }
    for( i = 0; i < n_variables; i++) {
	HK_LCP_IF(debug_lcs) {
	    hk_Console::get_instance()->printf("%f ",delta_accel[i]);
	}
	accel[ i ] = accel[i] + step_size * delta_accel[ i ]; //Vector Optimization possible
	full_x[ i ] = full_x[ i ] + step_size * delta_x[ i ];           //dito
    }
    HK_LCP_IF(debug_lcs) {
	hk_Console::get_instance()->printf("\n");
    }

    //in some cases (e.g. overriding an inactive) acceleration of inactives gets negative
    for( i=r_actives; i<ignored_pos; i++ ) {
	if(accel[actives_inactives_ignored[i]] < 0.0f) {
	    accel[actives_inactives_ignored[i]]=0.0f;  
	}
    }
    //same with actives
    for(i=0;i<r_actives;i++) {
	if(full_x[actives_inactives_ignored[i]] < 0.0f) {
	    full_x[actives_inactives_ignored[i]]=0.0f;
	}
    }
}
    
hk_result hk_LCP_Solver::get_xdirection() {
    // clear all
    int i,k;
    for(i=0;i<aligned_size;i++) {
        this->delta_x[ i ] = 0.0f;
    }

    if(r_actives==0) {
	return HK_OK;
    }

    hk_result ret_val = HK_FAULT;
    

    if(sub_solver_status == 0) {
	int ignored_index = actives_inactives_ignored[ ignored_pos ];

	for(k=0;k<r_actives;k++) {
	    int index=actives_inactives_ignored[k];
	    lu_sub_solver.m_inout_vec[k]= - this->full_A[index * aligned_size + ignored_index];
	}
	lu_sub_solver.solve_lin_equ();
	ret_val = HK_OK;
#if 0	
	HK_LCP_IF(debug_lcs) {
	    sub_solver_mat.columns = r_actives;
	    sub_solver_mat.calc_aligned_row_len();
	    sub_solver_mat.MATRIX_EPS = GAUSS_EPS * 0.001f; //we know our matrix is stable, so it is save to raise EPS
	    inv_mat.columns = r_actives;
	    debug_mat.columns = r_actives;
	    
	    for(i=0;i<r_actives;i++) {
		int index=actives_inactives_ignored[i];
		sub_solver_mat.desired_vector[i]= - this->full_A[index * aligned_size + ignored_index];
		debug_mat.desired_vector[i]= - this->full_A[index * aligned_size + ignored_index];
		int j;
		for(j=0;j<r_actives;j++) {
		    int index2=actives_inactives_ignored[j];
		    sub_solver_mat.matrix_values[i*sub_solver_mat.aligned_row_len + j] = this->full_A[ index * aligned_size + index2 ];
		    inv_mat.matrix_values[i*r_actives + j] = this->full_A[ index * aligned_size + index2 ];
		}
	    }
	    hk_result ret_val2 = sub_solver_mat.solve_great_matrix_many_zero();
	    hk_result ret_val3 = HK_FAULT; //inv_mat.invert(&debug_mat);
	    if( ret_val2 == HK_OK ) {
		for( i=0;i<r_actives;i++ ) {
		    IVP_DOUBLE diff=lu_sub_solver.out_vec[i]-sub_solver_mat.result_vector[i];
		    if(hk_Math::fabs(diff) > 0.001f) {
		      HK_LCP_IF(debug_lcs) {
			hk_Console::get_instance()->printf("gauss_lu_test_fail %f %f should be equal\n",lu_sub_solver.out_vec[i],sub_solver_mat.result_vector[i]);
		      }
		    }
		}
	    } else {
	      HK_LCP_IF(debug_lcs) {
		hk_Console::get_instance()->printf("gauss_lu_test_gauss_invalid\n");
	      }
	    }
	    if( ret_val3 == HK_OK ) {
		debug_mat.mult();
		for( i=0;i<r_actives;i++ ) {
		    IVP_DOUBLE diff=lu_sub_solver.out_vec[i]-debug_mat.result_vector[i];
		    if(hk_Math::fabs(diff) > 0.001f) {
		      HK_LCP_IF(debug_lcs) {
			hk_Console::get_instance()->printf("inv_lu_test_fail %f %f should be equal\n",lu_sub_solver.out_vec[i],debug_mat.result_vector[i]);
		      }
		    }
		}
	    }
	    sub_solver_mat.MATRIX_EPS = GAUSS_EPS;
	}
#endif
    } else {
	HK_LCP_IF((debug_lcs)) {
	    hk_Console::get_instance()->printf("sub_lu_is_off %d\n",r_actives);
	}
	debug_no_lu_count++;

	m_gauss_inp.m_n_columns = r_actives;
	m_gauss_inp.m_aligned_row_len = hk_VecFPU::calc_aligned_row_len(r_actives,m_gauss_inp.m_A);

	int ignored_index = actives_inactives_ignored[ ignored_pos ];
    
	for(i=0;i<r_actives;i++) {
	    int index=actives_inactives_ignored[i];
	    m_gauss_inp.m_b[i]= - this->full_A[index * aligned_size + ignored_index];

	    int j;
	    for(j=0;j<r_actives;j++) {
		int index2=actives_inactives_ignored[j];
		m_gauss_inp.m_A[i*m_gauss_inp.m_aligned_row_len + j] = this->full_A[ index * aligned_size + index2 ];

	    }
	}
	ret_val = m_gauss_sub_solver.solve_gauss_elemination( m_gauss_inp, m_gauss_out);
	for( i=0;i<r_actives;i++ ) {
	    lu_sub_solver.m_inout_vec[i]=m_gauss_out.m_result[i];
	}
    }
    
    //copy clamped
    for(i=0;i<r_actives;i++) {
	this->delta_x[ actives_inactives_ignored[i] ] = (hk_real)lu_sub_solver.m_inout_vec[i];
    }

    this->delta_x[ actives_inactives_ignored[ignored_pos] ] = 1.0f;
    
    return ret_val;
}

void hk_LCP_Solver::start_debug_lcs() {
    hk_Console::get_instance()->printf("b_vals ");
    int i;
    int j;
    for(i=0;i<n_variables;i++) {
	hk_Console::get_instance()->printf("%.4f ",full_b[i]);
    }
    hk_Console::get_instance()->printf("\n");
    
    hk_Console::get_instance()->printf("matrix:\n");
    for(i=0;i<n_variables;i++) {
	for(j=0;j<n_variables;j++) {
	    hk_Console::get_instance()->printf("%.4f ",full_A[i*aligned_size + j]);
	}
	hk_Console::get_instance()->printf("\n");
    }
    hk_Console::get_instance()->printf("\n");
}

void hk_LCP_Solver::debug_out_lcs() {
    hk_Console::get_instance()->printf("r_actives %d  ignored_nr %d\n",r_actives,ignored_pos);
    int i;

    if(ignored_pos>n_variables) {
	ignored_pos=n_variables;
    }
    
    hk_Console::get_instance()->printf("actives_inactives  ");
    for(i=0;i<r_actives;i++) {
	hk_Console::get_instance()->printf("%d ",actives_inactives_ignored[i]);
    }
    hk_Console::get_instance()->printf("  ");
    for(i=r_actives;i<ignored_pos;i++) {
	hk_Console::get_instance()->printf("%d ",actives_inactives_ignored[i]);
    }
    hk_Console::get_instance()->printf("  ");
    for(i=ignored_pos;i<n_variables;i++) {
	hk_Console::get_instance()->printf("%d ",actives_inactives_ignored[i]);
    }
    hk_Console::get_instance()->printf("\n");

    hk_Console::get_instance()->printf("x-vals ");
    for(i=0;i<n_variables;i++) {
	hk_Console::get_instance()->printf("%.4f ",full_x[i]);
    }
    hk_Console::get_instance()->printf("\n");

#if 0
    full_solver_mat.desired_vector = full_x;
    full_solver_mat.mult_aligned(); // temporary missuse
    full_solver_mat.desired_vector = delta_x;
    
    hk_Console::get_instance()->printf("accel1 ");
    for(i=0;i<n_variables;i++) {
	hk_Console::get_instance()->printf("%.4f ",full_solver_mat.result_vector[i] - full_b[i]);
    }
    hk_Console::get_instance()->printf("\n");
#endif

    hk_Console::get_instance()->printf("incr_accel ");
    for(i=0;i<n_variables;i++) {
	hk_Console::get_instance()->printf("%.4f ",accel[i]);
    }
    hk_Console::get_instance()->printf("\n\n");
  
}

//cannot be vector optimized because of indexed access
hk_result hk_LCP_Solver::setup_l_u_solver() {
    int i;
    HK_LCP_IF(0) {
	hk_Console::get_instance()->printf("setting_up_lu\n");
    }
    
    for(i=0; i <r_actives;i++) {
	    int index = actives_inactives_ignored[i];
	    //sub_solver_mat.desired_vector[i]= b[index];
	    lu_sub_solver.m_inout_vec[i]=full_b[index];
	    hk_real *source = & this->full_A[ index * aligned_size ];
	    hk_incrlu_real *dest = & lu_sub_solver.m_U_matrix[ i * lu_sub_solver.m_aligned_row_len ];
	    for(int j=0;j<r_actives;j++) {
	        int index2=actives_inactives_ignored[j];
		dest[j] = source[index2];
	    }
    }
    
    return lu_sub_solver.l_u_decomposition_with_pivoting();
}

void hk_LCP_Solver::lcs_bubble_sort_x_vals() {
    int r1=n_equations;
    int r2;
    while(1) {
	r2=r1+1;
	if(r2>=r_actives) {
	    break;
	}
	if( full_x[ actives_inactives_ignored[ r1 ] ] < full_x[ actives_inactives_ignored[ r2 ] ] ) {
	    exchange_lcs_variables( r1, r2 );
	    int r_t = r1;
	    while( (r_t > n_equations) && ( full_x[ actives_inactives_ignored [r_t-1] ] < full_x[ actives_inactives_ignored[ r_t ] ] ) ) {
		exchange_lcs_variables( r_t, r_t-1 );
		r_t--;
	    }
	}
	r1=r2;
    }

    HK_LCP_IF(1) {
	int i;
	for(i=1;i<r_actives;i++) {
	    if( full_x[ actives_inactives_ignored[ i-1 ] ] < full_x[ actives_inactives_ignored[ i ] ] ) {
		hk_Console::get_instance()->printf("lcs_bubble_failed\n");
	    }
	}
    }
}

// fills accel and full_x vectors
void hk_LCP_Solver::get_values_when_setup() {
#if 0
    full_solver_mat.result_vector=accel;
    full_solver_mat.desired_vector=full_x; //temporary misuse
    full_solver_mat.mult_aligned();
    full_solver_mat.result_vector=delta_accel;
    full_solver_mat.desired_vector=delta_x;
#endif

    input_struct->m_A->mult_vector( full_x, accel );

    int i;
    {
	for(i=0;i<n_variables;i++) {
	    accel[i]-=full_b[i];
	}
    }

    for(i=0;i<r_actives;i++) {
	accel[actives_inactives_ignored[i]]=0.0f; //just to remove rounding errors
    }
}

void hk_LCP_Solver::startup_setup(int try_actives)
{
    r_actives = try_actives;
    aligned_sub_size=hk_VecFPU::calc_aligned_row_len(r_actives,lu_sub_solver.m_L_matrix);
    ignored_pos=try_actives;
    lu_sub_solver.m_n_sub = r_actives;

    int i;
    for(i=0;i<aligned_size;i++)
	{
		full_x[i]=0.0f;
    }
    if(setup_l_u_solver() == HK_OK)
	{
		sub_solver_status = 0;
		HK_LCP_IF(debug_lcs)
		{
			lu_sub_solver.debug_print_l_u();
			int j;
			hk_Console::get_instance()->printf("\n\nline48:\n ");
			for(j=0;j<r_actives;j++)
			{
				hk_Console::get_instance()->printf("%.3f ",lu_sub_solver.m_L_matrix[lu_sub_solver.m_aligned_row_len*48+j]);
			}
			hk_Console::get_instance()->printf("\n\n");
		}
        //input vec for lu_sub_solver is filled within setup_l_u_solver

		while(1)
		{
begin_of_while:	    
			//solution_not_found=0;
			lu_sub_solver.solve_lin_equ();
			for(i=0;i<r_actives;i++)
			{
				int index=actives_inactives_ignored[i];
				full_x[index]=(hk_real)lu_sub_solver.m_inout_vec[i];
			}
			get_values_when_setup(); //fills x[i] and accel[i]

			for(i=r_actives-1;i>=n_equations;i--)
			{
				int index=actives_inactives_ignored[i];
				if(full_x[ index ] < 0.0f)
				{
					full_x[ index ] = 0.0f;
					r_actives--;
					ignored_pos--;
					exchange_lcs_variables(r_actives,i);
					decrement_sub_solver(i);
					if(sub_solver_status > 0) 
					{
						for(i=0;i<n_variables;i++)
						{
						   accel[i]=0.0f;
						   full_x[i]=0.0f;
						}
						goto fast_setup_failed;
					}
					else 
					{
						for(i=0;i<r_actives;i++) 
						{
							index=actives_inactives_ignored[i];
							lu_sub_solver.m_inout_vec[i]=full_b[index];
						}
						goto begin_of_while;
					}
				}
			}
			break;
		}
    }
	else
	{
fast_setup_failed:	
		// sorry fast setup failed
		r_actives=n_equations;
		ignored_pos=n_equations;
		lu_sub_solver.m_n_sub=n_equations;
    }
}

// full setup is done when numerical problems are encountered.
// lu_solver is setup with pivoting to get a stable inverted matrix
// for the active variables
hk_result hk_LCP_Solver::full_setup() {
    HK_LCP_IF(0) {
	hk_Console::get_instance()->printf("doing_fulll_setup\n");
    }
    
    int i;
    lu_sub_solver.m_n_sub = r_actives;
    
    for(i=0;i<aligned_size;i++) {
	full_x[i]=0.0f;
    }

    do_a_little_random_permutation();
    if( setup_l_u_solver() == HK_OK ) {
	sub_solver_status = 0;
        //input vec for lu_sub_solver is filled within setup_l_u_solver
	lu_sub_solver.solve_lin_equ();
	for(i=0;i<r_actives;i++) {
	    int index=actives_inactives_ignored[i];
	    full_x[index]=(hk_real)lu_sub_solver.m_inout_vec[i];
	}
    } else {
	sub_solver_status = 2;
	HK_LCP_IF(debug_lcs) {
	    hk_Console::get_instance()->printf("setup_lu_failed\n");
	}

	lcs_bubble_sort_x_vals(); //move actives with lowest x to the end (matrix is singular and last vars get a zero)
	                          //but giving the lowest x's the zeros is just a heuristic
	do_a_little_random_permutation();
	
	m_gauss_inp.m_n_columns = r_actives;
	m_gauss_inp.m_aligned_row_len = hk_VecFPU::calc_aligned_row_len(r_actives,m_gauss_inp.m_A);
	
	for(i=0;i<r_actives;i++) {
	    int index=actives_inactives_ignored[i];
	    m_gauss_inp.m_b[i]= full_b[index];

	    int j;
	    for(j=0;j<r_actives;j++) {
		int index2=actives_inactives_ignored[j];
		m_gauss_inp.m_A[i*m_gauss_inp.m_aligned_row_len + j] = this->full_A[ index * aligned_size + index2 ];

	    }
	}
	hk_result ret_val = m_gauss_sub_solver.solve_gauss_elemination( m_gauss_inp, m_gauss_out);

	if(ret_val!=HK_OK) {
	    HK_LCP_IF(1) {
		hk_Console::get_instance()->printf("error_setup_gauss_failed_too\n");
	    }
	    return ret_val;
	}
    
	for(i=0;i<r_actives;i++) {
	    int index=actives_inactives_ignored[i];
	    full_x[index]=(hk_real)m_gauss_out.m_result[i];
	}
    }

    get_values_when_setup();
    
    if( full_setup_test_ranges() > 0 ) {
	HK_LCP_IF(debug_lcs) {
	    hk_Console::get_instance()->printf("\ncleared_some_illegal_vars\n\n");
	}
	return full_setup();
    } else {
	
    }
    
    return HK_OK;
    //optimization:
    //fill directly in data, not versus reset_*
}

void hk_LCP_Solver::move_variable_to_end( int var_nr ) {
    int i;
    for(i=var_nr+1;i<n_variables;i++) {
	exchange_lcs_variables( i-1, i );
    }
}	

// returns number of removed actives
int hk_LCP_Solver::full_setup_test_ranges() {
    int illegal_vars=0;
    int i;
    for(i=n_equations;i<r_actives;i++) {
	if( full_x[actives_inactives_ignored[i]] < HK_LCP_SOLVER_EPS ) {
	    if( full_x[actives_inactives_ignored[i]] > -HK_LCP_SOLVER_EPS ) {
		full_x[actives_inactives_ignored[i]] = 0.0f;
		exchange_lcs_variables( i, r_actives-1 );
	    } else {
		move_variable_to_end( i );
		illegal_vars++;
		ignored_pos--;
	    }
	    i--;
	    r_actives--;
	    lu_sub_solver.m_n_sub--;
	    sub_solver_status=1;
	}
    }
    for(i=r_actives;i<ignored_pos;i++) {
	if( accel[actives_inactives_ignored[i]] < 0.0f ) {
	    move_variable_to_end( i );
	    i--;
	    ignored_pos--;
	}
    }
    return illegal_vars;
}

void hk_LCP_Solver::debug_test_all_values() {
    //TL: doesn't make sense, because reset_x and reset_accel have been removed 
#if 0
    int i;

    for(i=0;i<aligned_size;i++) {
	full_x[i] = full_x[i];
    }

    input_struct->m_A->mult_vector(full_x,accel);

#if 0
    full_solver_mat.result_vector = accel;
    full_solver_mat.desired_vector = full_x; //temporary misuse
    full_solver_mat.mult_aligned();
    full_solver_mat.result_vector = delta_accel;
    full_solver_mat.desired_vector = delta_x;
#endif

    for(i=0;i<aligned_size;i++) {
	accel[i]-=full_b[i];
    }

    HK_LCP_IF(debug_lcs) {
	for(i=0;i<n_variables;i++) {
	    hk_real diff=hk_Math::fabs( accel[i] - accel[i] );
	    if( diff > 0.001f ) {
		hk_Console::get_instance()->printf("accel_violated at %d %f should be %f\n",i,accel[i],accel[i]);
	    }
	    diff=hk_Math::fabs( full_x[i] - full_x[i] );
	    if( diff > 0.001f ) {
		hk_Console::get_instance()->printf("x_violated at %d\n",i);
	    }
	}
    }
#endif
}
