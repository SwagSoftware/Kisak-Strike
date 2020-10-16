// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


// IVP_EXPORT_PRIVATE

#include <ivp_physics.hxx>

#include <ivu_memory.hxx>
#include <ivp_great_matrix.hxx>
#include <ivp_core_macros.hxx>

#include <ivp_mindist_intern.hxx> // because of Mindist
#include <ivp_friction.hxx>
#include <ivp_friction_solver.hxx>
#include <ivp_debug_manager.hxx> // bvecause of debug psi_synchrone

#if !defined(WIN32) && !defined(PSXII) && !defined(GEKKO)
#	include <alloca.h>
#endif

inline IVP_DOUBLE ivp_frs_min(IVP_DOUBLE a,IVP_DOUBLE b){
    if(a<b) return a;
    return b;
}


IVP_RETURN_TYPE IVP_Friction_Solver::test_gauss_solution_suggestion(IVP_DOUBLE *push_results,int *active_is_at_pos,int total_actives,IVP_U_Memory *mem_friction){
    ivp_u_bool *was_already_tested=(int*)mem_friction->get_mem(dist_change_mat.columns*sizeof(ivp_u_bool));  
    memset((char*)was_already_tested,0,sizeof(int)*dist_change_mat.columns); // init with false = 0

    int gauss_failed=0;
    {
	for(int i=0;i<total_actives;i++){
	    int orig_index = active_is_at_pos[i];
	    IVP_Impact_Solver_Long_Term *info = contact_info_vector.element_at(orig_index);

	    IVP_DOUBLE push_val = push_results[i];
	    //printf("gauss_value %f  ",push_val);
	
	    IVP_DOUBLE allowed_negative = IVP_MAX_ADHESION_GAUSS * l_environment->gravity_scalar;
	
	    //printf("allowed_negative %f\n",allowed_negative);
	    if(allowed_negative > push_val * correct_x_factor * get_inv_virtual_mass(info) ) {
		IVP_IF((l_environment->debug_information->debug_friction)||0){
		    printf("gauss_fault_pull %f %f\n",allowed_negative,push_val);
		}
		//return IVP_FAULT;
		gauss_failed=1;
	    }
	    was_already_tested[ orig_index ] = IVP_TRUE;
	    dist_change_mat.result_vector[ orig_index ] = push_val;
	}
    }
    {
	for(int i=0 ; i < dist_change_mat.columns ; i++)   {
	    if(was_already_tested[i]==IVP_FALSE){
		if(dist_change_mat.matrix_check_unequation_line(i) == IVP_FAULT)	    {
		    //printf("gauss_unequation_failed\n");
		    gauss_failed=1;
		    return IVP_FAULT;
		} 
	    }
	}}
    if(gauss_failed) {
	return IVP_FAULT;
    }
    return IVP_OK;
}

void IVP_Friction_Solver::factor_result_vec() {
    int i;
    for( i=dist_change_mat.columns-1;i>=0;i-- ) {
	dist_change_mat.result_vector[i] *= correct_x_factor;
    }
}

void IVP_Friction_Solver::normize_constraint_equ() {
    IVP_DOUBLE max_val=0.0f;
    int i;
    // find highest diagonal element
    for(i=dist_change_mat.columns-1;i>=0;i--) {
	IVP_DOUBLE test_val=dist_change_mat.matrix_values[ i * dist_change_mat.aligned_row_len + i ];
	if( test_val > max_val ) {
	    max_val = test_val;
	}
    }

    IVP_DOUBLE a_norm;
    if( max_val > P_DOUBLE_EPS ) {
	a_norm=1.0f/max_val;
    } else {
	a_norm=1.0f;
    }
    
    correct_x_factor = a_norm;
    
    max_val=0.0f;
    // find highest value in desired vector
    for(i=dist_change_mat.columns-1;i>=0;i--) {
	IVP_DOUBLE test_val=IVP_Inline_Math::fabsd( dist_change_mat.desired_vector[i] );
	if( test_val > max_val ) {
	    max_val = test_val;
	}
    }
    
    IVP_DOUBLE b_norm;
    if( max_val > P_DOUBLE_EPS ) {
	b_norm=1.0f/max_val;
    } else {
	b_norm=1.0f;
	max_val=1.0f;
    }

    correct_x_factor *= max_val;

    // #+# use fpu to do this
    for( i=0;i<dist_change_mat.columns;i++ ) {
	IVP_DOUBLE *base=&dist_change_mat.matrix_values[i*dist_change_mat.aligned_row_len];
	for( int j=0;j<dist_change_mat.aligned_row_len;j++ ) {
	    base[j] *= a_norm;
	}
    }

    for( i=dist_change_mat.columns-1;i>=0;i-- ) {
	dist_change_mat.desired_vector[i] *= b_norm;
    }
}

// absorb this function somewhere !
int IVP_Friction_System::get_num_supposed_active_frdists() {
    int sum=0;
    IVP_Contact_Point *fr_dist;
    for(fr_dist=this->get_first_friction_dist();fr_dist;fr_dist=this->get_next_friction_dist(fr_dist)) {
	if(fr_dist->has_negative_pull_since < 0) {
	    sum++;
	} else {
	    return sum;
	}
    }
    return sum;
}


// 'total_actives' is number of active mindists to try gauss with
// array 'active_is_at_pos' has 'total_actives' entries and gives original pos of an active
void IVP_Friction_Solver::solve_linear_equation_and_push(IVP_Friction_System *my_fs,int *active_is_at_pos,int total_actives,IVP_U_Memory *mem_friction)
{
    int supposed_actives;
    IVP_Great_Matrix_Many_Zero actives_matrix;
    actives_matrix.columns=total_actives;
    actives_matrix.calc_aligned_row_len();

    this->normize_constraint_equ();    
    actives_matrix.desired_vector = (IVP_DOUBLE*)mem_friction->get_mem(actives_matrix.aligned_row_len*sizeof(IVP_DOUBLE));
    actives_matrix.result_vector = (IVP_DOUBLE*)mem_friction->get_mem(actives_matrix.aligned_row_len*sizeof(IVP_DOUBLE));
    actives_matrix.matrix_values = (IVP_DOUBLE*)mem_friction->get_mem((total_actives*actives_matrix.aligned_row_len+IVP_VECFPU_SIZE-1)*sizeof(IVP_DOUBLE));
    actives_matrix.align_matrix_values();


    memset((char*)dist_change_mat.result_vector,0,dist_change_mat.aligned_row_len*sizeof(IVP_DOUBLE));

    actives_matrix.fill_from_bigger_matrix(&dist_change_mat,active_is_at_pos, total_actives);

    IVP_Linear_Constraint_Solver constraint_unilateral;
    
    if( actives_matrix.solve_great_matrix_many_zero() == IVP_OK )
    {
	if(( test_gauss_solution_suggestion(actives_matrix.result_vector,active_is_at_pos,total_actives,mem_friction) == IVP_OK))
	{
            IVP_IF(1) {
	        this->gauss_succed++;
            }
	    goto first_try;
	}
	// no result with cheap method, lets do some complex ...    
    }

    supposed_actives=my_fs->get_num_supposed_active_frdists();
    //if(dist_change_mat.solve_lp_with_own_complex(mem_friction)==IVP_OK) {
    if(constraint_unilateral.init_and_solve_lc(dist_change_mat.matrix_values, dist_change_mat.desired_vector, dist_change_mat.result_vector, dist_change_mat.columns, supposed_actives, mem_friction) == IVP_OK) {
first_try:
	this->factor_result_vec();
	IVP_DOUBLE e_before=my_fs->kinetic_energy_of_hole_frs();
	this->do_resulting_pushes(my_fs);
	IVP_DOUBLE e_after = my_fs->kinetic_energy_of_hole_frs();
	IVP_DOUBLE e_max_grown = my_fs->get_max_energy_gain();
	if( e_after <= e_before+e_max_grown ) {
	    my_fs->confirm_complex_pushes();
	    goto finish_slp;
	} else {
	    IVP_IF(1) {
		printf("too_much_energy_gained\n");
	    }
	    my_fs->undo_complex_pushes();
	}
    } else {
	IVP_IF(1) {
	    printf("static_friction_unequation failed\n");
	}
    }

    //this->complex_failed(my_fs);
    
finish_slp:
    actives_matrix.desired_vector=NULL;
    actives_matrix.result_vector=NULL;
    actives_matrix.matrix_values=NULL;
}

void IVP_Friction_System::do_pushes_distance_keepers(const IVP_Event_Sim *es_in) {
    IVP_Friction_System *fs=this;

    for ( IVP_Contact_Point *fr_dist=fs->get_first_friction_dist();
	  fr_dist;
	  fr_dist = this->get_next_friction_dist(fr_dist)){
	
	fr_dist->now_friction_pressure=0.0f;
	
	IVP_Core *core0=fr_dist->get_synapse(0)->l_obj->friction_core;
	IVP_Core *core1=fr_dist->get_synapse(1)->l_obj->friction_core;
	IVP_BOOL allow0=core0->fast_piling_allowed();
	IVP_BOOL allow1=core1->fast_piling_allowed();
	if(( (!allow0) || (!allow1) )) {
	    continue;
	}

	IVP_FLOAT speedup;
	speedup = (
		   (IVP_FLOAT)(IVP_SLOWLY_TURN_ON_KEEPER - fr_dist->slowly_turn_on_keeper) *
		   ( IVP_FLOAT(1.0f / IVP_SLOWLY_TURN_ON_KEEPER)  * es_in->i_delta_time)
		   );

	IVP_Impact_Solver_Long_Term *info = fr_dist->tmp_contact_info;
	IVP_DOUBLE closing_speed = info->get_closing_speed();
	IVP_DOUBLE desired_gap = ivp_mindist_settings.keeper_dist;
	IVP_DOUBLE gap_diff = desired_gap - fr_dist->get_gap_length();
	IVP_DOUBLE a = closing_speed + gap_diff * speedup;
	IVP_DOUBLE b = info->virtual_mass;
	IVP_DOUBLE impulse = a * b;

	if(impulse > 0.0f) {
	    fr_dist->now_friction_pressure = impulse * es_in->i_delta_time;    
	    IVP_Friction_Solver::apply_impulse( info, impulse );
	    //IVP_DOUBLE closing_speed2 = info->get_closing_speed();
	    //printf("%x gap %f single old %f new %f  impulse %f\n", this, gap_diff, closing_speed, closing_speed2, impulse);
	} else {
	    fr_dist->now_friction_pressure=0.0f;
	}
	
	if(fr_dist->slowly_turn_on_keeper <= 0) {
	    fr_dist->slowly_turn_on_keeper = 0;
	} else {
	    fr_dist->slowly_turn_on_keeper--;
	}
    }
}

IVP_FLOAT IVP_Friction_Solver::do_penalty_step( IVP_FLOAT *impulses, IVP_FLOAT *pretension, IVP_FLOAT force_factor, IVP_FLOAT damp_factor){
    // forward step
    {
    for (int i = contact_info_vector.len()-1; i>=0;i--){
	IVP_Impact_Solver_Long_Term *info = contact_info_vector.element_at(i);
	IVP_DOUBLE delta_velocity = info->get_closing_speed();
	IVP_DOUBLE a = delta_velocity * damp_factor +  pretension[i] * force_factor;
	IVP_DOUBLE b = info->virtual_mass;
	IVP_DOUBLE impulse = a * b;

	IVP_DOUBLE new_sum = impulses[i] + impulse;
	if (new_sum < 0){
	    impulse = -impulses[i];
	    new_sum = 0.0f;
	}
	impulses[i] = new_sum;
	apply_impulse( info, impulse);
    }
    }
    {
    // update pretension
    for (int i = contact_info_vector.len()-1; i>=0;i--){
	IVP_Impact_Solver_Long_Term *info = contact_info_vector.element_at(i);
	IVP_DOUBLE delta_velocity = info->get_closing_speed();
	IVP_DOUBLE a = pretension[i];
	a += delta_velocity;
	pretension[i] = a;
    }
    }

    {
    // backward step
    for (int i = 0; i< contact_info_vector.len(); i++){
	IVP_Impact_Solver_Long_Term *info = contact_info_vector.element_at(i);
	IVP_DOUBLE delta_velocity = info->get_closing_speed();
	IVP_DOUBLE a = delta_velocity * damp_factor +  pretension[i] * force_factor;
	IVP_DOUBLE b = info->virtual_mass;
	IVP_DOUBLE impulse = a * b;

	IVP_DOUBLE new_sum = impulses[i] + impulse;
	if (new_sum < 0){
	    impulse = -impulses[i];
	    new_sum = 0.0f;
	}
	impulses[i] = new_sum;
	apply_impulse( info, impulse);
    }
    }
    // update pretension
    IVP_DOUBLE sum_quad_pretension = 0.0f;
    for (int i = contact_info_vector.len()-1; i>=0;i--){
	IVP_Impact_Solver_Long_Term *info = contact_info_vector.element_at(i);
	IVP_DOUBLE delta_velocity = info->get_closing_speed();
	IVP_DOUBLE a = pretension[i];
	a += delta_velocity;
	if (impulses[i]){
	    sum_quad_pretension += a * a;
	}
	pretension[i] = a;
    }
    
    return sum_quad_pretension;
}
#if 0
void IVP_Friction_Solver::do_penalty_method(IVP_Friction_System *fs){
    IVP_FLOAT *impulses   = (IVP_FLOAT*)alloca( sizeof(IVP_FLOAT) * contact_info_vector.len());
    IVP_FLOAT *pretension = (IVP_FLOAT*)alloca( sizeof(IVP_FLOAT) * contact_info_vector.len());
    
    // compress now_friction press into shorter array
    for ( IVP_Contact_Point *fr_dist=fs->get_first_friction_dist();
	  fr_dist;
	  fr_dist = fs->get_next_friction_dist(fr_dist)){
	IVP_Impact_Solver_Long_Term *info = fr_dist->get_lt();
	int i = info->index_in_fs;
	if (i<0) continue;
	
	IVP_DOUBLE desired_gap = ivp_mindist_settings.keeper_dist;
	IVP_DOUBLE gap_diff = desired_gap - fr_dist->get_gap_length();
	//desired_speed_to_close_gap[i] = gap_diff * es->i_delta_time;
	pretension[i] = gap_diff * es->i_delta_time * 0.1f;
	//impulses[i] = 0.0f;
	impulses[i] = 0.5f * fr_dist->now_friction_pressure * es->delta_time;

	//	IVP_DOUBLE delta_velocity = info->get_closing_speed();
	//printf("%x %i old %f pressure %f\n", fr_dist, i, delta_velocity, impulses[i]);
    }

    IVP_IF(0){
	for (int i = contact_info_vector.len()-1; i>=0;i--){
	    IVP_Impact_Solver_Long_Term *info = contact_info_vector.element_at(i);
	    IVP_DOUBLE delta_velocity = info->get_closing_speed();
	    printf("%i old %f\n", i, delta_velocity);
	}
    }
    
    // first do old now friction pressures
    for (int i = contact_info_vector.len()-1; i>=0;i--){
	IVP_Impact_Solver_Long_Term *info = contact_info_vector.element_at(i);
	IVP_DOUBLE push_val = impulses[i];
	apply_impulse( info, push_val );
    }

    IVP_IF(0){
	for (int i = contact_info_vector.len()-1; i>=0;i--){
	    IVP_Impact_Solver_Long_Term *info = contact_info_vector.element_at(i);
	    IVP_DOUBLE delta_velocity = info->get_closing_speed();
	    printf("%i new %f\n", i, delta_velocity);
	}
    }
    
    // do the steps (max 50)
    if (1){
	//IVP_FLOAT max_sum_qdiff = 0.01f * 0.01f;
	printf("penalty step: ");
	for (int s = 10; s>=0; s--){
	    IVP_FLOAT sum_qdiff = do_penalty_step( impulses, pretension, 0.5f, 0.9f );
	    printf("%f ",sum_qdiff);
	    //if (sum_qdiff < max_sum_qdiff) break;
	};
	printf("\n");
    }
    
    { ; // uncompress now_friction_pressure
	for ( IVP_Contact_Point *fr_dist=fs->get_first_friction_dist();
	      fr_dist;
	      fr_dist = fs->get_next_friction_dist(fr_dist)){
	    IVP_Impact_Solver_Long_Term *info = fr_dist->get_lt();
	    int i = info->index_in_fs;
	    if (i<0){
		fr_dist->now_friction_pressure = 0.0f;
		continue;
	    }
	    fr_dist->now_friction_pressure = impulses[i] * es->i_delta_time;
	}
    }
}
#endif

// fuer glaettung des complex
void IVP_Friction_Solver::do_inactives_pushes(IVP_Friction_System *fs){
    int counter_two=0;
    for(IVP_Contact_Point *fr_dist = fs->get_first_friction_dist();
	fr_dist;
	fr_dist=fs->get_next_friction_dist(fr_dist), counter_two++) {	
	if(fr_dist->has_negative_pull_since==0) {	    
	    IVP_Impact_Solver_Long_Term *info = fr_dist->get_lt();
	    IVP_DOUBLE push_val = fr_dist->now_friction_pressure;
	    apply_impulse( info, push_val * es->i_delta_time );
	}
    }
}

void IVP_Friction_Solver::debug_distance_after_push(int counter)
{
    IVP_USE(counter);
#if 0 /* muss noch umgeschrieben werden */   
    IVP_Core *core0,*core1;
    core0=cores_of_dist[counter*2];
    core1=cores_of_dist[counter*2+1];
    IVP_U_Float_Point speed_world0,speed_world1;

    IVP_U_Float_Point rotation,translation;
    rotation.set(&core0->rot_speed);
    rotation.add(&core0->rot_speed_change);
    translation.set(&core0->speed);
    translation.add(&core0->speed_change);
    core0->get_surface_speed_on_test(&obj_points_dist[counter*2],&translation,&rotation,&speed_world0);

    rotation.set(&core1->rot_speed);
    rotation.add(&core1->rot_speed_change);
    translation.set(&core1->speed);
    translation.add(&core1->speed_change);
    core1->get_surface_speed_on_test(&obj_points_dist[counter*2+1],&translation,&rotation,&speed_world1);

    speed_world1.subtract(&speed_world0);
    IVP_DOUBLE new_dist_speed=speed_world1.dot_product(&dist_vector_world[counter]);
    IVP_IF(environment->debug_information->debug_impact)
    {
	printf("dist_speed_d: distance %.5f  speed:  before %.5f  after %.5f dis_change %.5f\n",dist_len[counter],dist_velocity[counter],new_dist_speed,new_dist_speed-dist_velocity[counter]);
    }
#endif    
}

void IVP_Friction_Solver::complex_failed(IVP_Friction_System *fs) {
    IVP_Contact_Point *my_dist;
    int i;
    my_dist=fs->get_first_friction_dist();
    for(i=0;i<fs->friction_dist_number;i++)
    {
#if !defined(IVP_DISTANCE_KEEPERS) && !defined(IVP_PANELTY_SOLVER)
	my_dist->now_friction_pressure=0.0f;
#endif	
	my_dist=fs->get_next_friction_dist(my_dist);
    }
}

// returns 1 if a friction distance has to be removed (not used any more)
int IVP_Friction_Solver::do_resulting_pushes(IVP_Friction_System *fs)
{

    int m_index = 0;
    int active_count = 0;
    for ( IVP_Contact_Point *my_dist=fs->get_first_friction_dist();
	  my_dist;
	  m_index++, my_dist=fs->get_next_friction_dist(my_dist)){
	IVP_DOUBLE impulse_now;
	
	if((m_index < fs->complex_not_necessary_number)||0) {
	    impulse_now=0.0f;
	    goto no_complex_done;
	}
	
	impulse_now = dist_change_mat.result_vector[m_index];
	    
	if(impulse_now>0.0f){
	    if(my_dist->has_negative_pull_since>=0){
		my_dist->has_negative_pull_since=-1; //first positive push
	    } else {
		my_dist->has_negative_pull_since--;
	    }
	} else {
	    if(impulse_now==0.0f){
		//inactive
		my_dist->has_negative_pull_since=0;
		impulse_now = my_dist->now_friction_pressure * 0.0f * this->es->delta_time;
		goto cont_for_next;
	    } else {
		// negative push
		if(my_dist->has_negative_pull_since<0) {
		    my_dist->has_negative_pull_since=0;
		}
		my_dist->has_negative_pull_since++;
		if(my_dist->has_negative_pull_since > 9)  {//allow only 9 negative pushes in following PSIs
			my_dist->has_negative_pull_since=0; //next time inactiv
		}
	    }
	}
	{
	    IVP_Impact_Solver_Long_Term *info = my_dist->get_lt();	    
	    this->async_apply_impulse(info, impulse_now);
	}
	active_count ++;
    
	  cont_for_next:	    
	if(impulse_now<0.0f) {
	    impulse_now=0.0f;
	}
	
	  no_complex_done:
	
#if !defined(IVP_DISTANCE_KEEPERS) && !defined(IVP_PANELTY_SOLVER)
	my_dist->now_friction_pressure = impulse_now * this->es->i_delta_time;
#else	    
	my_dist->now_friction_pressure += impulse_now * this->es->i_delta_time; //keine Glaettung der Reibung
#endif	
	
    }
    IVP_IF(l_environment->debug_information->debug_friction){
	for(int j=0;j<fs->friction_dist_number;j++) {
	    IVP_IF(j>=fs->complex_not_necessary_number) {
		debug_distance_after_push(j-fs->complex_not_necessary_number);
	    }
	}
    }
    //printf("n active lines %i\n", active_count);
    
#ifdef DEBUG
    IVP_IF(l_environment->debug_information->debug_friction)
    {
	printf("pushes_ok. resulting_distance_velos:\n");
	this->print_dist_velocity(fs);
    }
#endif	/* DEBUG */
    
    return 0;
}

#ifdef DEBUG
void IVP_Friction_Solver::print_dist_velocity(IVP_Friction_System *fs)
{
    //int dist_counter=0;
    int dist_counter_two=0;
    int total_dist_counter=-1;
    IVP_Contact_Point *my_dist,*next_dist=NULL;
    my_dist=fs->get_first_friction_dist();
    while(my_dist)
    {
	total_dist_counter++;
	next_dist=fs->get_next_friction_dist(my_dist);

	if(total_dist_counter<fs->complex_not_necessary_number) {
	    my_dist=next_dist;
	    continue;
	}
		
	IVP_Synapse_Friction *syn0 = my_dist->get_synapse(0);
	IVP_Synapse_Friction *syn1 = my_dist->get_synapse(1);
	IVP_Core *core0,*core1;
	core0 = syn0->get_object()->get_core();
	core1 = syn1->get_object()->get_core();

	IVP_U_Float_Point w_speed0,w_speed1;
	IVP_U_Float_Point speed_var_trans,speed_var_rot;
	speed_var_trans.add(&core0->speed,&core0->speed_change);
	speed_var_rot.add(&core0->rot_speed,&core0->rot_speed_change);
	core0->get_surface_speed_on_test(&my_dist->tmp_contact_info->contact_point_cs[0],&speed_var_trans,&speed_var_rot,&w_speed0);
	speed_var_trans.add(&core1->speed,&core1->speed_change);
	speed_var_rot.add(&core1->rot_speed,&core1->rot_speed_change);
	core1->get_surface_speed_on_test(&my_dist->tmp_contact_info->contact_point_cs[1],&speed_var_trans,&speed_var_rot,&w_speed1);

	IVP_U_Float_Point rel_world_speed;
	rel_world_speed.subtract(&w_speed1,&w_speed0);

	IVP_IF(1) {
	    printf("dvelo_ %.4f   ",rel_world_speed.dot_product(&my_dist->get_lt()->surf_normal));
	}
       
	dist_counter_two++;
	my_dist=next_dist;
    }
    IVP_IF(1) {
	printf("\n");
    }
}
#endif


void IVP_Friction_System::test_hole_fr_system_data()
{
    IVP_IF(1) {
    //printf("hole_fr_test %lx \n",(long)this);
    IVP_Friction_System *fs=this;    
    IVP_Contact_Point *mindist=fs->get_first_friction_dist();
    if (!mindist) return;
    
#if 0	
    int startnumber;
    {
	startnumber=mindist->number_in_friction+1;
	if(startnumber!=fs->friction_dist_number)
	{
	    printf("test_fr dist_number wrong\n");
	    CORE;
	}
    }
#endif	
    int two_count=0;
    while(mindist)
    {
#if 0	
	startnumber--;
	if(startnumber<0)
	{
	    printf("test_fr number <0");
	    CORE;
	}

	if(startnumber!=mindist->number_in_friction)
	{
	    printf("test_fr wrong number");
	    CORE;
	}
#endif
	for(int i=0;i<2;i++){
	    IVP_Core *r_obj= mindist->get_synapse(i)->l_obj->friction_core;	    
	    IVP_Real_Object *obj_obj=r_obj->objects.element_at(0);
	    IVP_Time time=r_obj->environment->get_current_time();
	    
	    IVP_Friction_Info_For_Core *fr_info = r_obj->get_friction_info(fs);

	    if(fr_info->l_friction_system!=this){
		printf("test_fr l_friction_system %f %lx\n",time.get_time(),(long)obj_obj); CORE;
	    }
	    
	    int found_mine=0;
	    for (int j = fr_info->friction_springs.len()-1; j>=0;j--){
		IVP_Contact_Point *all_dists = fr_info->friction_springs.element_at(j);
		if(all_dists==mindist){
		    found_mine=1;
		} else {
		    int is_in_collection=0;
		    IVP_Contact_Point *temp_dist = fs->get_first_friction_dist();
		    while(temp_dist){
			if(temp_dist==all_dists){
			    is_in_collection=1;
			}
			temp_dist=fs->get_next_friction_dist(temp_dist);
		    }
		    if(!is_in_collection){
			printf("test_fr there was mindist %lx in obj which is not in system\n",(long)all_dists&0x0000ffff); CORE; 
		    }
		}
	    }
	    if(!found_mine)
	    {
		printf("test_fr mindist %lx missing in obj\n",(long)mindist); CORE;
	    }
	}

	{
	    //test if mindist is found in pairs
	    int found_mine=0;
	    for (int k = fr_pairs_of_objs.len()-1; k>=0;k--){
		IVP_Friction_Core_Pair *fr_pair = fr_pairs_of_objs.element_at(k);
		for (int n = fr_pair->fr_dists.len()-1; n>=0; n--){
		    IVP_Contact_Point *fr_dist=fr_pair->fr_dists.element_at(n);
		    if(fr_dist==mindist) {
			found_mine=1;
		    }
		}
	    }
	    if(!found_mine) {
		printf("test_fr dist %lx not found in pairs\n",(long)mindist);
		CORE;
	    }
	}
	
	two_count+=2;
	mindist=fs->get_next_friction_dist(mindist);
    }

    {
	//test if all dists in pairs are in system
	for (int n = fr_pairs_of_objs.len()-1; n>=0; n--){
	    IVP_Friction_Core_Pair *fr_pair = fr_pairs_of_objs.element_at(n);
	    for (int m = fr_pair->fr_dists.len()-1; m>=0; m--){
		IVP_Contact_Point *fr_dist = fr_pair->fr_dists.element_at(m);
		int found_mine=0;
		for(IVP_Contact_Point *mindist2=fs->get_first_friction_dist();mindist2;mindist2=fs->get_next_friction_dist(mindist2)){
		    if(mindist2==fr_dist) {
			found_mine=1;
		    }
		}
		if(!found_mine)	{
		    printf("test_fr pair %lx containing dist %lx not in system\n",(long)fr_pair,(long)fr_dist);
		    CORE;
		}
	    }
	}
    }
    }
}

#if 0
// doesnt work when a object has more than one friction system
void check_fr_info_mindist(IVP_Friction_Solver *fsolv,IVP_Contact_Point *my_dist,int dist_counter_two)
{
    IVP_Real_Object *obj0=(IVP_Real_Object*)my_dist->synapse[0]->l_obj;
    IVP_Real_Object *obj1=(IVP_Real_Object*)my_dist->synapse[1]->l_obj;

    IVP_Friction_Info_For_Obj *fr_i;
    fr_i=obj0->get_first_friction_info_of_obj();
    if(fr_i!=fsolv->friction_infos[dist_counter_two])
    {
	printf("fr_info_damaged obj %lx\n",(long)obj0&0x0000ffff);
	CORE;
    }
    fr_i=obj1->get_first_friction_info_of_obj();
    if(fr_i!=fsolv->friction_infos[dist_counter_two+1])
    {
	printf("fr_info_damaged obj %lx\n",(long)obj1&0x0000ffff);
	CORE;
    }
}
#endif


int global_friction_counter=0;

void IVP_Friction_Solver::setup_coords_mindists(IVP_Friction_System *fs)
{
    int index_of_contact_point = 0;
    int total_dist_counter = 0;
    for (IVP_Contact_Point *my_dist = fs->get_first_friction_dist();
	 my_dist;
	 my_dist = fs->get_next_friction_dist(my_dist)){

	IVP_Impact_Solver_Long_Term *info = my_dist->tmp_contact_info;

	if(total_dist_counter < fs->complex_not_necessary_number) {
	    info->index_in_fs = -1; //not used in complex
	    continue;
	}
	total_dist_counter++;
	
	info->friction.friction_infos[0] = my_dist->get_synapse(0)->get_object()->get_core()->get_friction_info(fs); // #+# bad bad
	info->friction.friction_infos[1] = my_dist->get_synapse(1)->get_object()->get_core()->get_friction_info(fs);
	info->friction.dist_len = my_dist->get_gap_length();
	info->friction.has_negative_pull_since = my_dist->has_negative_pull_since; // copy to avoid later cache misses

	contact_info_vector.add(info);
	info->index_in_fs = index_of_contact_point++;
    } // for all contact_points
}


#if 0
	IVP_Synapse_Friction *syn0 = my_dist->get_synapse(0);
	IVP_Synapse_Friction *syn1 = my_dist->get_synapse(1);

	IVP_Friction_Solver_Data *frisodat=&frimindist_data[dist_counter_two];

	frisodat->dist_vector_obj[0]=my_dist->tmp_contact_info->surface_normal_cs[0];
	frisodat->dist_vector_obj[1]=my_dist->tmp_contact_info->surface_normal_cs[1];	
 	frisodat->cores_of_dist[0]= syn0->l_obj->physical_core;
	frisodat->cores_of_dist[1]= syn1->l_obj->physical_core;
	
	//vielleicht nicht die IVP_Friction_Info_For_Obj speichern sondern gleich den IVP_U_Vector der mindists
	frisodat->friction_infos[0]=syn0->l_obj->friction_core->get_friction_info(fs);
	frisodat->friction_infos[1]=syn1->l_obj->friction_core->get_friction_info(fs);

	IVP_IF(0) {
	    IVP_U_Float_Point space_pointer;
	    IVP_U_Point world_points_dist[2];
	    IVP_U_Matrix *mat;
	    mat=&frisodat->cores_of_dist[0]->m_world_f_core_last_psi;
	    mat->vmult4(&frisodat->contact_point_cs[0],&world_points_dist[0]);
	    mat=&frisodat->cores_of_dist[1]->m_world_f_core_last_psi;
	    mat->vmult4(&frisodat->contact_point_cs[1],&world_points_dist[1]);

	    space_pointer.subtract(&world_points_dist[1],&world_points_dist[0]);
	    char *out_text=p_make_string("d%d_%.1f",my_dist->has_negative_pull_since,my_dist->now_friction_pressure);
	    syn0->l_obj->get_environment()->add_draw_vector(&world_points_dist[0],&space_pointer,out_text,1);
	    P_FREE (out_text);
	}

	//IVP_U_Float_Point diff_vec_world; //vector of position difference. points from first synapse to second
	//diff_vec_world.subtract(&world_points_dist[1],&world_points_dist[0]);
	//frisodat->dist_len=diff_vec_world.fast_real_length();
	frisodat->dist_len = my_dist->get_gap_length();
	//diff_vec_world.normize();
	//diff_vec_world.set(0.0f,1.0f,0.0f);
	//printf("difff %f %f %f\n",diff_vec_world.k[0],diff_vec_world.k[1],diff_vec_world.k[2]);
	//frisodat->dist_vector_world=diff_vec_world;

	frisodat->dist_vector_world.set_multiple(&my_dist->get_lt()->surf_normal,+1.0f);
	
	IVP_IF(l_environment->debug_information->debug_impact)
	{
	    //printf("dist_vec_world %.3f %.3f %.3f\n",dist_vector_world[dist_counter].k[0],dist_vector_world[dist_counter].k[1],dist_vector_world[dist_counter].k[2]);
	}
#endif


void out_friction_info_obj(IVP_Core *obj)
{
    return; //absturz
    IVP_IF(1) {
	IVP_Friction_Info_For_Core *fr_info=obj->moveable_core_has_friction_info();
	while(fr_info)    {
	    printf("obj %lx fs %lx obj_ma %d",(long)obj&0x0000ffff,(long)fr_info->l_friction_system&0x0000ffff,obj->physical_unmoveable);
	    
	    for (int i = fr_info->friction_springs.len()-1; i>=0; i--){
		IVP_Contact_Point *mindist = fr_info->friction_springs.element_at(i);
		printf("  md %lx",(long)mindist&0x0000ffff);
	    }
	    fr_info=NULL;
	    printf("\n");
	}
    }
}

/* friction distance with number 'current_contact_point_index' is tested. Therefore both objects of distance get a test push
 * Velocity changes are represented with 'rotation_vec' (obj coords) and 'translation_vec' (world coords)
 * then this funcion is called for both objects
 * for one object is looked if other distances are inflicted (speed of distance changes)
 * if yes, speed change is represented in a line of the matrix, with line number = number of the distance
 * values are added because a column of the matrix is built with testing of two objects (example: diagonal elements, but not only)
 */
void IVP_Friction_Solver::calc_distance_matrix_column(int current_contact_point_index,
						      IVP_Core *core,IVP_Friction_Info_For_Core *fr_info,
						      IVP_U_Float_Point *rotation_vec,IVP_U_Float_Point *translation_vec)
{

    //dist_counter is number of distance that is tested. It is represented with a column of the matrix.
    
    IVP_ASSERT(!core->physical_unmoveable);
        
    for (int col = 0;col < fr_info->friction_springs.n_elems; col++){
	IVP_Contact_Point *distance = fr_info->friction_springs.element_at(col);
	int line_num = distance->tmp_contact_info->index_in_fs;
	if (line_num < 0) continue;
	
	if(line_num < current_contact_point_index ) {
	    //continue; //my distance is not used in complex or symmetric
	}	
	int which_obj = 1; //a distance is between two objects, which one is mine ? - 0 first, 1 second
	IVP_DOUBLE factor;
	
	IVP_Impact_Solver_Long_Term *info = contact_info_vector.element_at(line_num);

	if(info->contact_core[0]==core){
	    which_obj=0;
	    factor = -1.0f;
	} else {
	    factor = +1.0f;
	    which_obj=1;
	}
	if ( info->contact_core[which_obj]){
	    IVP_DOUBLE speed_change_scalar = factor * get_closing_speed_core_i( info, which_obj, rotation_vec, translation_vec);
	    dist_change_mat.matrix_values[line_num * dist_change_mat.aligned_row_len + current_contact_point_index] += speed_change_scalar;
	    //if ( line_num != current_contact_point_index){
	    //dist_change_mat.matrix_values[current_contact_point_index * dist_change_mat.columns + line_num] += speed_change_scalar * ( ivp_rand() * 0.001f + 1.0f);
	    //}
	}
    }
}


// returns number of active mindists (that had in last PSI push_values different to 0.0f)
// array 'active_is_at_pos' is filled up to position 'active_is_at_pos'-1
// main task is to fill 'dist_change_mat'
int IVP_Friction_Solver::calc_solver_PSI(IVP_Friction_System *fs,int *active_is_at_pos){

    IVP_IF((l_environment->debug_information->debug_friction)||0)    {
	global_friction_counter++;
	printf("\ndoing_friction_calc %d\n",global_friction_counter);
    }
    {
	//delete distance change matrix
	int mat_width=(dist_change_mat.aligned_row_len * dist_change_mat.columns);
	for(int d = mat_width-1;d >= 0;d --)    {
	    dist_change_mat.matrix_values[d] = 0.0f; 
	}
    }

    int active_dists_so_far=0;
    // new super fast version
    for (int i = 0; i< contact_info_vector.len(); i++){
	IVP_Impact_Solver_Long_Term *info = contact_info_vector.element_at(i);
	// get closing speed
	IVP_DOUBLE dist_velocity = info->get_closing_speed();
	    
	IVP_DOUBLE distance_off = ivp_mindist_settings.friction_dist - info->friction.dist_len;  // distance to my optimal position
	dist_change_mat.desired_vector[i] = calc_desired_gap_speed(dist_velocity,distance_off,1.0f);

	// mark contact point as active if positive pressure
	{
	    //printf("has_neg_since %d\n",my_dist->has_negative_pull_since);
	    if(info->friction.has_negative_pull_since != 0) {
		*active_is_at_pos = i;
		active_is_at_pos++;
		active_dists_so_far++;
	    }
	}
	    

	// calc matrix
	if ( info->contact_core[0]){
	    IVP_U_Float_Point rotation_vec,translation_vec;
	    this->test_push_core_i( info, 0, &rotation_vec, &translation_vec, -1.0f);
	    calc_distance_matrix_column(i,info->contact_core[0],info->friction.friction_infos[0],&rotation_vec,&translation_vec);
	}
	    
	if ( info->contact_core[1]){
	    IVP_U_Float_Point rotation_vec,translation_vec;
	    this->test_push_core_i( info, 1, &rotation_vec, &translation_vec, +1.0f);
	    calc_distance_matrix_column(i,info->contact_core[1],info->friction.friction_infos[1],	&rotation_vec,&translation_vec);
	}

    }
    int k = 0;
    for ( IVP_Contact_Point *my_dist = fs->get_first_friction_dist();
	  my_dist;
	  k++, my_dist = fs->get_next_friction_dist(my_dist)){
	if (my_dist->get_lt()->index_in_fs <0) continue;
	if (my_dist->get_lt()->index_in_fs != k){
	    CORE;
	}
    }
    
    return active_dists_so_far;
}


void debug_out_friction_system(IVP_Friction_System *fr_sys)
{
    IVP_Contact_Point *my_dist;
    my_dist=fr_sys->get_first_friction_dist();
    IVP_U_Point world_p;
    IVP_U_Float_Point obj_p;
    int cou=0;
    while(my_dist)
    {
	IVP_Synapse_Friction *syn0 = my_dist->get_synapse(0);
	IVP_Synapse_Friction *syn1 = my_dist->get_synapse(1);
	if(syn0->get_status() !=IVP_ST_POINT){
	    syn0=syn1;
	}
	CORE; // no calc_object_coords should be used
	//syn0->calc_object_coords(&obj_p);
	IVP_U_Matrix *mat;
	mat=&syn0->get_object()->physical_core->m_world_f_core_last_psi;
	mat->vmult4(&obj_p,&world_p);

	IVP_U_Float_Point world_push_debug;
	world_push_debug.set(0.0f,-1.0f,0.0f);
	world_push_debug.normize();
	world_push_debug.mult(10.0f);

	IVP_IF(0) {
	    char *out_text=p_make_string("%ddistdd%lx",cou,my_dist);
	    syn0->get_object()->get_environment()->add_draw_vector(&world_p,&world_push_debug,out_text,3);
	    delete (out_text);
	}
	
	cou++;
	my_dist=fr_sys->get_next_friction_dist(my_dist);
    }
}

void IVP_Friction_System::reorder_mindists_for_complex() {
    int not_in_complex=0;
    IVP_Contact_Point *my_frdist,*next_frdist;
    my_frdist=this->get_first_friction_dist();
    while(my_frdist) {
	next_frdist=this->get_next_friction_dist(my_frdist);

	if( 1 && ((my_frdist->get_gap_length() >= ivp_mindist_settings.max_distance_for_friction) ||
		  (my_frdist->get_lt()->friction_is_broken == IVP_TRUE) )){
    	    
	    this->delete_friction_distance(my_frdist);
	    my_frdist=next_frdist;
	    continue;
	}
	
	if(my_frdist->get_gap_length() > ivp_mindist_settings.friction_dist + ivp_mindist_settings.distance_keepers_safety) {
	    // move to the start of the list

	    IVP_Core *core0,*core1;
	    core0=my_frdist->get_synapse(0)->get_object()->friction_core;
	    core1=my_frdist->get_synapse(1)->get_object()->friction_core;
	
	    IVP_BOOL allow0=core0->fast_piling_allowed();
	    IVP_BOOL allow1=core1->fast_piling_allowed();
	    if(( (allow0) & (allow1))|0 ) {
	        //not_in_complex++;
		//printf("want_to_remove %lx\n",(long)my_frdist);
		this->remove_dist_from_system( my_frdist );
		this->add_dist_to_system( my_frdist );		
	    }
	}
	
	my_frdist=next_frdist;
    }
    this->complex_not_necessary_number=not_in_complex;
    //printf("not_necessary_nr %ld\n",not_in_complex);
}


// first_frd and second_frd are neighbours in queue, first is before second, exchange them
void IVP_Friction_System::exchange_friction_dists(IVP_Contact_Point *first_frd,IVP_Contact_Point *second_frd)
{
    if(this->first_friction_dist==first_frd) {
	this->first_friction_dist=second_frd;
    }
    
    {
	if(first_frd->prev_dist_in_friction) {
	    first_frd->prev_dist_in_friction->next_dist_in_friction=second_frd;
	}
	if(second_frd->next_dist_in_friction) {
	    second_frd->next_dist_in_friction->prev_dist_in_friction=first_frd;
	}   
    }
    {
	first_frd->next_dist_in_friction=second_frd->next_dist_in_friction;
	second_frd->prev_dist_in_friction=first_frd->prev_dist_in_friction;
	first_frd->prev_dist_in_friction=second_frd;
	second_frd->next_dist_in_friction=first_frd;
    }
}

void IVP_Friction_System::ivp_debug_fs_pointers()
{
    IVP_IF(1) {
    printf("%lx  ",(long)first_friction_dist&0x0000ffff);
    for(IVP_Contact_Point *fr_d=get_first_friction_dist();fr_d;fr_d=get_next_friction_dist(fr_d))
    {
	printf("%lx %lx %d %lx  ",(long)fr_d->prev_dist_in_friction&0x0000ffff,(long)fr_d&0x0000ffff,fr_d->has_negative_pull_since,(long)fr_d->next_dist_in_friction&0x0000ffff);
    }
    printf("\n");
    }
}

//sort dists, dists beeing active for longest nr of PSIs are at start of queue
void IVP_Friction_System::bubble_sort_dists_importance()
{
    IVP_Contact_Point *first_dist=this->get_first_friction_dist();
    IVP_Contact_Point *second_dist;
    if( !first_dist ) {
	return;
    }
    while(1) {
	second_dist=get_next_friction_dist(first_dist);
	if( !second_dist ) {
	    break;
	}
	if( first_dist->has_negative_pull_since > second_dist->has_negative_pull_since ) {
	    exchange_friction_dists(first_dist,second_dist);
	    IVP_Contact_Point *test_dist;
	    while( (test_dist=get_prev_friction_dist(second_dist)) && ( test_dist->has_negative_pull_since > second_dist->has_negative_pull_since ) ) {
		exchange_friction_dists(test_dist,second_dist);
	    }
	    second_dist=first_dist;
	}
	first_dist=second_dist;
    }
    IVP_IF(1) {
	for( first_dist=get_first_friction_dist(); first_dist; first_dist=get_next_friction_dist(first_dist) ) {
	    second_dist=get_next_friction_dist(first_dist);
	    if(second_dist) {
		IVP_ASSERT( !(first_dist->has_negative_pull_since > second_dist->has_negative_pull_since ) );
	    }
	}
    }
}

#if 0
//sort dists, dists beeing active for longest nr of PSIs are at end of queue
void IVP_Friction_System::bubble_sort_dists_importance()
{
    int did_exchange=1;
    //printf("start_bubble\n");
    while(did_exchange) {
	did_exchange=0;
	IVP_Contact_Point *moving_dist=this->get_first_friction_dist();
	if(!moving_dist) {
	    return; //degenerated system (without any mindist)
	}
	IVP_Contact_Point *test_dist;
	//printf("tlp_for %lx  ",(long)moving_dist&0x0000ffff);
	//ivp_debug_fs_pointers();
	while((test_dist=this->get_next_friction_dist(moving_dist))!=NULL) {
	    if(moving_dist->has_negative_pull_since<test_dist->has_negative_pull_since) {
		this->exchange_friction_dists(moving_dist,test_dist);
		did_exchange=1;
		//printf("exchgg %lx %lx  ",(long)moving_dist&0x0000ffff,(long)test_dist&0x0000ffff);
		//ivp_debug_fs_pointers();
	    } else {
		moving_dist=test_dist;
	    }
	}
    }
    IVP_IF(1) {
	for(IVP_Contact_Point *fr_d=get_first_friction_dist();fr_d;fr_d=get_next_friction_dist(fr_d))
	{
	    IVP_Contact_Point *test_d=get_next_friction_dist(fr_d);
	    if(test_d) {
		if(fr_d->has_negative_pull_since<test_d->has_negative_pull_since) {
		    printf("bubble_sort_error\n");
		    CORE;
		}
	    }
	}
    }
    //printf("after_bs  ");
    //ivp_debug_fs_pointers();
    IVP_IF(0)
    {
        printf("sort_dists ");
	for(IVP_Contact_Point *fr_d=get_first_friction_dist();fr_d;fr_d=get_next_friction_dist(fr_d))
	{
	    printf("%d ",fr_d->has_negative_pull_since);
	}
	printf("\n");
    }
}
#endif

IVP_BOOL IVP_Friction_System::core_is_terminal_in_fs(IVP_Core *test_core) {
    int total_number_of_partners=0;
    IVP_ASSERT(test_core->physical_unmoveable==IVP_FALSE);
    for (int i = fr_pairs_of_objs.len()-1; i>=0; i--){
	IVP_Friction_Core_Pair *fcp = fr_pairs_of_objs.element_at(i);
	if( (fcp->objs[0]==test_core) || (fcp->objs[1]==test_core) ) {
	    int unmov=fcp->objs[0]->physical_unmoveable | fcp->objs[1]->physical_unmoveable;
	    if(!unmov) {
		total_number_of_partners++;
	    }
	}
    }
    if(total_number_of_partners>1) return IVP_FALSE;
    //printf("found_terminal %lx\n",(long)test_core&0x0000ffff);
    return IVP_TRUE;
}

//fallback for oversized matrizes:
//for terminal objects: turn on fast piling technique and hope they will be pushed away
//for nonterminal objects: turn on fast piling but delete whole energy (with delayed pushes the energy would be accumulated)
void IVP_Friction_System::static_fr_oversized_matrix_panic() {
    IVP_Core *my_core;
    int i;
    for(i=moveable_cores_of_friction_system.len()-1;i>=0;i--) {
	my_core=moveable_cores_of_friction_system.element_at(i);
	my_core->fast_piling_allowed_flag=IVP_TRUE;
	if(this->core_is_terminal_in_fs(my_core)) {
	    //hope the object will be pushed away some time
	} else {
	    //remove all energy from object
	    my_core->speed.set_to_zero();
	    my_core->rot_speed.set_to_zero();
	}
    }
}

// returns 1 if friction_system is no longer used
void IVP_Friction_System::do_friction_system(const IVP_Event_Sim *es_in)
{    
    IVP_U_Memory *my_mem = l_environment->short_term_mem;
    my_mem->start_memory_transaction();
    
    friction_global_counter++;
    IVP_IF(l_environment->debug_information->debug_friction)    {
	printf("doing_friction_nr %d\n",friction_global_counter);
	if(friction_global_counter==14)	{
	    printf("losgehts\n");
        }
	printf("friction_system_simulating %d distances\n",this->friction_dist_number);
    }    

    this->bubble_sort_dists_importance();   // to to put heavy pushed mindists ontop of the list
    this->reorder_mindists_for_complex();   // get rid of mindist already heaving a big gap @+@ ->check speed after do_pushes_distance_keepers also

#if defined(IVP_DISTANCE_KEEPERS) && !defined(IVP_PANELTY_SOLVER)
    this->do_pushes_distance_keepers(es_in);  // do local constraint pushes
#endif

    if((this->friction_dist_number - this->complex_not_necessary_number) > IVP_MAX_FRICTION_MATRIX) {
	this->static_fr_oversized_matrix_panic();
	my_mem->end_memory_transaction();
	return;    
    }
    
    IVP_Friction_Solver fr_solver(this,es_in);

    fr_solver.setup_coords_mindists(this);
    int *original_pos_of_active=(int*)my_mem->get_mem(friction_dist_number*sizeof(int));

    while(IVP_TRUE)   {
	int num_actives = fr_solver.calc_solver_PSI(this,original_pos_of_active);

	IVP_IF(l_environment->debug_information->disable_friction) {
	    break;
	}

#ifdef IVP_PANELTY_SOLVER
       	fr_solver.do_penalty_method(this);
#endif
	
	IVP_IF(1==0)   {
	  for(int j=0;j<num_actives;j++)		{
	    IVP_ASSERT(original_pos_of_active[j]>=0);
	    IVP_ASSERT(original_pos_of_active[j]<friction_dist_number);
	  }
	}
	
	fr_solver.solve_linear_equation_and_push(this,original_pos_of_active,num_actives,my_mem);
	break;
    }
    my_mem->end_memory_transaction();
}


IVP_DOUBLE IVP_Friction_System::kinetic_energy_of_hole_frs()
{
    IVP_DOUBLE sum=0.0f;
    for (int i = cores_of_friction_system.len()-1; i>=0; i--){
	IVP_Core *my_obj = cores_of_friction_system.element_at(i);
	IVP_U_Float_Point speed_vec,rot_vec;
	speed_vec.add(&my_obj->speed,&my_obj->speed_change);
	rot_vec.add(&my_obj->rot_speed,&my_obj->rot_speed_change);
	sum+=my_obj->get_energy_on_test(&speed_vec,&rot_vec);
    }
    return sum;
}

void IVP_Friction_System::confirm_complex_pushes() {
    for (int k = moveable_cores_of_friction_system.len()-1; k>=0; k--){
	IVP_Core *my_core = moveable_cores_of_friction_system.element_at(k);
	my_core->commit_all_async_pushes(); //not too nice: no clear_all_async_pushes done before
    }
}

void IVP_Friction_System::undo_complex_pushes() {
    for (int k = moveable_cores_of_friction_system.len()-1; k>=0; k--){
	IVP_Core *my_core = moveable_cores_of_friction_system.element_at(k);
	my_core->abort_all_async_pushes();
    }
}

IVP_DOUBLE IVP_Friction_System::get_max_energy_gain() {
    IVP_DOUBLE sum_gain=0.0f;
    for (int k = cores_of_friction_system.len()-1; k>=0; k--){
	IVP_Core *my_core = cores_of_friction_system.element_at(k);
	if(my_core->physical_unmoveable==IVP_FALSE) {
	    IVP_DOUBLE force_gravity=my_core->get_mass()*9.81f; //@@TL #+# oh oh 
	    // allow lift of 10cm
	    sum_gain+=force_gravity*0.1f;
	}
    }
    return sum_gain;
}
