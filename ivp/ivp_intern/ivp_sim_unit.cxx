// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#if !defined(WIN32) && !defined(PSXII) && !defined(GEKKO)
#	include <alloca.h>
#endif

#ifndef WIN32
#	pragma implementation "ivp_sim_unit.hxx"
#endif


#include <ivp_controller.hxx>
#include <ivp_mindist_intern.hxx>
#include <ivp_friction.hxx>
#include <ivp_core_macros.hxx>
#include <ivp_sim_unit.hxx>
#include <ivu_memory.hxx>
#include <ivp_calc_next_psi_solver.hxx>
#include <ivp_performancecounter.hxx>
#include <ivp_time.hxx>

IVP_U_Vector<IVP_Core> IVP_Controller_Independent::empty_list;

#ifdef WIN32
extern long p_get_time();
#endif

//the list of cores is valid, calculate the rest
void IVP_Simulation_Unit::sim_unit_calc_redundants() {
    for (int i = sim_unit_cores.len()-1; i>=0; i--){
	IVP_Core *my_core = sim_unit_cores.element_at(i);

	for (int k = my_core->controllers_of_core.len()-1; k>=0;k--){
	    IVP_Controller *my_controller = my_core->controllers_of_core.element_at(k);
	    if( this->controller_is_known_to_sim_unit( my_controller ) == IVP_FALSE ) {
	        this->add_controller_unit_sim( my_controller );
	    }
	    this->add_controlled_core_for_controller( my_controller, my_core );
        }
    }
    sim_unit_sort_controllers();
}

IVP_BOOL IVP_Simulation_Unit::controller_is_known_to_sim_unit( IVP_Controller *my_controller ) {
    int i;
    for( i=controller_cores.len()-1; i>=0 ; i-- ) {
        if( controller_cores.element_at(i)->l_controller == my_controller ) {
	    return IVP_TRUE;
	}
    }
    return IVP_FALSE;
}

// controller exists; add a core for this controller
void IVP_Simulation_Unit::add_controlled_core_for_controller( IVP_Controller *cntrl, IVP_Core *my_core ) {
    int i;
    for( i=controller_cores.len()-1; i>=0 ; i-- ) {
        if( controller_cores.element_at(i)->l_controller == cntrl ) {
	    break;
	}
    }
    IVP_ASSERT(i>=0);
    //IVP_ASSERT( controller_cores.element_at(i)->l_controller == cntrl );
    controller_cores.element_at(i)->cores_controlled_by.add( my_core );
}

void IVP_Simulation_Unit::add_controller_unit_sim( IVP_Controller *new_cntrl ) {
  //sim_unit_controllers.add( new_cntrl );
    IVP_Sim_Unit_Controller_Core_List *core_list=new IVP_Sim_Unit_Controller_Core_List();
    core_list->l_controller=new_cntrl;
    controller_cores.add(core_list);
}


void IVP_Simulation_Unit::split_sim_unit(IVP_Core *split_father) {
    IVP_ASSERT(split_father);
    IVP_Core *my_core;
    IVP_Core *next_split_father=NULL;
    IVP_BOOL next_split_necessary=IVP_FALSE;
    IVP_Simulation_Unit *split_new_unit=new IVP_Simulation_Unit();
    split_new_unit->sim_unit_movement_type=IVP_MT_MOVING;
    IVP_Sim_Units_Manager *s_man;
    s_man=    split_father->environment->get_sim_units_manager();
    s_man->add_sim_unit_to_manager( split_new_unit );
    
    //faster: operate directly on list, build two new lists -> only linear not quadratic
    int i;
    for(i=0;i<sim_unit_cores.len();i++) {
        my_core=sim_unit_cores.element_at(i);
        IVP_Core *test_core=my_core->union_find_get_father();
	if( test_core != split_father ) {
	  if( next_split_father != NULL ) {
	      if( test_core!=next_split_father ) {
		  next_split_necessary=IVP_TRUE;
	      }
	  } else {
	      next_split_father=test_core;
	  }
	} else {
	    sim_unit_cores.remove_at(i);
	    i--; //size of list is reduced
	    split_new_unit->sim_unit_cores.add(my_core);
	    my_core->sim_unit_of_core=split_new_unit;
	}
    }

    split_new_unit->sim_unit_calc_redundants();
#ifdef DEBUG
    IVP_IF(1) {
      split_new_unit->sim_unit_debug_consistency();
    }
#endif    
    if(next_split_necessary==IVP_TRUE) {
        this->split_sim_unit(next_split_father);
    }
}

void IVP_Simulation_Unit::perform_test_and_split() {
    IVP_Core *father=sim_unit_union_find_test();
    if(father) {
        this->clean_sim_unit();
        split_sim_unit(father);
	this->sim_unit_calc_redundants();
#ifdef DEBUG
	this->sim_unit_debug_consistency();
#endif	
    }
}

IVP_Core *IVP_Simulation_Unit::sim_unit_union_find_test()
{
    IVP_Core *my_core;
    for (int i = sim_unit_cores.len()-1; i>=0;i--){
	my_core = sim_unit_cores.element_at(i);
	my_core->tmp.union_find_father=NULL;
    }

    for(int  i2=controller_cores.len()-1; i2>=0; i2-- ) {
	IVP_Controller *my_controller=controller_cores.element_at(i2)->l_controller;
	IVP_U_Vector<IVP_Core> *list_of_cores;
	list_of_cores=my_controller->get_associated_controlled_cores();
	int j;
	j=list_of_cores->len()-1;
	if(j>=0) {
	    IVP_Core *father_core=list_of_cores->element_at(0)->union_find_get_father();
	    for(;j>=0;j--) {
	        IVP_Core *test_core=list_of_cores->element_at(j);
		IVP_Core *test_father=test_core->union_find_get_father();
		if(father_core!=test_father) {
		    test_father->tmp.union_find_father=father_core;
		}
	    }
	}
    }
#if 0    
    for(my_core=this->get_first_sim_unit_core();my_core;my_core=this->get_next_sim_unit_core())
    {
        IVP_Core *father_core=my_core->union_find_get_father();
        for(my_controller=my_core->get_first_core_controller();my_controller;my_controller=my_core->get_next_core_controller()) {
	    IVP_U_Vector<IVP_Core> *controlled_cores;
	    controlled_cores=my_controller->get_associated_controlled_cores(my_core);
	    for(i=controlled_cores->len()-1;i>=0;i--) {
	        IVP_Core *test_core=controlled_cores->element_at(i);
		IVP_Core *test_father=test_core->union_find_get_father();
		if(father_core!=test_father) {
		    test_core->tmp.union_find_father=father_core;
		}
	    }
        }
    }
#endif
    
    IVP_Core *first_father=this->sim_unit_cores.element_at(0)->union_find_get_father();
    // find representative obj for second system (must not be a fixed obj)

    {
	int j;
	for (j = 0; j < sim_unit_cores.len(); j++){
	    IVP_Core *obj = sim_unit_cores.element_at(j);
	    IVP_Core *test_father=obj->union_find_get_father();
	    if(test_father!=first_father)	{
		return test_father;
	    }
	}
    }
    return NULL;
}

//clear redundant part
void IVP_Simulation_Unit::clean_sim_unit() {
    int i;
    for(i=controller_cores.len()-1;i>=0;i--) {
        IVP_Sim_Unit_Controller_Core_List *c_info=controller_cores.element_at(i);
	P_DELETE( c_info );
    }
    //sim_unit_controllers.clear();
    controller_cores.clear();
}

void IVP_Simulation_Unit::throw_cores_into_my_sim_unit(IVP_Simulation_Unit *second_unit) {
    IVP_Environment *env=NULL;
    int i;
    for ( i=0; i<second_unit->sim_unit_cores.len(); i++){
	    IVP_Core *my_core = second_unit->sim_unit_cores.element_at(i);
        this->add_sim_unit_core( my_core );
	    env=my_core->environment;
	    my_core->sim_unit_of_core=this;
    }

    env->get_sim_units_manager()->rem_sim_unit_from_manager( second_unit );
}

// second_unit is destroyed
void IVP_Simulation_Unit::fusion_simulation_unities(IVP_Simulation_Unit *second_unit) {
    this->clean_sim_unit();

    throw_cores_into_my_sim_unit(second_unit);

    this->sim_unit_calc_redundants();
}

int IVP_Simulation_Unit::get_pos_of_controller( IVP_Controller *contr ) {
  int i;
  for(i=controller_cores.len()-1;i>=0;i--) {
    if(controller_cores.element_at(i)->l_controller == contr) {
      break;
    }
  }
  return i;
}

void IVP_Simulation_Unit::sim_unit_remove_core( IVP_Core *del_core ) {
    rem_sim_unit_core( del_core );
    int i;
    for (i = del_core->controllers_of_core.len()-1; i>=0;i--){
	IVP_Controller *my_controller= del_core->controllers_of_core.element_at(i);
	int pos=get_pos_of_controller(my_controller);
	IVP_ASSERT(pos>=0);
	IVP_Sim_Unit_Controller_Core_List *c_info=controller_cores.element_at(pos);
	c_info->cores_controlled_by.remove(del_core);
	if( c_info->cores_controlled_by.len() == 0 ) {
	    rem_sim_unit_controller( my_controller );
	}
    }
    if(sim_unit_cores.len()==0) {
        del_core->environment->get_sim_units_manager()->rem_sim_unit_from_manager(this);
	P_DELETE_THIS(this);
    }
}

IVP_Simulation_Unit::~IVP_Simulation_Unit() {
    this->clean_sim_unit();
    ;//printf("delete_simu %lx\n",(long)this&0x0000ffff);
}

IVP_Simulation_Unit::IVP_Simulation_Unit() {
    //printf("create_simu %lx\n",(long)this&0x0000ffff);
    union_find_needed_for_sim_unit = IVP_FALSE;
    sim_unit_movement_type = IVP_MT_NOT_SIM;
    sim_unit_just_slowed_down = IVP_FALSE;
    sim_unit_has_fast_objects = IVP_FALSE;
}

void IVP_Simulation_Unit::rem_sim_unit_controller( IVP_Controller *rem_controller ) {
    int pos=get_pos_of_controller( rem_controller );
    IVP_ASSERT(pos>=0);

    IVP_Sim_Unit_Controller_Core_List *c_info=controller_cores.element_at(pos);
    P_DELETE(c_info);
    controller_cores.remove_at(pos);
    //sim_unit_controllers.delete_at(pos);
}

void IVP_Simulation_Unit::add_sim_unit_core( IVP_Core *add_core ) {
    sim_unit_cores.add( add_core );
}

void IVP_Simulation_Unit::rem_sim_unit_core( IVP_Core *del_core ) {
    sim_unit_cores.remove( del_core );
}

IVP_BOOL IVP_Simulation_Unit::sim_unit_core_exists(IVP_Core *core) {
    int i;
    for(i=sim_unit_cores.len()-1;i>=0;i--) {
        if(sim_unit_cores.element_at(i) == core) {
	    return IVP_TRUE;
	}
    }
    return IVP_FALSE;
}

#ifdef DEBUG
void IVP_Simulation_Unit::sim_unit_debug_out() {
    printf("sim_unit_cores:\n");
    int i;
    for(i=sim_unit_cores.len()-1;i>=0;i--) {
        IVP_Core *my_core=sim_unit_cores.element_at(i);
        printf("%lx: ",(long)my_core);
	IVP_Controller *my_cnt;
	int j;
	for(j=my_core->controllers_of_core.len()-1;j>=0;j--) {
	    my_cnt=my_core->controllers_of_core.element_at(j);
	    printf("%lx ",(long)my_cnt);
	}
	printf("\n");
    }
    
    for(i=controller_cores.len()-1;i>=0;i--) {
        printf("    controlr %lx: ",(long)controller_cores.element_at(i)->l_controller);
	int j;
	IVP_Sim_Unit_Controller_Core_List *c_info=controller_cores.element_at(i);
	for(j=c_info->cores_controlled_by.len()-1;j>=0;j--) {
	    printf("%lx ",(long)c_info->cores_controlled_by.element_at(j));
	}
	printf("\n");
    }    
}
#endif

#ifdef DEBUG
void IVP_Simulation_Unit::sim_unit_debug_consistency() {
    //every core has to occur only once
    int i,j;
    for(i=sim_unit_cores.len()-1;i>=0;i--) {
        IVP_Core *search_core=sim_unit_cores.element_at(i);
	for(j=i-1;j>=0;j--) {
	    IVP_ASSERT( sim_unit_cores.element_at(j) != search_core );
	}
	IVP_ASSERT( !search_core->physical_unmoveable );
	IVP_ASSERT( search_core->sim_unit_of_core==this );
    }
    //every controller has to occur only once
    for(i=controller_cores.len()-1;i>=0;i--) {
        IVP_Controller *search_controller=controller_cores.element_at(i)->l_controller;
	for(j=i-1;j>=0;j--) {
	    IVP_ASSERT( controller_cores.element_at(j)->l_controller != search_controller );
	}
    }    
    
    //sim_unit_controllers and controller_cores are associated
    //IVP_ASSERT( sim_unit_controllers.len() == controller_cores.len() );
    for(i=controller_cores.len()-1;i>=0;i--) {
        IVP_Sim_Unit_Controller_Core_List *c_info=controller_cores.element_at(i);
	//IVP_ASSERT( c_info->l_controller == sim_unit_controllers.element_at(i) );
	//every core in controller list must exist
	for(j=c_info->cores_controlled_by.len()-1;j>=0;j--) {
	    IVP_Core *test_core=c_info->cores_controlled_by.element_at(j);
	    IVP_ASSERT( sim_unit_core_exists(test_core)==IVP_TRUE );
	}
    }
    
    //all cores must have all their controllers in controller list
    for (int i1 = sim_unit_cores.len()-1; i1>=0; i1--){
	IVP_Core *my_core = sim_unit_cores.element_at(i1);
	for (int i2 = my_core->controllers_of_core.len()-1;i2>=0; i2--){
	    IVP_Controller *my_controller = my_core->controllers_of_core.element_at(i2);

	    IVP_ASSERT( controller_is_known_to_sim_unit( my_controller )==IVP_TRUE );

	    //for every core its via controllers associated cores must exist
	    IVP_U_Vector<IVP_Core> *core_list = my_controller->get_associated_controlled_cores();
	    for(int i4=core_list->len()-1;i4>=0;i4--) {
		IVP_Core *test_core=core_list->element_at(i4);
		IVP_ASSERT( !test_core->physical_unmoveable );
		IVP_ASSERT( sim_unit_core_exists( test_core ) == IVP_TRUE );
	    }

	    //every core/controller pair must be represented in c_info
	    int pos=get_pos_of_controller(my_controller);
	    IVP_Sim_Unit_Controller_Core_List *c_info=controller_cores.element_at(pos);
	    int found=0;
	    for(int i5=c_info->cores_controlled_by.len()-1;i5>=0;i5--) {
		IVP_Core *tt_core=c_info->cores_controlled_by.element_at(i5);
	        if(tt_core == my_core) {
		    found=1;
	        }
		//this controller must be found in my cores controller list
		int find_contr=0;
		for(int k=0;k<tt_core->controllers_of_core.len();k++) {
		    if( tt_core->controllers_of_core.element_at(k) == my_controller ) {
		        find_contr=1;
		    }
		}
		IVP_ASSERT(find_contr==1);
	    }
	    IVP_ASSERT(found==1);
	}
    }
}
#endif

void IVP_Controller_Manager::ensure_core_in_simulation(IVP_Core *core) {
    core->ensure_core_in_simulation_delayed();
}

//only for Controllers that have all cores computed at the same time
void IVP_Controller_Manager::remove_controller_from_environment( IVP_Controller_Dependent *cntrl, IVP_BOOL silently ) {
    IVP_U_Vector<IVP_Core> *controlled_cores=cntrl->get_associated_controlled_cores();

    IVP_Simulation_Unit *reference_unit=NULL;
    int i;
    for(i=controlled_cores->len()-1;i>=0;i--) {
        IVP_Core *my_core=controlled_cores->element_at(i);
	my_core->rem_core_controller(cntrl);
	reference_unit=my_core->sim_unit_of_core;
    }
    if(reference_unit) {
      reference_unit->union_find_needed_for_sim_unit=IVP_TRUE;
      if(silently==IVP_FALSE) {
        reference_unit->sim_unit_ensure_in_simulation();
      }
    }
}

void IVP_Controller_Manager::ensure_controller_in_simulation(IVP_Controller_Dependent *cntrl) {
    IVP_U_Vector<IVP_Core> *controlled_cores=cntrl->get_associated_controlled_cores();
    IVP_ASSERT( controlled_cores->len() > 0);
    if(controlled_cores->len() > 0) { // #+# is this to avoid an assert ???
        controlled_cores->element_at(0)->sim_unit_of_core->sim_unit_ensure_in_simulation();
    }
}

void IVP_Controller_Manager::add_controller_to_core(IVP_Controller_Independent *cntrl, IVP_Core *core){
	core->add_core_controller(cntrl);
}

void IVP_Controller_Manager::remove_controller_from_core(IVP_Controller_Independent *cntrl, IVP_Core *core){
	core->rem_core_controller(cntrl);
}

//only for Controllers that have all cores computed at the same time
void IVP_Controller_Manager::announce_controller_to_environment( IVP_Controller_Dependent *cntrl ) {
    IVP_U_Vector<IVP_Core> *controlled_cores=cntrl->get_associated_controlled_cores();
  
    IVP_Simulation_Unit *reference_unit=NULL;
    IVP_BOOL did_fusion=IVP_FALSE;
    IVP_Movement_Type mtype=IVP_MT_NOT_SIM;
    
    int i;
    for(i=controlled_cores->len()-1;i>=0;i--) {
        IVP_Core *test_core=controlled_cores->element_at(i);
	if(!test_core->physical_unmoveable) {	    
	    
	    mtype=(IVP_Movement_Type)((int)mtype & (int)test_core->movement_state);
	    IVP_Simulation_Unit *test_sim_unit=test_core->sim_unit_of_core;
	    if(reference_unit!=NULL) {
	        if(test_sim_unit!=reference_unit) {
		    reference_unit->throw_cores_into_my_sim_unit(test_sim_unit); //
		    P_DELETE(test_sim_unit);
		    did_fusion=IVP_TRUE;
		}
	    } else {
	        reference_unit=test_sim_unit;
	    }
	    
	    test_core->add_core_controller(cntrl);
	    
	}
    }

    if(did_fusion==IVP_TRUE) {
        reference_unit->clean_sim_unit();
	reference_unit->sim_unit_calc_redundants();
    }
    
    if(mtype<IVP_MT_NOT_SIM) {
        reference_unit->sim_unit_revive_for_simulation(l_environment); //ensure in simulation is not enough (cannot handle mixture of simulated and not simulated objects)
    }
}

void IVP_Core::rem_core_controller( IVP_Controller *rem_cntrl ) {
    controllers_of_core.remove(rem_cntrl);
    this->sim_unit_of_core->remove_controller_of_core(this,rem_cntrl);
}

void IVP_Core::add_core_controller( IVP_Controller *add_cntrl ) {
    controllers_of_core.add(add_cntrl);
    sim_unit_of_core->add_controller_of_core(this,add_cntrl);
}

// e.g. a core enters a water pool
void IVP_Simulation_Unit::add_controller_of_core( IVP_Core *my_core, IVP_Controller *cntrl ) {
    if( controller_is_known_to_sim_unit( cntrl ) ) {
        ;
    } else {
        add_controller_unit_sim( cntrl );
    }
    add_controlled_core_for_controller( cntrl, my_core );
    this->sim_unit_sort_controllers();
}

// e.g. a core leaves a water pool
void IVP_Simulation_Unit::remove_controller_of_core( IVP_Core *my_core, IVP_Controller *cntrl ) {
    int pos=get_pos_of_controller(cntrl);
    IVP_ASSERT(pos>=0);
    IVP_Sim_Unit_Controller_Core_List *c_info=controller_cores.element_at(pos);
    c_info->cores_controlled_by.remove(my_core);
    if( c_info->cores_controlled_by.len() <= 0 ) {
        P_DELETE(c_info);
	controller_cores.remove_at( pos ); //this controller no longer controlls any cores
    }
    //sim_unit_controllers.delete_at( pos );
}

void IVP_Simulation_Unit::sim_unit_sort_controllers() {
    int contr_num=controller_cores.len();

    int first=0;
    while(1) {
	int second=first+1;
	if( second >= contr_num ) {
	    break;
	}
	IVP_CONTROLLER_PRIORITY second_prio = controller_cores.element_at(second)->l_controller->get_controller_priority();
	if( controller_cores.element_at(first)->l_controller->get_controller_priority() > second_prio ) {
	    sim_unit_exchange_controllers(first,second);
	    int test_pos=first;
	    while( (test_pos>0) && ( controller_cores.element_at(test_pos-1)->l_controller->get_controller_priority() > second_prio ) ) {//controller_cores.element_at(test_pos)->l_controller->get_controller_priority() ) ) {
		sim_unit_exchange_controllers(test_pos-1,test_pos);
		test_pos--;
	    }
	}
	first=second;
    }
    IVP_IF(1) {
	for( int i=0; i<contr_num-1; i++ ) {
		IVP_ASSERT( !( controller_cores.element_at(i)->l_controller->get_controller_priority() > controller_cores.element_at(i+1)->l_controller->get_controller_priority() ) );
	}
    }    
}

void IVP_Simulation_Unit::sim_unit_exchange_controllers(int first,int second) {
    IVP_ASSERT(first>=0);
    IVP_ASSERT(second>=0);
    IVP_ASSERT(first<controller_cores.len());
    IVP_ASSERT(second<controller_cores.len());
    //sim_unit_controllers.exchange_vector_elems(first,second);
    controller_cores.swap_elems(first,second);
}

IVP_Sim_Units_Manager::IVP_Sim_Units_Manager(IVP_Environment *env) {
    l_environment=env;
    sim_units_slots[0]=NULL;
    still_slot=NULL;
    nb = IVP_Time(9.73f);
    bt = IVP_Time(0.3f);
}

void IVP_Sim_Units_Manager::add_unit_to_slot(IVP_Simulation_Unit *sim_u,IVP_Simulation_Unit **slot) {
    IVP_Simulation_Unit *s_u=*slot;
    sim_u->next_sim_unit=s_u;
    if(s_u) {
        s_u->prev_sim_unit=sim_u;
    }
    sim_u->prev_sim_unit=NULL;
    *slot=sim_u;    
}

void IVP_Sim_Units_Manager::add_sim_unit_to_manager(IVP_Simulation_Unit *sim_u) {
    IVP_Simulation_Unit **slot;
    if( sim_u->get_unit_movement_type() < IVP_MT_NOT_SIM ) {
        slot=&sim_units_slots[0];
    } else {
        slot=&still_slot;
    }
    add_unit_to_slot(sim_u,slot);
}

void IVP_Sim_Units_Manager::rem_unit_from_slot(IVP_Simulation_Unit *sim_u,IVP_Simulation_Unit **slot)
{
    IVP_Simulation_Unit *prev_u,*next_u;
    prev_u=sim_u->prev_sim_unit;
    next_u=sim_u->next_sim_unit;

    if(prev_u) {
        prev_u->next_sim_unit=next_u;
    } else {
        *slot=next_u;
    }
    if(next_u) {
        next_u->prev_sim_unit=prev_u;
    }
}

void IVP_Sim_Units_Manager::rem_sim_unit_from_manager(IVP_Simulation_Unit *sim_u) {
    IVP_Simulation_Unit **slot;
    if( sim_u->get_unit_movement_type() < IVP_MT_NOT_SIM ) {
        slot=&sim_units_slots[0];
    } else {
        slot=&still_slot;
    }
    rem_unit_from_slot(sim_u,slot);
}


//during reviving of unit all cores are processed
//each core is revived and friction systems for that core are grown
//that may lead to more Simulation_units are fusioned with mine
//after a fusion is done, I have to restart the reviving of cores (Datastructure reassembly)
void IVP_Simulation_Unit::sim_unit_revive_for_simulation(IVP_Environment *env) {
revive_all_cores:    
    {
	for (int i = sim_unit_cores.len()-1; i>=0;i--){
	    IVP_Core *my_core = sim_unit_cores.element_at(i);
	    IVP_ASSERT( !my_core->physical_unmoveable );
	    if(my_core->movement_state>=IVP_MT_NOT_SIM) {
	        IVP_BOOL fusion_was_done;
	        fusion_was_done=my_core->revive_simulation_core();
		if(fusion_was_done==IVP_TRUE) {
		    goto revive_all_cores;
		}
	    }
	}
    }

    if(sim_unit_movement_type==IVP_MT_NOT_SIM) {
        env->get_sim_units_manager()->rem_sim_unit_from_manager(this);
	sim_unit_movement_type=IVP_MT_MOVING;
	env->get_sim_units_manager()->add_sim_unit_to_manager(this);
    }
}

void IVP_Simulation_Unit::sim_unit_ensure_cores_movement() {
  IVP_Core *my_core;
  int i;
  for(i=sim_unit_cores.len()-1;i>=0;i--) {
    my_core=sim_unit_cores.element_at(i);
    my_core->reset_freeze_check_values();
  }
}

void IVP_Standard_Gravity_Controller::set_standard_gravity(IVP_U_Point *gravity) {
    grav_vec.set(gravity);
}

void IVP_Standard_Gravity_Controller::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> *core_list) {
    int i;
    for(i=core_list->len()-1;i>=0;i--) {
        IVP_Core *my_core=core_list->element_at(i);

	if(!my_core->pinned) //@@CB
	{
    	    my_core->global_damp_core(es->delta_time);
	    my_core->commit_all_async_pushes();
	    my_core->speed.add_multiple( &this->grav_vec,es->delta_time );
	}
    }
}

void IVP_Simulation_Unit::sim_unit_clear_movement_check_values() {
    for (int c = sim_unit_cores.len()-1; c>=0; c--) {
	IVP_Core *core = sim_unit_cores.element_at(c);
	core->reset_freeze_check_values();
    }
}

void IVP_Simulation_Unit::do_sim_unit_union_find() {
    this->clean_sim_unit();
    this->sim_unit_calc_redundants();
    this->perform_test_and_split();
    this->union_find_needed_for_sim_unit=IVP_FALSE;
}

IVP_BOOL IVP_Simulation_Unit::sim_unit_calc_movement_state(IVP_Environment *env) {
#if !defined(IVP_DISABLE_FREEZING)
    IVP_Movement_Type whole_sys=IVP_MT_CALM;
    IVP_Time current_time=env->get_current_time();
    for (int c = sim_unit_cores.len()-1; c>=0; c--) {
	IVP_Core *core = sim_unit_cores.element_at(c);
	core->movement_state=core->calc_movement_state(current_time);
        whole_sys=(IVP_Movement_Type)((int)whole_sys & (int)core->movement_state);
    }
    if( whole_sys == IVP_MT_CALM ) {
	for (int n = sim_unit_cores.len()-1; n>=0; n--){
	    IVP_Core *my_core = sim_unit_cores.element_at(n);
	    my_core->freeze_simulation_core();	    //make callbacks for controllers??
	}
	env->get_sim_units_manager()->rem_sim_unit_from_manager(this);
	this->sim_unit_movement_type=IVP_MT_NOT_SIM;
	env->get_sim_units_manager()->add_sim_unit_to_manager(this);
	return IVP_TRUE;
    }
#endif
    return IVP_FALSE;
}

void IVP_Simulation_Unit::init_moving_core_for_psi(IVP_Core *core, const IVP_Time &c_time) {

    //IVP_PREFETCH(this->objects.elems); // now using special vector

    IVP_IF(1) {
	IVP_Friction_Info_For_Core *info=core->moveable_core_has_friction_info();
	IVP_Friction_System *fs=NULL;
	if(info) {
	  fs=info->l_friction_system;
	}
    }
  
    IVP_ASSERT(core->physical_unmoveable==IVP_FALSE);

//    IVP_ASSERT ( c_time.get_time() == 0 || IVP_Inline_Math::fabsd( core->time_of_last_psi - c_time + 1.0f / core->i_delta_time ) < 10E-4f);
 
    IVP_DOUBLE d_time = c_time - core->time_of_last_psi;
    
    core->q_world_f_core_next_psi.set_matrix(&core->m_world_f_core_last_psi);
    IVP_PREFETCH(core->objects.element_at(0),0);
    core->m_world_f_core_last_psi.vv.add_multiple( &core->pos_world_f_core_last_psi,  &core->delta_world_f_core_psis, d_time);        
}



void IVP_Simulation_Unit::simulate_single_sim_unit_psi(IVP_Event_Sim *es, IVP_U_Vector<IVP_Core> *touched_cores) {
#ifdef DEBUG
    IVP_IF(1) {
        this->sim_unit_debug_consistency();
    }
#endif
    es->sim_unit = this;
    es->environment->sim_unit_mem->start_memory_transaction();
    
    int controller_num=controller_cores.len();

    IVP_Time current_time = es->environment->get_current_time();

    int fast_moving_flag = 0;

    union {
	int fast_moving_core;
	float fast_moving_core_p;
	};  //warning: 4 Byte of memory are used to store an integer and a float value to get access to sign bit 
    
    for (int d = sim_unit_cores.len()-1; d>=1; d--){
	IVP_Core *next_core = sim_unit_cores.element_at(d-1);

	this->prefetch0_init_moving_core_for_psi(next_core);

	IVP_Core *my_core = sim_unit_cores.element_at(d);
        this->init_moving_core_for_psi(my_core, current_time);
	IVP_IF(1) {
	  for(int k=my_core->objects.len()-1;k>=0;k--) {	    IVP_ASSERT(my_core->objects.element_at(k)->get_movement_state()<IVP_MT_NOT_SIM);	  }	    
	}
	
	my_core->commit_all_async_pushes(); // @@@OS this happens very seldomly !!!!, remove !!!!this necessary as it may happen that core was temporarily_unmovable
	                                    // TL: it is also used for delayed pushes and async_pushes
	fast_moving_core_p = IVP_OBJECT_MOVING_FAST * IVP_OBJECT_MOVING_FAST - my_core->speed.quad_length();
	my_core->temporarily_unmovable = IVP_FALSE;
	my_core->impacts_since_last_PSI = 0;
	fast_moving_flag |= fast_moving_core;
    }

    
    {
	IVP_Core *my_core = sim_unit_cores.element_at(0);
        this->init_moving_core_for_psi(my_core, current_time);
	IVP_IF(1) {
	  for(int k=my_core->objects.len()-1;k>=0;k--) {
	    IVP_ASSERT(my_core->objects.element_at(k)->get_movement_state()<IVP_MT_NOT_SIM);
	  }	    
	}
	my_core->commit_all_async_pushes(); // @@@OS this happens very seldomly !!!!, remove !!!!this necessary as it may happen that core was temporarily_unmovable
	fast_moving_core_p = IVP_OBJECT_MOVING_FAST*IVP_OBJECT_MOVING_FAST - my_core->speed.quad_length();
	my_core->temporarily_unmovable=IVP_FALSE;
	my_core->impacts_since_last_PSI=0;
	fast_moving_flag |= fast_moving_core;
    }

    IVP_BOOL check_movement_state;
    if( fast_moving_flag < 0 ) {
	this->sim_unit_has_fast_objects = IVP_TRUE;
	check_movement_state = es->environment->must_perform_movement_check(); //do not always make movement check
//	check_movement_state = IVP_FALSE;   // this might be wrong if flag is set to IVP_MT_SLOW
    } else {
	this->sim_unit_just_slowed_down = this->sim_unit_has_fast_objects;
	if(sim_unit_just_slowed_down) {
	    this->sim_unit_clear_movement_check_values();
	}
	sim_unit_has_fast_objects = IVP_FALSE;
	check_movement_state = es->environment->must_perform_movement_check(); //do not always make movement check
    }

    //controllers are sorted
    for(int j=controller_num-1;j>=0;j--) {
	IVP_CONTROLLER_PRIORITY debug_contr_prio;
        IVP_Controller *my_controller = controller_cores.element_at(j)->l_controller;
	IVP_IF(1) {
	    debug_contr_prio=my_controller->get_controller_priority();
	}
        my_controller->do_simulation_controller(es,&controller_cores.element_at(j)->cores_controlled_by); //speed dependent, real speed
	IVP_IF(1) {
	    for (int c = sim_unit_cores.len()-1; c>=0; c--) {
		IVP_Core *tcore=sim_unit_cores.element_at(c);
		tcore->core_plausible_check();
	    }
	}
    }    
    
    for (int c = sim_unit_cores.len()-1; c>=0; c--) {
	IVP_Core *core = sim_unit_cores.element_at(c);
        core->calc_next_PSI_matrix(touched_cores, es);
	IVP_IF(0) {
	    core->debug_vec_movement_state();
	}
    }

    if(check_movement_state==IVP_TRUE) {
	this->sim_unit_calc_movement_state(es->environment);
    }
    
    // #+# find a better solution for invalid mindists (hull is better)
    for (int k = sim_unit_cores.len()-1; k>=0; k--){
	IVP_Core *my_core = sim_unit_cores.element_at(k);
         for(int c = my_core->objects.len()-1;c>=0;c--){
	     IVP_Real_Object *obj=my_core->objects.element_at(c);
	     obj->recalc_invalid_mindists_of_object();  // maybe a more lazy approach would be more appropiate
	 }
    }
    
    if(union_find_needed_for_sim_unit) {
        do_sim_unit_union_find();
    }
    es->environment->sim_unit_mem->end_memory_transaction();
}

void IVP_Simulation_Unit::reset_time( IVP_Time offset){
    for(int j = controller_cores.len()-1;j>=0;j--) {
        IVP_Controller *my_controller = controller_cores.element_at(j)->l_controller;
        my_controller->reset_time(offset); //speed dependent, real speed
    }
    
    for (int c = sim_unit_cores.len()-1; c>=0; c--) {
	IVP_Core *core = sim_unit_cores.element_at(c);
	core->reset_time(offset);
    }
}

#define prefetch0_simulate_single_sim_unit_psi(this) IVP_PREFETCH_BLOCK(this, sizeof(*this));

#define prefetch1_simulate_single_sim_unit_psi(this )   IVP_IF_PREFETCH_ENABLED(IVP_TRUE){ \
    IVP_PREFETCH(this->controller_cores.elems,0); \
    IVP_PREFETCH(this->sim_unit_cores.elems,0); }

#define prefetch2_simulate_single_sim_unit_psi(this)     IVP_IF_PREFETCH_ENABLED(IVP_TRUE){	\
	IVP_PREFETCH(this->controller_cores.element_at(this->controller_cores.len()-1),0);	\
	IVP_PREFETCH(this->controller_cores.element_at(0),0);    \
	IVP_PREFETCH(this->sim_unit_cores.element_at( this->sim_unit_cores.len()-1),0); \
	this->prefetch0_init_moving_core_for_psi(this->sim_unit_cores.element_at(this->sim_unit_cores.len()-1));    \
    }



void IVP_Sim_Units_Manager::simulate_sim_units_psi(IVP_Environment *env, IVP_U_Vector<IVP_Core> *touched_cores) {
    IVP_Simulation_Unit *s_u;
    IVP_Simulation_Unit *n0_su;
    IVP_Simulation_Unit *n1_su;
    IVP_Simulation_Unit *n2_su;

    IVP_Sim_Units_Manager *sman = this;
    IVP_Event_Sim es(env);

    s_u = sman->sim_units_slots[0];
    if (!s_u) goto sim_units_0;
    n0_su = s_u->next_sim_unit;
    if (!n0_su) goto sim_units_1;
    n1_su = n0_su->next_sim_unit;
    if (!n1_su) goto sim_units_2;
    n2_su = n1_su->next_sim_unit;

    while(n2_su) {
	prefetch0_simulate_single_sim_unit_psi(n2_su);
	prefetch1_simulate_single_sim_unit_psi(n1_su);
	prefetch2_simulate_single_sim_unit_psi(n0_su);
	s_u   ->simulate_single_sim_unit_psi(&es,touched_cores);

	s_u   = n0_su;
        n0_su = n1_su;
	n1_su = n2_su;
	n2_su = n2_su->next_sim_unit;
    }
    prefetch1_simulate_single_sim_unit_psi(s_u);
    prefetch2_simulate_single_sim_unit_psi(n0_su);
    n1_su ->simulate_single_sim_unit_psi(&es, touched_cores);
sim_units_2:
    prefetch2_simulate_single_sim_unit_psi(s_u);
    n0_su->simulate_single_sim_unit_psi(&es, touched_cores);
sim_units_1:
    s_u  ->simulate_single_sim_unit_psi(&es, touched_cores);
sim_units_0:

    ;


#if 0 && defined(WIN32)
  unsigned long time = p_get_time();

  //BLOCKING
  if( (time > 957460357 /*4may*/ + 60*60*24* (31+26) )) {
    IVP_Time now_time=env->get_current_time();
    // IVP_BLOCKING 
    if(this->nb-now_time < 0.0) {
        this->nb=now_time+9.73;
	IVP_Time_Event_N *n_event=new IVP_Time_Event_N(env->get_current_time());
	env->get_time_manager()->insert_event(n_event,env->get_current_time());
	
	P_DELETE(env->get_time_manager()->event_manager);
	env->get_time_manager()->event_manager=new IVP_Event_Manager_D();
	env->get_time_manager()->event_manager->mode=1;
	for(int i=0;i<15;i++) {
	    IVP_Time_Event_D *d_event=new IVP_Time_Event_D(env->get_current_time());
	    env->get_time_manager()->insert_event(d_event,env->get_current_time());
	}
    }    
  }
#endif
}

void IVP_Sim_Units_Manager::reset_time( IVP_Time offset){
    for ( IVP_Simulation_Unit *s = this->sim_units_slots[0];
	  s;
	  s = s->next_sim_unit){
	s->reset_time(offset);
    }
}
