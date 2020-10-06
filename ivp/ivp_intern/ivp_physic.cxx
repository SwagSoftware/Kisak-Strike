// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#ifndef WIN32
#	pragma implementation "ivp_physics.hxx"
#endif

#include <ivp_physics.hxx>
#include <ivp_debug_manager.hxx> //because of debug psi_synchrone

void IVP_Statistic_Manager::output_statistic() {
    printf("nr_impacts %d  hard_resc %d  resc_after %d  delayed %d  sys_strt %d sys_imps %d unmov %d  coll_check %d  mindists %d  gen_mindists %d ov %d\n",
	   impact_counter,impact_hard_rescue_counter,impact_rescue_after_counter,impact_delayed_counter,impact_sys_num,impact_sum_sys,impact_unmov,impact_coll_checks, sum_of_mindists, mindists_generated, range_world_exceeded);
    clear_statistic();
    IVP_IF(1){
	IVP_Debug_Manager *deb_man = l_environment->get_debug_manager();
	if(deb_man->revolve_deb_file) {
	    fclose(deb_man->out_deb_file);
	    deb_man->file_nr++;
	    if(deb_man->file_nr>9) {
	      deb_man->file_nr=0;
	    }
	#ifndef GEKKO
	    char *next_name=p_make_string("debugout%d",deb_man->file_nr);
	    deb_man->out_deb_file=fopen(next_name,"w");
	    printf("just_opened %s\n",next_name);
	    P_FREE(next_name);
	#endif
	}
    }
}

void IVP_Statistic_Manager::clear_statistic() {
    max_rescue_speed=0.0f;
    max_speed_gain=0.0f;
    impact_sys_num=0;
    impact_counter=0;
    impact_sum_sys=0;
    impact_hard_rescue_counter=0;
    impact_rescue_after_counter=0;
    impact_delayed_counter=0;
    impact_coll_checks=0;
    mindists_deleted = 0;
    mindists_generated = 0;
    processed_fmindists = 0;
    range_intra_exceeded = 0;
    range_world_exceeded = 0;
    impact_unmov=0;
}

IVP_Statistic_Manager::IVP_Statistic_Manager(){
    P_MEM_CLEAR(this);
}


IVP_Application_Environment::IVP_Application_Environment(){
    P_MEM_CLEAR(this);
    n_cache_object = 256;

#if defined(PSXII) && 0
    scratchpad_addr = (char *)0x70000000;
    scratchpad_size = 0x4000;
#endif
}
