// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

class IVP_Contact_Point;

class IVP_Debug_Manager {
public:
    ivp_u_bool psi_synchrone; // display every psi
    ivp_u_bool disable_mindists;
    ivp_u_bool disable_friction;
    ivp_u_bool debug_impact;
    ivp_u_bool debug_mindist;
    ivp_u_bool debug_friction;
    ivp_u_bool disable_core_dampening;
    ivp_u_bool display_statistic;
    ivp_u_bool display_debug_vectors;
    
    IVP_DOUBLE all_time_impacts; //number of impacts so far
    IVP_DOUBLE psi_counter;   //number of PSI processed so far
    
    IVP_FLOAT last_impact_translation_speed_normal;
    IVP_FLOAT last_impact_rot_speed_normal;
    IVP_FLOAT last_impact_rot_speed_whole;
    
    FILE	*out_deb_file;
    int         file_nr; //revolve to next filename (10 pieces) 
    IVP_BOOL	file_out_impacts;
    IVP_BOOL    debug_imp_sys;
    IVP_BOOL    revolve_deb_file;
  
    IVP_BOOL    check_fs;
    int arbitrary_counter;
    int  arbitrary_flag;

    void clear_debug_manager();
    void init_debug_manager();
    ~IVP_Debug_Manager();
    IVP_Debug_Manager();
};


