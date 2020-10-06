
// IVP_EXPORT_PROTECTED

class IVP_Vector_of_Hull_Managers_1: public IVP_U_Vector<IVP_Hull_Manager_Base> {
    void *elem_buffer[1];
public:
    IVP_Vector_of_Hull_Managers_1(): IVP_U_Vector<IVP_Hull_Manager_Base>(&elem_buffer[0], 1){;};
};


/********************************************************************************
 *	type:		private, internal
 *	section:	calc_next_PSI_matrix subfunctions
 *	Description:	these functions are used to calc the next psi matrix
 ********************************************************************************/

class IVP_Calc_Next_PSI_Solver {
    class IVP_Core *core;

    inline void calc_rotation_matrix( IVP_FLOAT delta_sim_time, IVP_U_Quat *q_core_f_core_out); 	// calc q_core_f_core
    IVP_DOUBLE get_longest_time_step_dependent_on_rot();

    //void reduce_energy_by_damping(IVP_DOUBLE remove_energy); //not used at moment
    //void get_next_simulation_slot(IVP_DOUBLE next_lowest_psi,int *next_used_slot,int *time_step);
    
    void calc_psi_rotation_axis(const IVP_U_Quat *q_core_f_core);                     // calc the rotation matrix

    static inline void prefetch0_calc_next_PSI_matrix(IVP_Core *core);
public:
    void set_transformation( const IVP_U_Quat *rotation, const IVP_U_Point *position, IVP_BOOL optimize_for_repeated_calls);

    void calc_next_PSI_matrix(class IVP_Event_Sim *event_sim, IVP_U_Vector<IVP_Hull_Manager_Base> *active_hull_managers_out);

    static void commit_all_calc_next_PSI_matrix(IVP_Environment *, IVP_U_Vector<IVP_Core> *cores_which_needs_calc_next_psi, IVP_U_Vector<IVP_Hull_Manager_Base> *active_hull_managers_out);

    static void commit_one_hull_manager( IVP_Environment *, IVP_U_Vector<IVP_Hull_Manager_Base> *active_hull_managers); // if you now there is only one hull_manager
    static void commit_all_hull_managers( IVP_Environment *, IVP_U_Vector<IVP_Hull_Manager_Base> *active_hull_managers);
    

    /********************************************************************************
     *	type:		IVP_Calc_Next_PSI_Solver
     *	parameter:	core_in, the core to calculate
     *	Description:	these functions are used to calc the next psi matrix
     ********************************************************************************/
    
    IVP_Calc_Next_PSI_Solver( IVP_Core *core_in ) { core = core_in;};
};
