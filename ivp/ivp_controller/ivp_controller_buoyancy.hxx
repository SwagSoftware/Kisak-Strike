// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC


#ifndef _IVP_CONTROLLER_BUOYANCY_INCLUDED
#define _IVP_CONTROLLER_BUOYANCY_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#ifndef _IVP_ATTACHER_TO_CORES_INCLUDED
#	include <ivp_attacher_to_cores.hxx>
#endif

#ifndef IVP_CONTROLLER_INCLUDED
#	include <ivp_controller.hxx>
#endif

#ifndef IVP_SET_INCLUDED
#	include <ivu_set.hxx>
#endif

//class IVP_Multidimensional_Interpolator;
#include "ivp_multidimensional_interp.hxx"


/********************************************************************************
 *	Name:	       	IVP_Template_Buoyancy
 *	Description:	Template that holds values for user definable variables
 *                      used by the fluid dynamics related methods
 ********************************************************************************/
class IVP_Template_Buoyancy {
public:
    IVP_BOOL simulate_wing_behavior;  //if set to IVP_TRUE then the dampening forces of each triangle will be directed in negative normal direction of this triangle
                                       //instead of the direction of the medium's current.
    
    IVP_FLOAT medium_density;       //density of the medium in 'kg/m^3'
    IVP_FLOAT pressure_damp_factor;   //resistance constant that influences the strength of the pressure dampening (caused by suction behind object)
    IVP_FLOAT friction_damp_factor;   //resistance constant (depending on the object's shape) that influences the strength of the friction dampening (caused by push tension)
    IVP_FLOAT torque_factor;        //influences how much the point the dampening impulse has an effect on will be shifted towards the current
    IVP_FLOAT buoyancy_eps;         //denotes an epsilon value

    IVP_FLOAT ball_rot_dampening_factor;  //influences the strength of dampening of a ball's rotation, which can result e.g. from the ball not falling straightly into the medium
    IVP_FLOAT viscosity_factor;           //viscosity_of_medium = 1 - (viscosity_factor * delta_psi_time)
    IVP_FLOAT viscosity_input_factor;     //defines how much influence the viscosity of the fluid will have on the object; depends on aerodynamic shape of object

    IVP_BOOL use_interpolation;   //defines if the fluid dynamics are calculated anew in every PSI step or if we try to interpolate them from values calculated in PSIs before
    int max_interpolation_tries;  //defines how many interpolations will be tried in controller_buoyancy before the computation (via the buoyancy_solver) will be forced
    int max_tries_nr_of_vectors_involved;  //defines how many interpolations will be tried in controller_buoyancy with the same or an increasing number of involved vectors
    IVP_BOOL use_stochastic_insertion;  //defines if a stochastic algorithm should be used to determine where to put a new vector into 'previous_xxxxx' of 'IVP_Multidimensional_Interpolator'
    IVP_BOOL insert_extrapol_only;   //defines if in case the MI failed to interpolate a new solution only the extrapolated solution based on the calculation of the buoyancy solver should be inserted
                                     //into 'previous_solutions' or if additionally also the buoyancy solver's calculated solution is inserted into 'previous_solutions'

    //defines weights that determine the amount of influence of the different input values on the interpolation in Multidimensional_Interpolator
    struct MI_Weights {
	IVP_FLOAT weight_current_speed;
	IVP_FLOAT weight_surface;
	IVP_FLOAT weight_rot_speed;
    } mi_weights;

    //maximum value the residuum of the LINFIT algortihm in the Multidimensional Interpolator is allowed to have
    //It describes the maximum deviation of LINFIT's result from the optimum
    IVP_FLOAT max_res;

    //defines how many PSIs a new solution will be extrapolated into the future by the Multidimensional Interpolator
    //if an interpolation from previous values was not possible
    int nr_future_psi_for_extrapolation;

    //constructor
    IVP_Template_Buoyancy() {
	simulate_wing_behavior = IVP_FALSE;
	
	//medium_damp_factor = 1.0f;
	pressure_damp_factor = 0.1f;
	friction_damp_factor = 0.05f;
	torque_factor = 1.0f;
	medium_density = 0.3f;
	buoyancy_eps = 1E-10f; //1e-3f;
	viscosity_factor = 0.01f;

	viscosity_input_factor = 2.0f;
	ball_rot_dampening_factor = 0.01f;

	use_interpolation = IVP_FALSE;
	max_interpolation_tries = 10;
	max_tries_nr_of_vectors_involved = 15;
	use_stochastic_insertion = IVP_TRUE;
	insert_extrapol_only = IVP_TRUE;

	mi_weights.weight_current_speed = 1.0f;
	mi_weights.weight_surface = 1.0f;
	mi_weights.weight_rot_speed = 1.0f;

	max_res = 1e-2f;
	nr_future_psi_for_extrapolation = 1;
    }
};

#define INPUT_VECTOR_LENGTH                    ( (sizeof(IVP_U_Float_Point) * 2 + sizeof(IVP_U_Float_Hesse)) / sizeof(IVP_FLOAT)) 

/*****************************************************************
 * Name:        Class IVP_Buoyancy_Input
 * Description: Wrapper for instances of 'IVP_MI_Vector' used as
 *              a vector of input values
 *****************************************************************/
class IVP_Buoyancy_Input: public IVP_MI_Vector_Base {
public:
    IVP_U_Float_Hesse surface_os;
    IVP_U_Float_Point rel_speed_of_current_os;
    IVP_U_Float_Point rot_speed;
    
    IVP_Buoyancy_Input(){
	nr_of_elements = INPUT_VECTOR_LENGTH;
        weight_statistic = 0.0f;
#ifdef IVP_VECTOR_UNIT_FLOAT
	rel_speed_of_current_os.hesse_val = 0.0;
	rot_speed.hesse_val = 0.0;
#endif	
    };
};


#define SOLUTION_VECTOR_LENGTH          ( (sizeof(IVP_U_Float_Point) * 4 + sizeof(IVP_FLOAT)*2) / sizeof(IVP_FLOAT)) 
/*****************************************************************
 * Name:        Class IVP_Buoyancy_Output
 * Description: Wrapper for instances of 'IVP_MI_Vector' used as
 *              a vector of output values
 *****************************************************************/
class IVP_Buoyancy_Output: public IVP_MI_Vector_Base {
public:
    IVP_U_Float_Point volume_center_under;
    IVP_U_Float_Point sum_impulse;
    IVP_U_Float_Point sum_impulse_x_point;
    IVP_U_Float_Point sum_impulse_x_movevector;
    IVP_FLOAT volume_under;
    IVP_FLOAT object_visible_surface_content_under;
    
    IVP_Buoyancy_Output(){ nr_of_elements = SOLUTION_VECTOR_LENGTH;
                           weight_statistic = 0.0f;
                           volume_under = 0.0f;
                           object_visible_surface_content_under = 0.0f;
			   volume_center_under.set(0.0f, 0.0f, 0.0f);
			   sum_impulse.set(0.0f, 0.0f, 0.0f);
			   sum_impulse_x_point.set(0.0f, 0.0f, 0.0f);
			   sum_impulse_x_movevector.set(0.0f, 0.0f, 0.0f);
    };
};


/********************************************************************************
 *	Name:	       	IVP_Controller_Buoyancy
 *	Description:    Manages the calculation of fluid dynamic related forces for
 *                      a core of objects
 *      Note:           One instance of IVP_Controller_Buoyancy is set up for every
 *                      core for which buoyancy forces are wished to be computed.
 *                      Can use IVP_Multidimensional_Interpolator to approximate
 *                      the fluid dynamics for greater perfomance
 ********************************************************************************/
class IVP_Controller_Buoyancy : protected IVP_Controller_Independent {

private:
    class IVP_Attacher_To_Cores_Buoyancy *attacher_buoyancy;
    IVP_Core *core;    //holds the core for which this controller manages the fluid dynamics

    friend class IVP_Buoyancy_Solver;
    IVP_U_Float_Point relative_speed_of_current_in_objects_vicinity_old;  //holds the speed of the fluid's current in the vicinity of the object from the PSI before
    IVP_FLOAT core_visible_surface_content_under;                             //the area of the surface of the core that is dampened
    IVP_FLOAT core_visible_surface_content_under_old;                         //the area of the surface of the core that is dampened from the PSI before

    //vector which applies an instance of 'IVP_Multidimensional_Interpolator' to every object in the core of this 'IVP_Controller_Buoyancy'
    typedef struct {
	IVP_Real_Object *object;
	IVP_Multidimensional_Interpolator *mi;
	struct Last_IO {
	    IVP_MI_Vector *last_input_vector;
	    IVP_MI_Vector *last_solution_vector;
	    IVP_DOUBLE last_psi_time;
	} last_io_vectors;  //holds values from the previous PSI run
	//temporary, needed for debugging
	int nr_interpolated;
	int nr_not_interpolated;
    } Attacher_Interpolator;
    Attacher_Interpolator *attacher_interpolator;

    //int max_interpolation_tries;  //defines how many interpolations will be tried before the computation of the forces via the buoyancy_solver will be forced
    int interpolation_counter;    //the actual counter

    friend class IVP_Attacher_To_Cores<IVP_Controller_Buoyancy>;
    /*********************
     * constructor method
     *********************/
    IVP_Controller_Buoyancy(IVP_Attacher_To_Cores<IVP_Controller_Buoyancy> *, IVP_Core *);

    /*********************
     * destructor method
     *********************/
    ~IVP_Controller_Buoyancy();


    /**************************************************************************
     * Name:        calculate_future_extrapolation(...)
     * Description: extrapolates a new input vector for the multidimensional
     *              interpolator from the current input and the previous input
     **************************************************************************/
    IVP_RETURN_TYPE calculate_future_extrapolation(const IVP_Controller_Buoyancy::Attacher_Interpolator::Last_IO *last_io_vectors,
						   const IVP_MI_Vector *new_input,
						   const IVP_MI_Vector *solution_values,
						   const IVP_DOUBLE d_time,
						   const IVP_Time current_time,
						   IVP_MI_Vector *future_input,
						   IVP_MI_Vector *future_solution);



    /*****************************************************************************
     * Name:        use_buoyancy_solver(...)
     * Description: calls the 'IVP_Buoyancy_Solver' and puts the results into
     *              'solution_values_out'
     *****************************************************************************/
    IVP_BOOL use_buoyancy_solver(const IVP_Buoyancy_Input *b_input,
				 const IVP_Template_Buoyancy *,
				 IVP_Buoyancy_Output *solution_values_out,
				 const IVP_U_Float_Point *resulting_speed_of_current_ws,
				 int index_attacher_interpolator);

    /**********************************************************************************
     * Name:        apply_dampening(...)
     * Description: applies the dampening forces to the object provided as a parameter
     **********************************************************************************/
    void apply_dampening( IVP_Real_Object *object,
			  IVP_FLOAT object_visible_surface_content_under,
			  IVP_DOUBLE delta_time,
			  IVP_U_Float_Point *sum_impulse_x_movevector,
			  IVP_U_Float_Point *sum_impulse_x_point,
			  IVP_U_Float_Point *sum_impulse);

    
    /*********************************************************************************
     * Name:        apply_buoyancy_impulse(...)
     * Description: applies the buoyancy forces to the object provided as a parameter
     *********************************************************************************/
    void apply_buoyancy_impulse( IVP_Real_Object *object,
				 IVP_Template_Buoyancy *temp_buoyancy,
				 IVP_DOUBLE delta_time,
				 IVP_FLOAT volume_under,
				 IVP_U_Float_Point *volume_center_under);

    
    /*************************************************************************************
     * Name:        provide_new_input_solution_combination(...)
     * Description: combines various ways of inserting new vectors of input and solution
     *              values into 'previous_inputs' and 'previous_solutions' of the current
     *              'IVP_Multidimensional_Interpolator' instance
     *************************************************************************************/
    void provide_new_input_solution_combination(Attacher_Interpolator *attacher_interpolator,
						IVP_Template_Buoyancy *temp_buoyancy,
						const IVP_MI_Vector *new_input,
						const IVP_MI_Vector *solution_values,
						const IVP_DOUBLE d_time,
						const IVP_Time current_time);

    //for debugging
    int nr_not_interpolated;
    
protected:
    void core_is_going_to_be_deleted_event(IVP_Core *) { delete this;};
    IVP_DOUBLE get_minimum_simulation_frequency() { return 0.0f; };
    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_FORCEFIELDS; };
    
    /********************************************************************
     * Name:        do_simulation_controller(...)
     * Description: Simulates the fluid dynamics in the current PSI step
     *              for the core in its class variable 'core'
     * Note:        do_simulation_controller is called by the internal
     *              simulation unit of the physics engine
     ********************************************************************/
    void do_simulation_controller(IVP_Event_Sim *sim_u,IVP_U_Vector<IVP_Core> *core_list);
public:
    // not for public use
};



/********************************************************************************
 *	Name:	       	IVP_Attacher_To_Cores_Buoyancy
 *	Description:    Attaches a IVP_Controller_Buoyancy to every core provided
 *                      by the list "*set_of_cores"
 ********************************************************************************/
class IVP_Attacher_To_Cores_Buoyancy: public IVP_Attacher_To_Cores<IVP_Controller_Buoyancy> {
protected:
    friend class IVP_Controller_Buoyancy;
    IVP_Template_Buoyancy template_buoyancy;
    IVP_U_Set_Active<IVP_Core> *set_of_cores;
    class IVP_Liquid_Surface_Descriptor *liquid_surface_descriptor;

public:
    /********************************************************************************
     *	Name:	       	get_parameters_per_core
     *	Description:    get parameters for one core
     ********************************************************************************/
    virtual IVP_Template_Buoyancy *get_parameters_per_core( IVP_Core * ){ return &template_buoyancy; };
    
    /********************************************************************************
     *	Name:	       	get_buoyancy_surface
     *	Description:    get surface used for buoyancy
     ********************************************************************************/
    virtual IVP_SurfaceManager *get_buoyancy_surface(IVP_Real_Object *obj){ return obj->get_surface_manager(); };
    IVP_Attacher_To_Cores_Buoyancy(IVP_Template_Buoyancy &templ, IVP_U_Set_Active<IVP_Core> *set_of_cores_, IVP_Liquid_Surface_Descriptor *liquid_surface_descriptor_);
};


#endif
