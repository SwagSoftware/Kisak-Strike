// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

class IVP_Real_Object;
class IVP_Core;

#include <ivp_great_matrix.hxx>

#ifndef IVP_CONSTRAINT_INCLUDED
#include <ivp_constraint.hxx>
#endif

#define IVP_CONSTRAINT_CAR_MAX_WHEELS 12

class IVP_Constraint_Solver_Car;

//=============================================================================
//
// Car constraint object
//
class IVP_Constraint_Car_Object
{
public:

	// Constructor/Deconstructor
    IVP_Constraint_Car_Object( IVP_Constraint_Solver_Car *solver_car, IVP_Real_Object *i_real_obj_app, 
		                       IVP_Real_Object *i_real_obj_body, IVP_U_Float_Point *target_Bos_override = NULL );
    ~IVP_Constraint_Car_Object();

	// Core
    IVP_Core *get_core() const					{ return real_object->get_core(); }

public:

    IVP_Real_Object				*real_object;				// core could become invalid !
    IVP_Constraint_Solver_Car	*solver_car;
    IVP_U_Matrix				target_position_bs;			// in body system
	IVP_U_Float_Point			last_contact_position_ws;
	IVP_FLOAT					last_skid_value;
	IVP_Time					last_skid_time;
    IVP_Constraint				*fix_wheel_constraint;
};

//=============================================================================
//
// Car constraint solver
//
// Special (very fast) vehicle constraint system: A body with 4 wheels.
// NOT constrained is the vertical positions (is solved by springs)
// and (of course not!) the wheel spin.
//
class IVP_Constraint_Solver_Car : public IVP_Controller_Dependent 
{
public:

	// Constructor/Deconstructor
    IVP_Constraint_Solver_Car( IVP_COORDINATE_INDEX right, IVP_COORDINATE_INDEX up, IVP_COORDINATE_INDEX forward, IVP_BOOL is_left_hand );
    ~IVP_Constraint_Solver_Car();

	// Preparation for run time simulation, sets co_matrix.
    IVP_RETURN_TYPE			init_constraint_system( IVP_Environment *env, IVP_Real_Object *body, IVP_U_Vector<IVP_Real_Object> &wheels,
											        IVP_U_Vector<IVP_U_Float_Point> &p_Bos ); 
	// Simulation.
    void					do_simulation_controller( IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list );

	IVP_U_Vector<IVP_Core>	*get_associated_controlled_cores()		{ return &cores_of_constraint_system; };
    int						get_num_of_appending_terminals()		{ return wheel_objects.len(); };			// e.g. 4 for four wheels, might be more flexible in the future

public:
    
    friend class IVP_Constraint_Solver_Car_Builder;

public:

    IVP_Constraint_Car_Object				*body_object;
    IVP_U_Vector<IVP_Constraint_Car_Object>	wheel_objects;
    IVP_Great_Matrix_Many_Zero				co_matrix;			// inverted matrix for delta vals -> pushes (*time), all in B.
	int										x_idx;				// specifies the axis which points to the right of the car
	int										y_idx;				// specifies the axs which points down
	int										z_idx;				// specifies the axis which points forward
	IVP_FLOAT								angle_sign;			// 1.0f or -1.0f depending on right/left hand system

#if 0
	// Debugging!
	IVP_FLOAT								m_wheelRotationTorque[4][3];
	IVP_FLOAT								m_wheelTranslationTorque[4][3];
#endif

protected:

    void					core_is_going_to_be_deleted_event( IVP_Core *core );
    IVP_DOUBLE				get_minimum_simulation_frequency()						{ return 30;}
    IVP_CONTROLLER_PRIORITY	get_controller_priority()								{ return IVP_CP_CONSTRAINTS; };

	// Simulation.
	void					do_simulation_controller_rotation( IVP_Event_Sim *es, IVP_Core *core_B, const IVP_U_Matrix *m_world_f_B );

protected:

    IVP_U_Vector<IVP_Core>	cores_of_constraint_system;
    IVP_BOOL				constraint_is_disabled[6];								// default: all IVP_FALSE
    IVP_BOOL				local_translation_in_use;								// flag for plan B
    IVP_Constraint			*c_local_ballsocket[IVP_CONSTRAINT_CAR_MAX_WHEELS];		// remember for plan B
    IVP_Environment			*environment;
    int						psis_left_for_plan_B;									// internal countdown for deactivating plan B
    IVP_FLOAT				max_delta_speed;										// max delta speed allowed for constraint system inner movements
};

//=============================================================================
//
// Car constraint solver builder
//
// Contains temporarily used vars.  Helps building up th car solver matrix.
//
class IVP_Constraint_Solver_Car_Builder
{
public:

    IVP_Constraint_Solver_Car_Builder( IVP_Constraint_Solver_Car *car_solver );

    void				disable_constraint( int co_idx );
    IVP_RETURN_TYPE		calc_constraint_matrix();

public:

    IVP_Constraint_Solver_Car	*car_solver;
    int							n_appends;				// e.g. 4 for four wheels, determines matrix size
    int							n_constraints;			// for matrix size, decreases with num of disabled constraints
    
private:

    void				calc_pushing_behavior( int A_obj_idx, int push_vec_idx );		// translation, in init

private:

    IVP_Great_Matrix_Many_Zero	tmp_matrix;				// temp, non-inverted matrix
};

