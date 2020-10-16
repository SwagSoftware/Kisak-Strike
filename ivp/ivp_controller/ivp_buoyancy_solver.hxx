// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef WIN32
#	pragma interface
#endif


class IVP_Controller_Buoyancy;
class IVP_Compact_Triangle;

/********************************************************************************
 *	Name:	  	IVP_Buoyancy_Solver
 *	Description:	IVP_Buoyancy_Solver calculates forces which affect objects
 *                      placed into a fluid (i.e. buoyancy, dampening, current of fluid)
 *      Note:           IVP_Buoyancy_Solver is only to be used internally by
 *                      IVP_Controller_Buoyancy
 ********************************************************************************/
class IVP_Buoyancy_Solver {
private:

    IVP_FLOAT buoyancy_eps;   //denotes an epsilon value
    IVP_Controller_Buoyancy *controller_buoyancy;
    
    IVP_Environment *environment;               //hold the environment the object is part of
    IVP_Core *core;                             //holds the core the solver is working on
    IVP_DOUBLE delta_time;                          //holds the time interval one PSI takes

    IVP_BOOL simulate_wing_behavior;  //if set to IVP_TRUE then the dampening forces of each triangle will be directed in negative normal direction of this triangle
                                       //instead of the direction of the medium's current.
    IVP_FLOAT medium_density;          //denotes the density of the fluid in (kg/m^3)
    IVP_FLOAT pressure_damp_factor;   //resistance constant that influences the strength of the pressure dampening (caused by suction behind object)
    IVP_FLOAT friction_damp_factor;   //resistance constant (depending on the object's shape) that influences the strength of the friction dampening (caused by push tension)
    IVP_FLOAT viscosity_factor;                           //viscosity of the fluid = 1 - (viscosity_factor * delta_time)
    IVP_U_Float_Point resulting_speed_of_current_ws;  //speed of fluid's current that takes into account the speed of the fluid directly surounding the object
    IVP_U_Float_Point resulting_speed_of_current_os;  //speed of fluid's current that takes into account the speed of the fluid directly surounding the object
    
    
    IVP_FLOAT torque_factor;            //influences how much the point the dampening impulse has an effect on will be shifted towards the current
    IVP_FLOAT viscosity_input_factor;   //defines how much influence the viscosity of the fluid will have on the object; depends on aerodynamic shape of object

    IVP_FLOAT ball_rot_dampening_factor;  //influences the strength of dampening of a ball's rotation, which can result e.g. from the ball not falling straightly into the medium
    


    /*** Methods ***/

    // ball related methods

    //calls the appropriate methods to calculate the buoyancy and the dampening impulse
    void compute_values_for_one_ball(const IVP_Real_Object *object,
				     const IVP_U_Float_Hesse *surface_os,
				     const IVP_U_Float_Point *rel_speed_of_current_os);

    //computes the volume and its center that lies under the surface of the medium
    void compute_buoyancy_values_for_one_ball(const int &decision,
					      const IVP_FLOAT &distance,
					      const IVP_FLOAT &radius,
					      const IVP_U_Float_Hesse *surface_os,
					      const IVP_U_Float_Point *geom_center_os);

    //computes the values which determine the dampening of a given ball
    void compute_dampening_values_for_one_ball(const int &decision,
					       const IVP_FLOAT &distance,
					       const IVP_FLOAT &radius,
					       const IVP_U_Float_Point *geom_center_os,
					       const IVP_U_Float_Point *rel_speed_of_current_os_,
					       const IVP_U_Float_Hesse *speed_plane_os_,
					       const IVP_U_Float_Hesse *surface_os,
					       const IVP_U_Float_Point *p1_os,
					       const IVP_U_Float_Point *p2_os);

    //method to determine the two points that result from the disection of a ball with a straight
    //The straight is defined by the disection of two planes
    int compute_disection_points_with_ball(const IVP_U_Float_Hesse *plane1_os,
					   const IVP_U_Float_Hesse *plane2_os,
					   const IVP_U_Float_Point *geom_center_os,
					   const IVP_FLOAT &radius,
					   IVP_U_Float_Point *p1_os, IVP_U_Float_Point *p2_os);



    // polygon related methods
    
    //calls the appropriate methods to calculate the buoyancy and the dampening impulse
    void compute_values_for_one_polygon(IVP_Real_Object *object, const IVP_U_Float_Hesse *surface_os);

    //called by 'compute_values_for_one_polygon'
    void compute_values_for_one_ledge(IVP_Real_Object *object,
				      const IVP_Compact_Ledge *current_ledge,
				      const IVP_U_Float_Hesse *surface_os,
				      const IVP_U_Float_Point *s_point);

    //called by 'compute_values_for_one_ledge'
    void compute_values_for_one_triangle(IVP_Real_Object *object,
					 const IVP_Compact_Triangle *triangle,
					 const IVP_U_Float_Hesse *surface_os,
					 const IVP_U_Float_Point *s_point,
					 const IVP_Compact_Ledge *current_ledge);

    //computes the buoyancy values for the pyramid built by one triangle
    inline void compute_volumes_and_centers_for_one_pyramid(IVP_Real_Object *object,
							    const IVP_U_Float_Point *triangle_points[5],
							    const IVP_FLOAT distance[6],
							    const int &decision,
							    const int &index_positiv,
							    const int &index_neg,
							    const IVP_U_Float_Point *s_point);

    //compute the dampening values provided to the whole result by one triangle of the object's surface
    inline void compute_rotation_and_translation_values_for_one_triangle(IVP_Real_Object *object,
									 const IVP_Compact_Triangle *current_triangle,
									 const IVP_U_Float_Point *triangle_points[5],
									 const IVP_Compact_Ledge *current_ledge,
									 const IVP_FLOAT distance[6],
									 const int &decision,
									 const int &index_positiv,
									 const int &index_neg);

    //returns the volume of the pyramid defined by the four point arguments
    inline IVP_FLOAT compute_pyramid_volume( const IVP_U_Float_Point *p1,
					 const IVP_U_Float_Point *p2,
					 const IVP_U_Float_Point *p3,
					 const IVP_U_Float_Point *s_point);


    
    
public:

    IVP_FLOAT object_visible_surface_content_under;   //holds the area of the object's surface in the fluid that the dampening has an effect on
    IVP_U_Float_Point sum_impulse;                //sum of the dampening impulses that have an effect on the different triangles of an object
    IVP_U_Float_Point sum_impulse_x_point;        //sum of the cross products of the dampening impulses and the points on the triangles they have an effect on
    IVP_U_Float_Point sum_impulse_x_movevector;   //sum of the cross products of the dampening impulses and the vectors that denote the shift of the impulse points towards the current

    IVP_FLOAT volume_under;                	        // holds the volume of the object's part that lies under the surface
    IVP_U_Float_Point volume_center_under;     	// holds the center of the volume of the object's part that lies under the surface

    //main method of this class; computes the buoyancy/dampening forces and the points on the object they have an effect on
    //surface_ws is the surface of the fluid in world coordinates
    IVP_BOOL compute_forces( const IVP_U_Float_Point *rel_speed_of_current_os, const IVP_U_Float_Hesse *surface_ws, IVP_Real_Object *object );
    
    //constructor
    IVP_Buoyancy_Solver( IVP_Core *core_, IVP_Controller_Buoyancy *cntrl, const class IVP_Template_Buoyancy *input, const IVP_U_Float_Point *resulting_speed_of_current_ws );
};
