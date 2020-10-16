
#include <ivp_physics.hxx>
#ifndef WIN32
#	pragma implementation "ivp_buoyancy_solver.hxx"
#endif

#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_buoyancy_solver.hxx>
#include <ivu_geometry.hxx>
#include <ivp_compact_ledge_solver.hxx>
#include <ivp_controller_buoyancy.hxx>

//define some decision variables
#define DAMPENING_WITH_MOVEVECTOR               1
#define DAMPENING_WITH_PARTICLE_ACCELERATION    1

#define BALL_DAMPENING_CALC_PUSHING_FORCES       1
#define BALL_DAMPENING_CALC_SUCKING_FORCES      -1

//return surface speed in object coordinate system
// #+# do everything in object space !!!!
void ivp_core_get_surface_speed_os(IVP_Core *pc,IVP_Real_Object *object, const IVP_U_Float_Point *point_os, IVP_U_Float_Point *speed_out_os) {
    const IVP_U_Float_Point *rot_speed_var = &pc->rot_speed;
    const IVP_U_Float_Point *center_speed_var_world = &pc->speed;
	
    //convert point_os into non_float version
    
    //convert non_float_point_os into core system
    IVP_U_Point point_ws;
    IVP_U_Float_Point point_core;

    IVP_Cache_Object *cache_object = object->get_cache_object_no_lock();  //needed to transform coordinates into other coord. systems
    cache_object->transform_position_to_world_coords(point_os, &point_ws);

    const IVP_U_Matrix *m_world_f_core_PSI = object->get_core()->get_m_world_f_core_PSI();
    m_world_f_core_PSI->vimult4( &point_ws, &point_core );
    
    IVP_U_Float_Point speed_core;
    speed_core.inline_calc_cross_product(rot_speed_var,&point_core);
	
    IVP_U_Float_Point speed_out_ws;
    pc->m_world_f_core_last_psi.vmult3(&speed_core,&speed_out_ws); // transform to world coords
    speed_out_ws.add(center_speed_var_world);
	
    IVP_U_Float_Point float_speed_out_ws;    float_speed_out_ws.set(&speed_out_ws);
    cache_object->transform_vector_to_object_coords(&float_speed_out_ws, speed_out_os);
	
    IVP_IF(0) {
	//template_polygon_current_values->sum_impulse.print("");
	if ((IVP_Inline_Math::fabsd(speed_out_os->k[0]) > 1E1f) ||
	    (IVP_Inline_Math::fabsd(speed_out_os->k[1]) > 1E1f) ||
	    (IVP_Inline_Math::fabsd(speed_out_os->k[2]) > 1E1f)) {
	    CORE;
	}
    }
}

IVP_Buoyancy_Solver::IVP_Buoyancy_Solver( IVP_Core *core_, IVP_Controller_Buoyancy *cntrl, const class IVP_Template_Buoyancy *input, const IVP_U_Float_Point *resulting_speed_of_current_ws_ ){
    core = core_;
    environment = core->environment;

    simulate_wing_behavior = input->simulate_wing_behavior;
    
    buoyancy_eps = input->buoyancy_eps;
    medium_density = input->medium_density;
    pressure_damp_factor = input->pressure_damp_factor * 0.5f * medium_density;
    friction_damp_factor = input->friction_damp_factor * 0.5f * medium_density;
    torque_factor = input->torque_factor;
    controller_buoyancy = cntrl;
    
    ball_rot_dampening_factor = input->ball_rot_dampening_factor;
    viscosity_input_factor = input->viscosity_input_factor;
	
    object_visible_surface_content_under = 0.0f;
    sum_impulse_x_point.set(0.0f, 0.0f, 0.0f);
    sum_impulse_x_movevector.set(0.0f, 0.0f, 0.0f);
    sum_impulse.set(0.0f, 0.0f, 0.0f);
    viscosity_factor = input->viscosity_factor;
    resulting_speed_of_current_ws.set(resulting_speed_of_current_ws_);

	
    //initialize buoyancy class variables with zero values
    volume_under = 0;
    volume_center_under.set_to_zero();
}

IVP_BOOL IVP_Buoyancy_Solver::compute_forces( const IVP_U_Float_Point *rel_speed_of_current_os, const IVP_U_Float_Hesse *surface_os, IVP_Real_Object *object ) {
	
	
	// track object, but don't spend CPU on buoyancy solve
	if ( medium_density <= 0 )
		return IVP_FALSE;
    //initialize some global variables in case this solver should be used more than once
    object_visible_surface_content_under = 0.0f;
    sum_impulse_x_point.set(0.0f, 0.0f, 0.0f);
    sum_impulse_x_movevector.set(0.0f, 0.0f, 0.0f);
    sum_impulse.set(0.0f, 0.0f, 0.0f);

    {
	//translate resulting_speed_of_current_ws into object coordinate system
	IVP_Cache_Object *cache_object = object->get_cache_object_no_lock();
	cache_object->transform_vector_to_object_coords(&resulting_speed_of_current_ws, &resulting_speed_of_current_os);
    }

    
    switch (object->get_type()) {
    case IVP_BALL: {
			
	//ensure that surface_ws is normized
	IVP_IF(1){
	    IVP_ASSERT( IVP_Inline_Math::fabsd( surface_os->real_length() - 1.0f) < 0.01f);
	}
			
	//compute buoyancy and dampening values for this ball
	compute_values_for_one_ball(object, surface_os, rel_speed_of_current_os);  //compute the volume centers and the volumes under surface_ws for the pushing forces

    }
    break;
    case IVP_POLYGON: {
			
	compute_values_for_one_polygon(object, surface_os);

    }
    default: {
			
    }
    break;
    }
		
    
		
    
    //return value describes if the object is in the water
    if ((volume_under > buoyancy_eps) || (sum_impulse.quad_length() > buoyancy_eps)) {
	// //copy the solution values to the structure
// 	solution_values.volume_under = volume_under;
// 	solution_values.object_visible_surface_content_under = object_visible_surface_content_under;
// 	for (int i=0; i<3; i++) {
// 	    solution_values.volume_center_under[i] = volume_center_under.k[i];
// 	    solution_values.sum_impulse[i] = sum_impulse.k[i];
// 	    solution_values.sum_impulse_x_point[i] = sum_impulse_x_point.k[i];
// 	    solution_values.sum_impulse_x_movevector[i] = sum_impulse_x_movevector.k[i];
// 	}
	return(IVP_TRUE);
    } else {
	return(IVP_FALSE);
    }
}



void IVP_Buoyancy_Solver::compute_buoyancy_values_for_one_ball(const int &decision,
							       const IVP_FLOAT &distance,
							       const IVP_FLOAT &radius,
							       const IVP_U_Float_Hesse *surface_os,
							       const IVP_U_Float_Point *geom_center_os) {
	
    //initialize buoyancy class variables with zero values
    //volume_under = volume_above = 0;
    volume_under = 0;
    volume_center_under.set_to_zero();
    //volume_center_above.set(0,0,0);
    
	
    switch (decision) {
    case 0: { //speed_plane_os and/or ball is completely under surface
		
	volume_under = (4.0f/3.0f) * IVP_PI * (radius*radius*radius);
	volume_center_under.set(geom_center_os);
		
	break;
    }
    case 1:
    case 2:
    case 3:
    case 4: {
		
	IVP_FLOAT height_of_balls_part_under_water = radius + distance;
		
	volume_under = (IVP_PI * height_of_balls_part_under_water*height_of_balls_part_under_water * (3*radius - height_of_balls_part_under_water)) / 3.0f;
		
	IVP_FLOAT volume_center_under_factor = 0.75f * (4.0f *radius*radius - 4.0f * radius*height_of_balls_part_under_water + height_of_balls_part_under_water*height_of_balls_part_under_water) / (3.0f*radius - height_of_balls_part_under_water);
	volume_center_under.set(geom_center_os);
	volume_center_under.add_multiple(surface_os, volume_center_under_factor);  //surface_os has to be normized!
		
	break;
    }
    }
}


void IVP_Buoyancy_Solver::compute_dampening_values_for_one_ball(const int &decision,
								const IVP_FLOAT &distance,
								const IVP_FLOAT &radius,
								const IVP_U_Float_Point *geom_center_os,
								const IVP_U_Float_Point *rel_speed_of_current_os_,
								const IVP_U_Float_Hesse *speed_plane_os_,
								const IVP_U_Float_Hesse *surface_os,
								const IVP_U_Float_Point *p1_os,
								const IVP_U_Float_Point *p2_os) {
	
    //initialize dampening variables with zero values
    IVP_U_Float_Point ball_point_of_impulse; ball_point_of_impulse.set_to_zero();
    IVP_U_Float_Point ball_impulse_vector;ball_impulse_vector.set_to_zero();

    IVP_U_Float_Point rel_speed_of_current_os;
    rel_speed_of_current_os.set(rel_speed_of_current_os_);
    IVP_U_Float_Hesse speed_plane_os;
    speed_plane_os.set(speed_plane_os_);
    speed_plane_os.hesse_val = speed_plane_os_->hesse_val;
	
    
    //will hold the content of the projected surface that´s under the water surface
    IVP_DOUBLE ball_projected_surface_content_under = 0.0f;
    
    switch (decision) {
    case 0: { //speed_plane_os and/or ball is completely under surface
		
	ball_projected_surface_content_under = IVP_PI*radius*radius;  //surface content of speed_plane_os
	ball_point_of_impulse.set(geom_center_os);
    }
    break;
    case 1: { //center lies above the surface and speed_plane_os disects ball under and above the surface

	//check if surface_os and speed_plane_os are nearly parallel
	//if so, the we can switch to case 2
	IVP_U_Float_Point direction_down;
	speed_plane_os.proj_on_plane(surface_os, &direction_down);
	if (direction_down.quad_length() < buoyancy_eps) {
	    //if this is the case, then surface_os and speed_plane_os are nearly parallel, so we can switch to case 2
	    goto case_2;
	}
	direction_down.normize();
		
	IVP_U_Float_Point p1p2_os;
	p1p2_os.subtract(p2_os, p1_os);
		
	//calc point in the middle of the straight that results from the intersection between surface_os and speed_plane_os
	IVP_U_Float_Point Q_os;  //point between p1_os and p2_os
	Q_os.set(p1_os);
	Q_os.add_multiple(&p1p2_os, 0.5f);
		
	IVP_DOUBLE length_p1p2 = p1p2_os.real_length();
	if (length_p1p2 < buoyancy_eps) {  //check for safety reasons
	    ball_projected_surface_content_under = 0.0f;
	    break;
	}
	IVP_U_Float_Hesse p1p2_hesse_os;
	p1p2_hesse_os.set(&p1p2_os);
	p1p2_hesse_os.calc_hesse_val(&Q_os);
	p1p2_hesse_os.normize();
		
		
	/*** calc projected lowest point O_os on arc of ellipse ***/
		
	IVP_U_Float_Point tmp_O1_os;
	IVP_U_Float_Point tmp_O2_os;

    	//int erg =
	compute_disection_points_with_ball(surface_os, &p1p2_hesse_os, geom_center_os, radius, &tmp_O1_os, &tmp_O2_os);
	//IVP_ASSERT( erg == 1 );
	
	//decide which point is the right one and project it onto speed_plane_os => point O_os
	IVP_U_Float_Point tmp_O1O2_os;
	tmp_O1O2_os.subtract(&tmp_O2_os, &tmp_O1_os);
	IVP_U_Float_Point O_os;
	if (speed_plane_os.dot_product(&tmp_O1O2_os) >= 0.0f) {
	    speed_plane_os.proj_on_plane(&tmp_O1_os, &O_os);
	} else {
	    speed_plane_os.proj_on_plane(&tmp_O2_os, &O_os);
	}
		
		
	//check in which direction the ball is heading
	IVP_DOUBLE factor = surface_os->dot_product(&speed_plane_os);
	if (factor <= buoyancy_eps) {  //ball falls into the medium
			
	    /*** compute circle segment under surface ***/
	    IVP_U_Float_Point MQ_os;
	    MQ_os.subtract(&Q_os, geom_center_os);
	    IVP_DOUBLE height = radius - MQ_os.real_length();  //height of disected part of circle
	    IVP_DOUBLE circle_arc = 2*IVP_Inline_Math::fast_asin(length_p1p2 / (2.0f * radius)) * radius;  //length of arc which is cut off speed_plane_os by surface_os
			
	    IVP_DOUBLE surface_content_down = IVP_Inline_Math::fabsd(0.5f * (circle_arc*radius - length_p1p2*(radius - height)));

	    IVP_U_Float_Point center_of_segment_os;
	    if (surface_content_down < buoyancy_eps) { //safety check
		//if this is the case then the ball is nearly out of the water
		center_of_segment_os.set(0.0f, 0.0f, 0.0f);
	    } else {
		//check if speed_plane_os is orthogonal on surface_os; if yes we are almost finished
		if (factor > -buoyancy_eps) { //speed_plane_os is orthogonal on surface_os
		    //direction_down cannot be taken as direction vector
		    ball_point_of_impulse.set(geom_center_os);
		    ball_point_of_impulse.add_multiple(surface_os, length_p1p2*length_p1p2*length_p1p2 / (12 * surface_content_down));
				
		    ball_projected_surface_content_under = surface_content_down;
		    goto calculate_impulse; //already finished here
				
		} else {
		    center_of_segment_os.set(geom_center_os);
		    center_of_segment_os.add_multiple(&direction_down, length_p1p2*length_p1p2*length_p1p2 / (12 * surface_content_down));
		}
	    }
			
			
	    /*** compute part of ellipse above surface ***/
			
			
	    //compute center of whole ellipse
	    IVP_U_Float_Point projected_center_on_surface_os;
	    surface_os->proj_on_plane(geom_center_os, &projected_center_on_surface_os);
	    IVP_U_Float_Point center_of_ellipse_os;
	    speed_plane_os.proj_on_plane(&projected_center_on_surface_os, &center_of_ellipse_os);
			
	    IVP_U_Float_Point edge_O_center_os;
	    edge_O_center_os.subtract(&center_of_ellipse_os, &O_os);
	    IVP_DOUBLE b = edge_O_center_os.real_length();
	    IVP_U_Float_Point edge_Q_center_os;
	    edge_Q_center_os.subtract(&center_of_ellipse_os, &Q_os);
	    IVP_DOUBLE part_b = edge_Q_center_os.real_length();	
	    IVP_DOUBLE radius_of_water_disection = IVP_Inline_Math::ivp_sqrtf(IVP_Inline_Math::fabsd(radius*radius - distance*distance));

	    IVP_DOUBLE surface_content_of_cut_off_ellipse_segment;
	    IVP_U_Float_Point center_of_part_ellipse_os;
	    IVP_DOUBLE surface_content_up;
	    if (b < buoyancy_eps) {  //safety check
		//if this is the case, then speed_plane_os is nearly orthogonal to surface_os, so this part of the dampened surface can be assumed 0
		surface_content_of_cut_off_ellipse_segment = 0;
		surface_content_up = 0.0f; // OS @@@@ ???
		center_of_part_ellipse_os.set(0.0f, 0.0f, 0.0f);
	    } else {

		surface_content_of_cut_off_ellipse_segment = IVP_Inline_Math::fabsd((radius_of_water_disection*b*IVP_Inline_Math::save_acosf(part_b / b) - part_b*(p1p2_os.real_length()*0.5f)));  //segment of ellipse under surface_os
	    
		IVP_DOUBLE surface_content_of_whole_ellipse = (IVP_PI*radius_of_water_disection*b);
		surface_content_up = IVP_Inline_Math::fabsd(surface_content_of_whole_ellipse - surface_content_of_cut_off_ellipse_segment);
			
		//compute the center of the part of the ellipse above surface
		IVP_DOUBLE h = b - part_b;
		IVP_DOUBLE s = 2.0f * IVP_Inline_Math::ivp_sqrtf(IVP_Inline_Math::fabsd(2*h*b - h*h));
		IVP_DOUBLE arc = IVP_Inline_Math::fabsd(2*IVP_Inline_Math::fast_asin( s/(2.0f * b) )*b);
		IVP_DOUBLE A = IVP_Inline_Math::fabsd(0.5f * (arc * b - s*(b - h)));

		IVP_U_Float_Point center_of_cut_off_part_of_ellipse;  //center of small cut-off part of ellipse under surface
		if (A < buoyancy_eps) { //safety check
		    center_of_cut_off_part_of_ellipse.set(0.0f, 0.0f, 0.0f);
		    surface_content_of_cut_off_ellipse_segment = 0.0f;
		} else {
		    edge_O_center_os.normize();  //savety check above (for b)
		    center_of_cut_off_part_of_ellipse.set(&center_of_ellipse_os);
		    center_of_cut_off_part_of_ellipse.add_multiple(&edge_O_center_os, s*s*s / (12 * A));
		}

		if (surface_content_up < buoyancy_eps) { //safety check
		    center_of_part_ellipse_os.set(0.0f, 0.0f, 0.0f);
		    surface_content_up = 0.0f;
		} else {
		    center_of_part_ellipse_os.set_multiple(&center_of_cut_off_part_of_ellipse, -surface_content_of_cut_off_ellipse_segment);
		    center_of_part_ellipse_os.add_multiple(&center_of_ellipse_os, surface_content_of_whole_ellipse);
		    center_of_part_ellipse_os.mult( 1.0f / surface_content_up );
		}
	    }
			
	    /*** compute results ***/
	    ball_projected_surface_content_under = surface_content_up + surface_content_down;

	    if (ball_projected_surface_content_under < buoyancy_eps) {
		ball_projected_surface_content_under = 0.0f;
		//ball_point_of_impulse is still set to (0,0,0)
	    } else {
		ball_point_of_impulse.set_multiple(&center_of_segment_os, surface_content_down);
		ball_point_of_impulse.add_multiple(&center_of_part_ellipse_os, surface_content_up);
		ball_point_of_impulse.mult(1.0f / ball_projected_surface_content_under);
	    }
			
	} else { //ball flies out of the water
			
	    /*** calc ball_projected_surface_content_under ***/
			
	    //compute center of whole ellipse
	    IVP_U_Float_Point projected_center_on_surface_os;
	    surface_os->proj_on_plane(geom_center_os, &projected_center_on_surface_os);
	    IVP_U_Float_Point center_of_ellipse_os;
	    speed_plane_os.proj_on_plane(&projected_center_on_surface_os, &center_of_ellipse_os);
			
	    //calc various ellipse related values
	    IVP_U_Float_Point edge_O_center_os;
	    edge_O_center_os.subtract(&center_of_ellipse_os, &O_os);
	    IVP_DOUBLE b = edge_O_center_os.real_length();
	    IVP_U_Float_Point edge_Q_center_os;
	    edge_Q_center_os.subtract(&center_of_ellipse_os, &Q_os);
	    IVP_DOUBLE radius_of_water_disection = IVP_Inline_Math::ivp_sqrtf(IVP_Inline_Math::fabsd(radius*radius - distance*distance));
	    IVP_DOUBLE height = IVP_Inline_Math::fabsd(b - edge_Q_center_os.real_length());
	    //IVP_DOUBLE length_p1p2 = p1p2_os.real_length();
			
	    //calc circle segment under water
	    IVP_U_Float_Point MQ_os;
	    MQ_os.subtract(&Q_os, geom_center_os);
	    IVP_DOUBLE h = radius - MQ_os.real_length();  //height of disected segment of circle
	    IVP_DOUBLE arc = 2*IVP_Inline_Math::fast_asin(length_p1p2 / (2.0f * radius)) * radius;  //length of arc of circle segment
	    IVP_DOUBLE surface_content_of_circle_segment = IVP_Inline_Math::fabsd(0.5f * (arc*radius - length_p1p2*(radius - h)));

	    if (surface_content_of_circle_segment < buoyancy_eps) {  //safety check
		//if this is the case, then there is near to nothing to damp
		ball_projected_surface_content_under = 0.0f;
		goto calculate_impulse;
	    }
			
	    //calc segment of ellipse under water
	    IVP_DOUBLE surface_content_of_ellipse_segment;
	    if (b < buoyancy_eps) { //safety check
		//if this is the case then surface_os and speed_plane_os are nearly orthogonal, so the projected ellipse has nearly vanished
		surface_content_of_ellipse_segment = 0.0f;
	    } else {
		surface_content_of_ellipse_segment = b*radius_of_water_disection *
		    IVP_Inline_Math::save_acosf(edge_Q_center_os.real_length() / b) - 
		    edge_Q_center_os.real_length()*(length_p1p2 * 0.5f);
	    }
			
	    ball_projected_surface_content_under = surface_content_of_circle_segment - surface_content_of_ellipse_segment;
	    if (ball_projected_surface_content_under < buoyancy_eps) {  //safety check
		ball_projected_surface_content_under = 0.0f;
		goto calculate_impulse;
	    }
			
	    /*** calc ball_point_of_impulse ***/
			
	    //IVP_U_Float_Point direction;
	    //direction.set_multiple(&edge_O_center_os, -1.0f);  //turn direction
	    //direction.normize();

	    //calc mass center of circle segment
	    IVP_U_Float_Point center_of_circle_segment;
	    center_of_circle_segment.set(geom_center_os);
	    center_of_circle_segment.add_multiple(&direction_down, length_p1p2*length_p1p2*length_p1p2 / (12*surface_content_of_circle_segment)); //safety check for surface_content_of_circle_segment already above
			
	    //calc mass center of ellipse segment
	    IVP_U_Float_Point center_of_ellipse_segment;
	    IVP_DOUBLE s = 2.0f * IVP_Inline_Math::ivp_sqrtf(IVP_Inline_Math::fabsd(2.0f * height * b - height*height));
	    IVP_DOUBLE ellipse_arc = 2*IVP_Inline_Math::fast_asin(s / (2.0f*b))*b;
	    IVP_DOUBLE A = 0.5f * (ellipse_arc * b - s*(b - height));
	    if ( (b<buoyancy_eps) || (A<buoyancy_eps) ) { //safety check
		//if this is the case then surface_os and speed_plane_os are nearly orthogonal, so the projected ellipse has nearly vanished
		center_of_ellipse_segment.set(0.0f, 0.0f, 0.0f);
	    } else {
		center_of_ellipse_segment.set(&center_of_ellipse_os);
		center_of_ellipse_segment.add_multiple(&direction_down, s*s*s / (12.0f * A));
	    }

	    ball_point_of_impulse.set_multiple(&center_of_ellipse_segment, -surface_content_of_ellipse_segment);
	    ball_point_of_impulse.add_multiple(&center_of_circle_segment, surface_content_of_circle_segment);
	    ball_point_of_impulse.mult(1.0f / ball_projected_surface_content_under);
			
	}
		
    }
    break;
    case 2: { //center lies above the surface and speed_plane_os disects ball only above the surface
    case_2:
	//check in which direction the ball is heading
	IVP_DOUBLE factor = surface_os->dot_product(&speed_plane_os);
	if (factor < 0.0f) {
			
	    //compute ball_point_of_impulse
	    IVP_U_Float_Point projected_center_on_surface_os;
	    surface_os->proj_on_plane(geom_center_os, &projected_center_on_surface_os);
	    speed_plane_os.proj_on_plane(&projected_center_on_surface_os, &ball_point_of_impulse);
			
	    //compute ball_projected_surface_content_under
	    ball_projected_surface_content_under = IVP_Inline_Math::fabsd(factor*IVP_PI*(radius*radius - distance*distance));
			
	} else {
			
	    //no dampening needed
	    ball_projected_surface_content_under = 0.0f;
	    //ball_point_of_impulse.set(0.0f, 0.0f, 0.0f);
			
	}
    }
    break;
    case 3: { //center lies under the surface and speed_plane_os disects ball under and above the surface

	//check if surface_os and speed_plane_os are nearly parallel
	//if so, the we can switch to case 4
	IVP_U_Float_Point direction_up;
	speed_plane_os.proj_on_plane(surface_os, &direction_up);
	direction_up.mult(-1.0f);
	if (direction_up.quad_length() < buoyancy_eps) {
	    //if this is the case, then surface_os and speed_plane_os are nearly parallel, so we can switch to case 4
	    goto case_4;
	}
	direction_up.normize();
		
	IVP_U_Float_Point p1p2_os;
	p1p2_os.subtract(p2_os, p1_os);
		
	//calc point in the middle of the straight that results from the intersection between surface_os and speed_plane_os
	IVP_U_Float_Point Q_os;  //point between p1_os and p2_os
	Q_os.set(p1_os);
	Q_os.add_multiple(&p1p2_os, 0.5f);

	IVP_DOUBLE length_p1p2 = p1p2_os.real_length();
	if (length_p1p2 < buoyancy_eps) {  //check for safety reasons
	    //in this case Q lies on the surface of the ball
	    ball_projected_surface_content_under = IVP_PI * radius * radius;
	    ball_point_of_impulse.set(geom_center_os);
	    break;
	}
	//calc projected highest point O_os on arc of ellipse
	IVP_U_Float_Hesse p1p2_hesse_os;
	p1p2_hesse_os.set(&p1p2_os);
	p1p2_hesse_os.calc_hesse_val(&Q_os);
	p1p2_hesse_os.normize();
		
	IVP_U_Float_Point tmp_O1_os;
	IVP_U_Float_Point tmp_O2_os;
	//int erg =
	compute_disection_points_with_ball(surface_os, &p1p2_hesse_os, geom_center_os, radius, &tmp_O1_os, &tmp_O2_os);
	//	    IVP_ASSERT( erg == 1 );
	//decide which point is the right one and project it onto speed_plane_os => point O_os
	IVP_U_Float_Point tmp_O1O2_os;
	tmp_O1O2_os.subtract(&tmp_O2_os, &tmp_O1_os);
	IVP_U_Float_Point O_os;
	if (speed_plane_os.dot_product(&tmp_O1O2_os) >= 0.0f) {
	    speed_plane_os.proj_on_plane(&tmp_O1_os, &O_os);
	} else {
	    speed_plane_os.proj_on_plane(&tmp_O2_os, &O_os);
	}
		
	//compute center of whole ellipse
	IVP_U_Float_Point projected_center_on_surface_os;
	surface_os->proj_on_plane(geom_center_os, &projected_center_on_surface_os);
	IVP_U_Float_Point center_of_ellipse_os;
	speed_plane_os.proj_on_plane(&projected_center_on_surface_os, &center_of_ellipse_os);
		
		
	//check in which direction the ball is heading
	IVP_DOUBLE factor = surface_os->dot_product(&speed_plane_os);
	if (factor <= buoyancy_eps) {  //ball falls into the medium
			
	    /*** compute circle segment under surface ***/
			
	    IVP_U_Float_Point MQ_os;
	    MQ_os.subtract(&Q_os, geom_center_os);
	    IVP_DOUBLE h = radius - MQ_os.real_length();  //height of disected part of circle
	    IVP_DOUBLE arc = 2*IVP_Inline_Math::fast_asin(length_p1p2 / (2.0f * radius)) * radius;  //length of arc which is cut off speed_plane_os by surface_os
			
	    IVP_DOUBLE cut_off_part_above_surface = IVP_Inline_Math::fabsd(0.5f * (arc*radius - length_p1p2*(radius - h)));
	    IVP_DOUBLE surface_content_down = IVP_Inline_Math::fabsd(IVP_PI*radius*radius - cut_off_part_above_surface);

	    IVP_ASSERT(surface_content_down > buoyancy_eps);
			
	    IVP_U_Float_Point center_of_segment_os;  //center of part under surface

	    if (cut_off_part_above_surface < buoyancy_eps) { //safety check
		//nearly the whole surface to damp is in the medium
		ball_projected_surface_content_under = IVP_PI * radius * radius;
		ball_point_of_impulse.set(geom_center_os);
	    } else {
			
		if (factor > -buoyancy_eps) { //speed_plane_os is orthogonal on surface_os
		    IVP_U_Float_Point center_of_cut_off_part_above_surface;
		    center_of_cut_off_part_above_surface.set(geom_center_os);
		    center_of_cut_off_part_above_surface.add_multiple(surface_os, -(length_p1p2*length_p1p2*length_p1p2 / (12 * cut_off_part_above_surface)));
		    center_of_segment_os.set_multiple(&center_of_cut_off_part_above_surface, -cut_off_part_above_surface);
		    center_of_segment_os.add_multiple(geom_center_os, IVP_PI*radius*radius);
		    center_of_segment_os.mult( 1.0f / surface_content_down );
				
		    //if both surfaces are orthogonal, then nothing else has to be computed
		    ball_projected_surface_content_under = surface_content_down;
		    ball_point_of_impulse = center_of_segment_os;
		    goto calculate_impulse;
		} else {
		    //IVP_U_Float_Point direction_up;
		    //direction_up.subtract(&O_os, geom_center_os);
		    //direction_up.normize();
				
		    IVP_U_Float_Point center_of_cut_off_part_above_surface;
		    center_of_cut_off_part_above_surface.set(geom_center_os);
		    center_of_cut_off_part_above_surface.add_multiple(&direction_up, (length_p1p2*length_p1p2*length_p1p2) / (12 * cut_off_part_above_surface));
		    //compute center_of_segment_os by weighed surface contents
		    center_of_segment_os.set_multiple(&center_of_cut_off_part_above_surface, -cut_off_part_above_surface);
		    center_of_segment_os.add_multiple(geom_center_os, IVP_PI*radius*radius);
		    center_of_segment_os.mult( 1.0f / surface_content_down );
		}
	    }
			
			
	    /*** compute part of ellipse above surface ***/
			
	    //compute surface content of ellipse segment above surface
	    IVP_U_Float_Point edge_O_center_os;
	    edge_O_center_os.subtract(&center_of_ellipse_os, &O_os);
	    IVP_DOUBLE b = edge_O_center_os.real_length();
	    IVP_U_Float_Point edge_Q_center_os;
	    edge_Q_center_os.subtract(&center_of_ellipse_os, &Q_os);
	    IVP_DOUBLE radius_of_water_disection = IVP_Inline_Math::ivp_sqrtf(IVP_Inline_Math::fabsd(radius*radius - distance*distance));
	    IVP_DOUBLE height = IVP_Inline_Math::fabsd(b - edge_Q_center_os.real_length());

	    IVP_U_Float_Point center_of_ellipse_segment;
	    IVP_DOUBLE surface_content_up;
	    if (b < buoyancy_eps) { //safety check
		surface_content_up = 0.0f;
		center_of_ellipse_segment.set(0.0f, 0.0f, 0.0f);
	    } else {
		surface_content_up = b*radius_of_water_disection*IVP_Inline_Math::save_acosf(edge_Q_center_os.real_length() / b) - edge_Q_center_os.real_length()*(0.5f*length_p1p2);
		
		//compute center of ellipse segment above water
		IVP_DOUBLE s = 2 * IVP_Inline_Math::ivp_sqrtf(IVP_Inline_Math::fabsd(2*height*b - height*height));
		IVP_DOUBLE ellipse_arc = 2*IVP_Inline_Math::fast_asin( s/(2*b))*b;
		IVP_DOUBLE A = IVP_Inline_Math::fabsd(0.5f * (ellipse_arc * b - s*(b - height)));
		if (A < buoyancy_eps) {
		    surface_content_up = 0.0f;
		    center_of_ellipse_segment.set(0.0f, 0.0f, 0.0f);
		} else {
		    //edge_O_center_os.normize();
		    //edge_O_center_os.mult(-1);  //change direction
		    center_of_ellipse_segment.set(&center_of_ellipse_os);
		    //center_of_ellipse_segment.add_multiple(&edge_O_center_os, s*s*s / (12 * A));
		    center_of_ellipse_segment.add_multiple(&direction_up, s*s*s / (12 * A));
		}
	    }
			
			
	    /*** compute results ***/
			
	    //compute ball_projected_surface_content_under
	    ball_projected_surface_content_under = surface_content_up + surface_content_down;
	    if (ball_projected_surface_content_under < buoyancy_eps) {
		ball_projected_surface_content_under = 0.0f;
		ball_point_of_impulse.set(0.0f, 0.0f, 0.0f);
	    } else {
	    //compute ball_point_of_impulse
		ball_point_of_impulse.set_multiple(&center_of_segment_os, surface_content_down);
		ball_point_of_impulse.add_multiple(&center_of_ellipse_segment, surface_content_up);
		ball_point_of_impulse.mult(1.0f / ball_projected_surface_content_under);
	    }
			
	} else { //ball flies out of the water
			
	    //calc various ellipse specific values
	    IVP_U_Float_Point edge_O_center_os;
	    edge_O_center_os.subtract(&center_of_ellipse_os, &O_os);
	    IVP_DOUBLE b = edge_O_center_os.real_length();
	    IVP_U_Float_Point edge_Q_center_os;
	    edge_Q_center_os.subtract(&center_of_ellipse_os, &Q_os);
	    IVP_DOUBLE radius_of_water_disection = IVP_Inline_Math::ivp_sqrtf(IVP_Inline_Math::fabsd(radius*radius - distance*distance));
	    IVP_DOUBLE height = IVP_Inline_Math::fabsd(b - edge_Q_center_os.real_length());
	    //IVP_DOUBLE length_p1p2 = p1p2_os.real_length();
			
	    //calc segment of circle above water
	    IVP_U_Float_Point MQ_os;
	    MQ_os.subtract(&Q_os, geom_center_os);
	    IVP_DOUBLE h = radius - MQ_os.real_length();  //height of disected segment of circle
	    IVP_DOUBLE arc = 2*IVP_Inline_Math::fast_asin(length_p1p2 / (2.0f * radius)) * radius;  //length of arc of circle segment
	    IVP_DOUBLE surface_content_of_circle_segment = IVP_Inline_Math::fabsd(0.5f * (arc*radius - length_p1p2*(radius - h)));

	    if (surface_content_of_circle_segment < buoyancy_eps) {  //safety check
		//if this is the case then case 4 can be applied
		goto case_4;
	    }

	    //calc segment of ellipse above water
	    IVP_DOUBLE surface_content_of_ellipse_segment;
	    if (b < buoyancy_eps) { //safety check
		surface_content_of_ellipse_segment = 0.0f;
	    } else {
		surface_content_of_ellipse_segment = b*radius_of_water_disection*IVP_Inline_Math::save_acosf(edge_Q_center_os.real_length() / b) - edge_Q_center_os.real_length()*(length_p1p2 * 0.5f);
	    }
	    
	    IVP_DOUBLE part_ring_above_water = IVP_Inline_Math::fabsd(surface_content_of_circle_segment - surface_content_of_ellipse_segment);
			
	    //calc surface content under water (surface_content_circle - surface_content_ellipse - part_ring_above_water)
	    ball_projected_surface_content_under = IVP_Inline_Math::fabsd(IVP_PI*(radius*radius - radius_of_water_disection*b) - part_ring_above_water);
	    if (ball_projected_surface_content_under < buoyancy_eps) {  //safety check
		ball_projected_surface_content_under = 0.0f;
		goto calculate_impulse;
	    }
			
	    /*** calc ball_point_of_impulse ***/
	    //IVP_U_Float_Point direction;
	    //direction.set(&edge_O_center_os);
	    //direction.normize();
			
	    //calc mass center of circle segment above surface
	    IVP_U_Float_Point center_of_circle_segment;
	    center_of_circle_segment.set(geom_center_os);
	    center_of_circle_segment.add_multiple(&direction_up, length_p1p2*length_p1p2*length_p1p2 / (12*surface_content_of_circle_segment));  //safety check already above
			
	    //calc mass center of circle segment under surface
	    IVP_U_Float_Point center_of_circle_segment_under_surface;
	    center_of_circle_segment_under_surface.set_multiple(&center_of_circle_segment, -surface_content_of_circle_segment);
	    center_of_circle_segment_under_surface.add_multiple(geom_center_os, IVP_PI*radius*radius);
	    IVP_DOUBLE surface_content_of_circle_segment_under_surface = IVP_Inline_Math::fabsd(IVP_PI*radius*radius - surface_content_of_circle_segment);
	    if (surface_content_of_circle_segment_under_surface < buoyancy_eps) { //safety check
		surface_content_of_circle_segment_under_surface = 0.0f;
		center_of_circle_segment_under_surface.set(0.0f, 0.0f, 0.0f);
	    } else {
		center_of_circle_segment_under_surface.mult( 1.0f / surface_content_of_circle_segment_under_surface);
	    }
			
	    //calc mass center of ellipse segment
	    IVP_U_Float_Point center_of_ellipse_segment;
	    IVP_DOUBLE s = 2.0f * IVP_Inline_Math::ivp_sqrtf(IVP_Inline_Math::fabsd(2*height*b - height*height));
	    IVP_DOUBLE ellipse_arc = 2*IVP_Inline_Math::fast_asin(s / (2*b))*b;
	    IVP_DOUBLE A = 0.5f * (ellipse_arc * b - s*(b - height));
	    if ( (b<buoyancy_eps) || (A<buoyancy_eps) ) {  //safety check
		center_of_ellipse_segment.set(0.0f, 0.0f, 0.0f);
	    } else {
		center_of_ellipse_segment.set(&center_of_ellipse_os);
		center_of_ellipse_segment.add_multiple(&direction_up, s*s*s / (12 * A));
	    }
			
	    //calc mass center of ellipse segment under surface
	    IVP_U_Float_Point center_of_ellipse_segment_under_surface;
	    center_of_ellipse_segment_under_surface.set_multiple(&center_of_ellipse_segment, -surface_content_of_ellipse_segment);
	    center_of_ellipse_segment_under_surface.add_multiple(&center_of_ellipse_os, IVP_PI*radius_of_water_disection*b);
	    IVP_DOUBLE surface_content_of_ellipse_segment_under_surface = IVP_Inline_Math::fabsd(IVP_PI*radius_of_water_disection*b - surface_content_of_ellipse_segment);
	    if (surface_content_of_ellipse_segment_under_surface < buoyancy_eps) { //safety check
		center_of_ellipse_segment_under_surface.set(0.0f, 0.0f, 0.0f);
		surface_content_of_ellipse_segment_under_surface = 0.0f;
	    } else {
		center_of_ellipse_segment_under_surface.mult( 1.0f / surface_content_of_ellipse_segment_under_surface);
	    }
			
	    //calc ball_point_of_impulse
	    ball_point_of_impulse.set_multiple(&center_of_ellipse_segment_under_surface, -surface_content_of_ellipse_segment_under_surface);
	    ball_point_of_impulse.add_multiple(&center_of_circle_segment_under_surface, surface_content_of_circle_segment_under_surface);
	    ball_point_of_impulse.mult( 1.0f / ball_projected_surface_content_under );  //safety check above
	}
		
    }
    break;
    case 4: { //center lies under surface and speed_plane_os disects completely under surface
    case_4:
	
	//check in which direction the ball is heading
	IVP_DOUBLE factor = surface_os->dot_product(&speed_plane_os);
	if (factor <= 0.0f) {
			
	    ball_projected_surface_content_under = IVP_PI*radius*radius;  //surface content of speed_plane_os
	    ball_point_of_impulse.set(geom_center_os);
			
	} else {
			
	    //compute surface content of resulting ring
	    IVP_DOUBLE surface_content_circle = IVP_PI*radius*radius;
	    IVP_DOUBLE surface_content_ellipse = factor*IVP_PI*(radius*radius - distance*distance);
	    ball_projected_surface_content_under = IVP_Inline_Math::fabsd(surface_content_circle - surface_content_ellipse);

	    if (ball_projected_surface_content_under < buoyancy_eps) { //safety check
		ball_projected_surface_content_under = 0.0f;
		goto calculate_impulse;
	    }
			
	    //compute ball_point_of_impulse
	    IVP_U_Float_Point projected_center_on_surface_os;
	    surface_os->proj_on_plane(geom_center_os, &projected_center_on_surface_os);
	    IVP_U_Float_Point projected_center_on_ellipse_os;
	    speed_plane_os.proj_on_plane(&projected_center_on_surface_os, &projected_center_on_ellipse_os);
			
	    //compute center of resulting ring by weighed sum of both the center of the whole circle and the center of the ellipse
	    ball_point_of_impulse.set_multiple(&projected_center_on_ellipse_os, -surface_content_ellipse);
	    ball_point_of_impulse.add_multiple(geom_center_os, surface_content_circle);
	    ball_point_of_impulse.mult( 1.0f / ball_projected_surface_content_under );
			
	}
    }
    break;
    default: {
		
		
    }
    break;
    }

calculate_impulse:
    IVP_DOUBLE quad_length_of_speed = rel_speed_of_current_os.quad_length();
    if (quad_length_of_speed < buoyancy_eps) {
	quad_length_of_speed = 0.0f;
	rel_speed_of_current_os.set(0.0f, 1.0f, 0.0f);  //any value, is compensated by quad_length_of_speed=0
    }
    IVP_DOUBLE pressure_dampening_impulse = ball_projected_surface_content_under * quad_length_of_speed * pressure_damp_factor;
    //area content of ball where water flows around is approximated with the help of ball_projected_surface_content_under
    IVP_DOUBLE friction_dampening_impulse = (3.0f * ball_projected_surface_content_under) * quad_length_of_speed * friction_damp_factor;

    rel_speed_of_current_os.normize();
    ball_impulse_vector.set_multiple( &rel_speed_of_current_os, pressure_dampening_impulse + friction_dampening_impulse );
	
    //add the surface content that's under the water to the corresponding global variable
    object_visible_surface_content_under += ball_projected_surface_content_under;

    //calc the cross product between the impulse vector and the point of impulse and add it to the global variable
    IVP_U_Float_Point impulse_x_point;
    impulse_x_point.inline_calc_cross_product(&ball_point_of_impulse, &ball_impulse_vector);
    sum_impulse_x_point.add(&impulse_x_point);
  
    //calc the move-direction vector
    //setting it to zero value
    IVP_U_Float_Point move_direction_os; move_direction_os.set_to_zero();
  
    //calc cross product between impulse_vector and move-direction vector and add it to the global variable
    IVP_U_Float_Point impulse_x_movevector;
    impulse_x_movevector.inline_calc_cross_product(&move_direction_os, &ball_impulse_vector);
    sum_impulse_x_movevector.add(&impulse_x_movevector);
    
    //add impulse to global variable
    sum_impulse.add(&ball_impulse_vector);
}


void IVP_Buoyancy_Solver::compute_values_for_one_ball(const IVP_Real_Object *object,
						      const IVP_U_Float_Hesse *surface_os,
						      const IVP_U_Float_Point *rel_speed_of_current_os) {
	
    //fetch some ball related values
    IVP_Ball *ball = (IVP_Ball *) object;
	
#if 0
    IVP_U_Point non_float_geom_center_ws;
    IVP_U_Float_Point geom_center_ws;  //the mass(!!!!) center in world space
    ball->get_geom_center_world_space(&non_float_geom_center_ws);
    geom_center_ws.set(&non_float_geom_center_ws);
#endif
    IVP_U_Float_Point geom_center_os; geom_center_os.set_to_zero();
    IVP_FLOAT radius = ball->get_radius();  //get radius of ball
	
    //create the plane the speed has an impact on
    IVP_U_Float_Hesse speed_plane_os;
    IVP_FLOAT scalar_speed=rel_speed_of_current_os->quad_length();
    if(scalar_speed < P_DOUBLE_EPS) {
	speed_plane_os.set(0.0f,1.0f,0.0f);	
    } else {
	speed_plane_os.set(rel_speed_of_current_os);
    }
    speed_plane_os.calc_hesse_val(&geom_center_os);
    speed_plane_os.normize();
	
    //the disection points of the straight (which results from the disection of surface_ws and speed_plane_ws) and the ball
    IVP_U_Float_Point p1_os;
    IVP_U_Float_Point p2_os;
	
    //compute whether the ball is under or above the surface
    IVP_FLOAT distance = surface_os->get_dist(&geom_center_os);  //get distance of center to surface
    int decision = -1;
	
    if (distance <= 0.0f) { //center lies above surface
	if (IVP_Inline_Math::fabsd(distance) >= radius) { //ball is completely above surface
	    return;  //nothing to do
	} else {  //ball partly under the surface, center lies above
	    int location = compute_disection_points_with_ball(surface_os, &speed_plane_os, &geom_center_os, radius, &p1_os, &p2_os);
	    if (location) {
		decision = 1;  //speed_plane_os disects ball under and above the surface
	    } else {
		decision = 2;  //speed_plane_os disects ball above the surface
	    }
	}
    } else {  //center lies under surface
	if (distance >= radius) { //speed_plane_os and/or ball is completely under surface
	    decision = 0;
	} else {  //ball partly under the surface, center lies under
	    int location = compute_disection_points_with_ball(surface_os, &speed_plane_os, &geom_center_os, radius, &p1_os, &p2_os);
	    if (location) {
		decision = 3;  //speed_plane_os disects ball under and above the surface
	    } else {
		decision = 4;  //speed_plane_os disects completely under surface
	    }
	}
    }
	
#if 0
    P_IF(1) {
	IVP_U_Float_Point geom_center_ws;
	IVP_U_Float_Hesse surface_ws;
	{
	    IVP_Cache_Object *cache_object = object->get_cache_object();
	    cache_object->transform_vector_to_world_coords(&geom_center_os, &geom_center_ws);
	    cache_object->transform_vector_to_world_coords(surface_os, &surface_ws);
	    cache_object->remove_reference();
	}
	//draw debug vectors to indicate surface (dirty!)
	IVP_U_Point middle(geom_center_ws.k[0], -(surface_ws.hesse_val), geom_center_ws.k[2]);
	IVP_U_Point vec1(-5.0f, 0.0f, 0.0f);
	IVP_U_Point vec2(5.0f, 0.0f, 0.0f);
	IVP_U_Point vec3(0.0f, 0.0f, 5.0f);
	IVP_U_Point vec4(0.0f, 0.0f, -5.0f);
	environment->add_draw_vector(&middle,&vec1,"",2);
	environment->add_draw_vector(&middle,&vec2,"",2);
	environment->add_draw_vector(&middle,&vec3,"",2);
	environment->add_draw_vector(&middle,&vec4,"",2);
    }
#endif
	
	
    //compute buoyancy values
    compute_buoyancy_values_for_one_ball(decision,
					 distance,
					 radius,
					 surface_os,
					 &geom_center_os);
    
	
    //compute the volume centers and the volumes under surface_os for the pushing forces
    compute_dampening_values_for_one_ball(decision,
					  distance,
					  radius,
					  &geom_center_os,
					  rel_speed_of_current_os,
					  &speed_plane_os,
					  surface_os,
					  &p1_os,
					  &p2_os);

}


int IVP_Buoyancy_Solver::compute_disection_points_with_ball(const IVP_U_Float_Hesse *plane1_os,
							    const IVP_U_Float_Hesse *plane2_os,
							    const IVP_U_Float_Point *geom_center_os,
							    const IVP_FLOAT &radius,
							    IVP_U_Float_Point *p1_os, IVP_U_Float_Point *p2_os) {
	
    /*** convert plane2_ws from hesse form into point-direction form ***/
    IVP_U_Hesse tmp_plane2_os;
    tmp_plane2_os.set(plane2_os);
    tmp_plane2_os.hesse_val = plane2_os->hesse_val;
	
    IVP_U_Plain point_direction_form_of_plane2_os(&tmp_plane2_os);  //create point-direction form of speed_plane_ws
    
    IVP_U_Straight straight_os;
    IVP_U_Hesse non_float_plane1_os;  //IVP_U_Plain cannot handle IVP_U_Float_Points
    non_float_plane1_os.set(plane1_os);
    non_float_plane1_os.hesse_val = plane1_os->hesse_val;
	
    IVP_RETURN_TYPE is_parallel;
    is_parallel = point_direction_form_of_plane2_os.calc_intersect_with(&non_float_plane1_os, &straight_os);  //calc intersection of both planes => straight
    if (is_parallel == IVP_FAULT) {
	return 0;  //both planes are parallel, so no intersection exists between them
    }
	
    /*** calc intersection of straight with ball ***/
    IVP_U_Float_Point straight_start_point;
    straight_start_point.set(&(straight_os.start_point));  //start point of straight_os
    IVP_U_Float_Point straight_vec;
    straight_vec.set(&(straight_os.vec));  //vector of straight_os
    IVP_U_Float_Point T;
    T.subtract(&straight_start_point, geom_center_os);
	
    IVP_U_Point parameters;
    parameters.k[0] = straight_vec.k[0]*straight_vec.k[0] + straight_vec.k[1]*straight_vec.k[1] + straight_vec.k[2]*straight_vec.k[2];
    parameters.k[1] = 2*T.k[0]*straight_vec.k[0] + 2*T.k[1]*straight_vec.k[1] + 2*T.k[2]*straight_vec.k[2];
    parameters.k[2] = T.k[0]*T.k[0] +  T.k[1]*T.k[1] +  T.k[2]*T.k[2] - radius*radius;
	
    IVP_U_Point result;
    result.solve_quadratic_equation_accurate(&parameters); //solve the quadratic equation and store in result
	
    if (result.k[0] >= 0.0f) {  //solutions exist
	p1_os->set(&straight_start_point);
	p1_os->add_multiple(&straight_vec, result.k[1]);
		
	p2_os->set(&straight_start_point);
	p2_os->add_multiple(&straight_vec, result.k[2]);
		
	return 1;
    } else {  //no solution exists
	p1_os->set(0.0f, 0.0f, 0.0f);
	p2_os->set(0.0f, 0.0f, 0.0f);
		
	return 0;
    }
}


void IVP_Buoyancy_Solver::compute_values_for_one_polygon( IVP_Real_Object *object, const IVP_U_Float_Hesse *surface_os) {
	
    //initialize buoyancy variables with zero values
    //volume_under = volume_above = 0;
    volume_under = 0;
    volume_center_under.set_to_zero();
    //volume_center_above.set(0,0,0);
	
	
    IVP_U_Float_Point s_point;  //projected center point
    {
	IVP_IF(1){
	    IVP_DOUBLE leng=surface_os->real_length();
	    IVP_DOUBLE testval=IVP_Inline_Math::fabsd(leng-1.0f);
	    IVP_ASSERT( testval < 0.001f);
	}
	s_point.set_multiple(surface_os,-surface_os->hesse_val);  //projects center onto surface_os
    }
	
    IVP_U_BigVector<IVP_Compact_Ledge> object_ledges(256); //will contain all ledges belonging to "object"
    {	//initialize object_ledges with all ledges
	IVP_SurfaceManager *surman = controller_buoyancy->attacher_buoyancy->get_buoyancy_surface( object );
	surman->get_all_terminal_ledges(&object_ledges);
    }
    {
	int index;
	for (index = object_ledges.len()-1; index>=0; index--) {
	    IVP_Compact_Ledge *tmp_ledge = object_ledges.element_at(index);
	    compute_values_for_one_ledge(object, tmp_ledge, surface_os, &s_point);
	}
    }
    
    // finish the volume center computation
    if ( volume_under > buoyancy_eps )
	volume_center_under.mult( 0.25f / volume_under );
#if 0
    if (volume_above > buoyancy_eps )
	volume_center_above.mult( 1.0f / volume_above );
#endif
}




/*****************************************************************
*  compute the volume between a triangular surface and s_point  *
*****************************************************************/

IVP_FLOAT IVP_Buoyancy_Solver::compute_pyramid_volume( const IVP_U_Float_Point *p1,
						   const IVP_U_Float_Point *p2,
						   const IVP_U_Float_Point *p3,
						   const IVP_U_Float_Point *s_point) {
	
    IVP_U_Point hesse;
    hesse.inline_set_vert_to_area_defined_by_three_points(p1,p2,p3);  //compute orthogonal vector
	
    IVP_U_Float_Point v;
    v.subtract( p1, s_point );  //compute difference vector
    return (hesse.dot_product(&v)) * ( -1.0f / 6.0f );
}



void IVP_Buoyancy_Solver::compute_values_for_one_triangle(IVP_Real_Object *object,
							  const IVP_Compact_Triangle *triangle,
							  const IVP_U_Float_Hesse *surface_os,
							  const IVP_U_Float_Point *s_point,
							  const IVP_Compact_Ledge *current_ledge) {
	
    //fetch the points defining the triangle
	IVP_U_Float_Point three_points[3];
	
	three_points[0].set(triangle->get_edge(0)->get_start_point(current_ledge));
	three_points[1].set(triangle->get_edge(1)->get_start_point(current_ledge));
	three_points[2].set(triangle->get_edge(2)->get_start_point(current_ledge));

    const IVP_U_Float_Point *triangle_points[5];
    triangle_points[0] = &three_points[0]; //p0
    triangle_points[1] = &three_points[1];
    triangle_points[2] = &three_points[2];
    triangle_points[3] = &three_points[0];
    triangle_points[4] = &three_points[1];


    //check if triangle is under/above surface
    IVP_FLOAT distance[6];     //indicates distance of the 3 vertices of the triangle
    int sum_distance = 0;   //indicates the distance of the whole triangle: completely above/under surface, if 0 or 3
    int index_positiv, index_neg;

    for (int j=2; j>=0; j--) {
	IVP_FLOAT x  = surface_os->get_dist(triangle_points[j]);
	if (x < 0.0f) {
	    index_neg = j;
	    sum_distance++;
	    //decrease distance to avoid division through zero
	    distance[j] = x - buoyancy_eps;
	    distance[j+3] = distance[j];
	} else {
	    index_positiv = j;
	    //increase distance to avoid division trough zero
	    distance[j] = x + buoyancy_eps;
	    distance[j+3] = distance[j];
	}
    }
	
    compute_volumes_and_centers_for_one_pyramid(object,
						triangle_points,
						distance,
						sum_distance,
						index_positiv,
						index_neg,
						s_point);
	
    compute_rotation_and_translation_values_for_one_triangle(object,
							     triangle,
							     triangle_points,
							     current_ledge,
							     distance,
							     sum_distance,
							     index_positiv,
							     index_neg);
	
}



void IVP_Buoyancy_Solver::compute_volumes_and_centers_for_one_pyramid(IVP_Real_Object *object,
								      const IVP_U_Float_Point *triangle_points[5],
								      const IVP_FLOAT distance[6],
								      const int &decision,
								      const int &index_positiv,
								      const int &index_neg,
								      const IVP_U_Float_Point *s_point) {
	
    
    switch (decision){
    case 0: { //triangle is completely under the surface
		
	IVP_DOUBLE volume_pyramid_under = compute_pyramid_volume( triangle_points[0], triangle_points[1], triangle_points[2], s_point );
	volume_under += volume_pyramid_under;
		
	//compute the volume center of the pyramid
	IVP_U_Float_Point volume_center_pyramid;	//volume center of the pyramid
	volume_center_pyramid.add(triangle_points[0], triangle_points[1]);
	volume_center_pyramid.add(triangle_points[2]);
	volume_center_pyramid.add(s_point);
	volume_center_pyramid.mult( volume_pyramid_under); //compute the volume center and weigh it with the corresponding volume
		
	//save results to class variables
	volume_center_under.add(&volume_center_pyramid);
    }
    break;
#if 0
    case 3: {//triangle is completely above the surface
		
	volume_pyramid_above = compute_pyramid_volume( triangle_points[0], triangle_points[1], triangle_points[2], &s_point );
	volume_above += volume_pyramid_above;
		
	//compute the volume center of the pyramid
	volume_center_pyramid.set(triangle_points[0]);
	volume_center_pyramid.add(triangle_points[1]);
	volume_center_pyramid.add(triangle_points[2]);
	volume_center_pyramid.add(&s_point);
	volume_center_pyramid.mult( 0.25f  * volume_pyramid_above);  //weight the volume center of this part of the object with its volume
		
	//save results to class variables
	volume_center_above.add(&volume_center_pyramid);
    }
    break;
#endif
    case 1: { //the vertex with index "index_neg" is above the surface

	//compute the first disection point on the surface 
	IVP_DOUBLE factor1 = -distance[index_neg] / (distance[index_neg+1] - distance[index_neg]);  //division by zero not possible, because distance[index_neg] < 0 and buoyancy_eps is added to each
		
	//compute the second disection point on the surface
	IVP_DOUBLE factor2 = -distance[index_neg] / (distance[index_neg+2] - distance[index_neg]);  //division by zero not possible, because distance[index_neg] < 0 and buoyancy_eps is added to each
		
	IVP_IF(1){
	    IVP_U_Float_Point sp1, sp2;
	    IVP_U_Point sp1_ws, sp2_ws;
	    sp1.set_interpolate(triangle_points[index_neg], triangle_points[index_neg+1], factor1);
	    sp2.set_interpolate(triangle_points[index_neg], triangle_points[index_neg+2], factor2);
			
	    IVP_Cache_Object *cache_object = object->get_cache_object();  //needed to transform coordinates into other coord. systems
	    cache_object->transform_position_to_world_coords(&sp1, &sp1_ws);
	    cache_object->transform_position_to_world_coords(&sp2, &sp2_ws);
	    cache_object->remove_reference();
			
			
	    IVP_U_Float_Point edge12;
	    edge12.subtract(&sp1_ws, &sp2_ws);
	    IVP_U_Point tmp_sp2_ws;
	    tmp_sp2_ws.set(&sp2_ws);
	    environment->add_draw_vector(&tmp_sp2_ws, &edge12,"",4);
	}
		
	       		
	/*** compute the volumes ***/
		
	//compute volume of whole pyramid triangle is part of
	IVP_DOUBLE volume_pyramid = compute_pyramid_volume( triangle_points[0], triangle_points[1], triangle_points[2], s_point );
		
	//compute volume of pyramid above the surface
	IVP_DOUBLE volume_pyramid_above = volume_pyramid * factor1 * factor2;
		
	//save results to class variables
	volume_under += (volume_pyramid - volume_pyramid_above);
		
		
	/*** compute the volume centers ***/
		
	//compute the area center of the whole triangle (Ce = (A+B+C)/3)
	IVP_U_Float_Point area_center_total;	        //area center of the whole triangle
	area_center_total.add(triangle_points[0], triangle_points[1]);
	area_center_total.add(triangle_points[2]);
		
	//compute the area center of the cut-off triangle above the surface (Ce2 = (SP1+SP2+C)/3)
	IVP_U_Float_Point area_center_part_above;	//area center of the triangle's part above the surface
	area_center_part_above.set_multiple(triangle_points[index_neg], 3.0f - factor1-factor2);
	area_center_part_above.add_multiple(triangle_points[index_neg+1], factor1);
	area_center_part_above.add_multiple(triangle_points[index_neg+2], factor2);
		
		
	//compute the area center of the cut-off quadrangle under the surface
	//Ce1 = (C - Ce2 * k) / (1-k)
	IVP_DOUBLE k = factor1 * factor2;		//factor the area center is moved
	IVP_U_Float_Point area_center_part_under;	//area center of the triangle's part under the surface
	area_center_part_above.mult(k);
	area_center_part_under.set(&area_center_total);
	area_center_part_under.subtract(&area_center_part_above);
	if ((1.0f-k) < buoyancy_eps) {  //both other vertices are exactly on the surface, so there is no volume under the surface
	    volume_under = 0.0f;
	    volume_center_under.set(0.0f, 0.0f, 0.0f);
	    return;
	}
	area_center_part_under.mult( 1.0f / (1.0f-k));
		
	//compute the volume center of the part under the surface from area_center_part_under
	area_center_part_under.add(s_point);
		
	area_center_part_under.mult(volume_pyramid - volume_pyramid_above);
	volume_center_under.add(&area_center_part_under);
    }
    break;
    case 2: { //two vertices are above the surface, the vertex with index "index_positiv" is under the surface
		
		
	//compute the first disection point on the surface
	IVP_DOUBLE factor1 = distance[index_positiv] / (distance[index_positiv] - distance[index_positiv+1]);  //division by zero not possible, because distance[index_positiv+1] < 0 and buoyancy_eps is added to each
		
	//compute the second disection point on the surface
	IVP_DOUBLE factor2 = distance[index_positiv] / (distance[index_positiv] - distance[index_positiv+2]);  //division by zero not possible, because distance[index_positiv+2] < 0 and buoyancy_eps is added to each
		
		
	IVP_IF(1){
	    //for debugging reasons - begin
	    IVP_U_Float_Point sp1, sp2;
	    IVP_U_Point sp1_ws, sp2_ws;
	    sp1.set_interpolate(triangle_points[index_positiv], triangle_points[index_positiv+1], factor1);
	    sp2.set_interpolate(triangle_points[index_positiv], triangle_points[index_positiv+2], factor2);
	    IVP_Cache_Object *cache_object = object->get_cache_object();  //needed to transform coordinates into other coord. systems
	    cache_object->transform_position_to_world_coords(&sp1, &sp1_ws);
	    cache_object->transform_position_to_world_coords(&sp2, &sp2_ws);
	    cache_object->remove_reference();
			
	    IVP_U_Float_Point edge12;
	    edge12.subtract(&sp1_ws, &sp2_ws);
	    IVP_U_Point tmp_sp2_ws;
	    tmp_sp2_ws.set(&sp2_ws);
	    environment->add_draw_vector(&tmp_sp2_ws, &edge12,"",4);
	    //for debugging reasons - end
	}
		
	//compute volume of whole pyramid triangle is part of
	IVP_DOUBLE volume_pyramid = compute_pyramid_volume( triangle_points[0], triangle_points[1], triangle_points[2], s_point );
		
	//compute volume of pyramid under the surface
	IVP_DOUBLE volume_pyramid_under = factor1 * factor2 * volume_pyramid;
		
	volume_under += volume_pyramid_under;
		
		
	/*** compute the area centers ***/
		
	//compute the area center of the cut-off triangle under the surface (Ce2 = (SP1+SP2+C)/3)
	IVP_U_Float_Point area_center_part_under;
	area_center_part_under.set_multiple(triangle_points[index_positiv], 3.0f-factor1-factor2);
	area_center_part_under.add_multiple(triangle_points[index_positiv+1], factor1);
	area_center_part_under.add_multiple(triangle_points[index_positiv+2], factor2);
		
	//compute the volume center of the pyramid under the surface from area_center_part_under
	area_center_part_under.add(s_point);
		
	//save results to class variables
	area_center_part_under.mult(volume_pyramid_under); //weight the volume center of this part of the object with its volume
	volume_center_under.add(&area_center_part_under);
    }
    break;
    default:
	break;
    }
	
}


void IVP_Buoyancy_Solver::compute_rotation_and_translation_values_for_one_triangle(IVP_Real_Object *object,
										   const IVP_Compact_Triangle *current_triangle,
										   const IVP_U_Float_Point *triangle_points[5],
										   const IVP_Compact_Ledge *current_ledge,
										   const IVP_FLOAT distance[6],
										   const int &decision,
										   const int &index_positiv,
										   const int &index_neg) {
	
	
	
    //declare some common variables
    IVP_U_Float_Point area_center_part_under_os;
    IVP_FLOAT triangle_surface_under_factor;
    
    switch (decision) {
    case 0: { //current_triangle is completely under the surface
		
	//compute the area center of the current_triangle
	area_center_part_under_os.add(triangle_points[0], triangle_points[1]);
	area_center_part_under_os.add(triangle_points[2]);
	area_center_part_under_os.mult( 1.0f/3.0f );
		
	triangle_surface_under_factor = 1.0f;
		
	break; 
    }
		
    case 1: { //the vertex with index "index_neg" is above the surface
		
	//compute the relation factor between the vertex above the surface and the first one under the surface
	IVP_DOUBLE factor1 = -distance[index_neg] / (distance[index_neg+1] - distance[index_neg]);  //division by zero not possible, because distance[index_neg] < 0 and buoyancy_eps is added to each
		
	//compute the relation factor between the vertex above the surface and the second one under the surface
	IVP_DOUBLE factor2 = -distance[index_neg] / (distance[index_neg+2] - distance[index_neg]);  //division by zero not possible, because distance[index_neg] < 0 and buoyancy_eps is added to each
		
	//compute the area center of the current_triangle
	IVP_U_Float_Point area_center_of_triangle_os;
	area_center_of_triangle_os.add(triangle_points[0], triangle_points[1]);
	area_center_of_triangle_os.add(triangle_points[2]);
	area_center_of_triangle_os.mult( 1.0f/3.0f );
		
	//compute the area center of the cut-off triangle above the surface (Ce2 = (SP1+SP2+C)/3)
	IVP_U_Float_Point area_center_part_above_os;       //area center of the triangle's part above the surface
	area_center_part_above_os.set_multiple(triangle_points[index_neg], (3.0f-factor1-factor2) * (1.0f/3.0f));
	area_center_part_above_os.add_multiple(triangle_points[index_neg+1], factor1 * (1.0f/3.0f));
	area_center_part_above_os.add_multiple(triangle_points[index_neg+2], factor2 * (1.0f/3.0f));
		
	//compute the area center of the cut-off quadrangle under the surface
	//Ce1 = (C - Ce2 * k) / (1-k)
	IVP_DOUBLE k = factor1 * factor2;           //factor the area center is moved
		
	area_center_part_above_os.mult(k);
	area_center_part_under_os.set(&area_center_of_triangle_os);
	area_center_part_under_os.subtract(&area_center_part_above_os);
	
	area_center_part_under_os.mult( 1.0f / (1.0f-k));
	if ((1.0f-k) < buoyancy_eps) {  //both other vertices are exactly on the surface, so there is nothing to damp
	    return;
	}
	triangle_surface_under_factor = 1.0f - k;
		
	break;
    }
		
    case 2: { //two vertices are above the surface, the vertex with index "index_positiv" is under the surface
		
	//compute the relation factor between the vertex under the surface and the first one above the surface
	IVP_DOUBLE factor1 = distance[index_positiv] / (distance[index_positiv] - distance[index_positiv+1]);  //division by zero not possible, because distance[index_positiv+1] < 0 and buoyancy_eps is added to each
		
	//compute the relation factor between the vertex under the surface and the first one above the surface
	IVP_DOUBLE factor2 = distance[index_positiv] / (distance[index_positiv] - distance[index_positiv+2]);  //division by zero not possible, because distance[index_positiv+2] < 0 and buoyancy_eps is added to each
		
	//compute the area center of the cut-off triangle under the surface (Ce2 = (SP1+SP2+C)/3)
	area_center_part_under_os.set_multiple(triangle_points[index_positiv], (3.0f-factor1-factor2) * (1.0f/ 3.0f));
	area_center_part_under_os.add_multiple(triangle_points[index_positiv+1], factor1 * (1.0f/3.0f));
	area_center_part_under_os.add_multiple(triangle_points[index_positiv+2], factor2 * (1.0f/3.0f));
		
	triangle_surface_under_factor = factor1*factor2;
		
	break;
    }
		
    default:
	return;
    }
    
	
    {
	// compute normal_os of triangle
	const IVP_Compact_Edge *edge = current_triangle->get_first_edge();
	IVP_U_Float_Point normal_os;
	IVP_Compact_Ledge_Solver::calc_hesse_vec_object_not_normized(edge, current_ledge, &normal_os);
		
	if (normal_os.quad_length() < buoyancy_eps) {  //savety check
	    //if this is the case then it is better not to damp this triangle
	    return;
	}
		
	//get surface speed of the object at the area center of the part under the surface
	IVP_U_Float_Point speed_of_object_os;
	ivp_core_get_surface_speed_os(core,object, &area_center_part_under_os, &speed_of_object_os);
		
	//calc the relative speed of the medium
	IVP_U_Float_Point rel_speed_of_current_os;
	rel_speed_of_current_os.subtract( &resulting_speed_of_current_os, &speed_of_object_os);
		
	//get quad_length of relative speed, needed later
	IVP_FLOAT quad_length_of_speed = rel_speed_of_current_os.quad_length();
	if (quad_length_of_speed < buoyancy_eps) {  //savety check
	    //better no dampening of this triangle
	    quad_length_of_speed = 0.0f;
	    rel_speed_of_current_os.set(0.0f, 1.0f, 0.0f);
	}

		
	IVP_FLOAT surface_factor_check = normal_os.dot_product( &rel_speed_of_current_os );
	//cancel calculation of triangle if turned away from the current
	if (surface_factor_check > buoyancy_eps) {
	    return;
	}
	
	rel_speed_of_current_os.normize();

	IVP_U_Float_Point normized_normal_os;
	normized_normal_os.set(&normal_os);
	IVP_DOUBLE len_of_unscaled_normal = normized_normal_os.real_length_plus_normize();
	IVP_FLOAT surface_factor = normized_normal_os.dot_product( &rel_speed_of_current_os );
		
	//calc the surface content of the whole triangle
	IVP_FLOAT triangle_surface_content = 0.5f * len_of_unscaled_normal;

	//calculate the projected surface content
	IVP_FLOAT visible_surface_content_under = -surface_factor * triangle_surface_under_factor * triangle_surface_content;
 
		
	IVP_DOUBLE impulse;
	//impulse = (visible_surface_content_under * quad_length_of_speed * medium_damp_factor);

	//calculate the pressure dampening
	impulse = (visible_surface_content_under * quad_length_of_speed * pressure_damp_factor);
	//calculate the friction dampening
	impulse += (triangle_surface_content * quad_length_of_speed * friction_damp_factor);
	IVP_U_Float_Point impulse_vector;

	if (simulate_wing_behavior) {
	    impulse_vector.set_multiple( &normized_normal_os, -impulse );
	} else {
	    impulse_vector.set_multiple( &rel_speed_of_current_os, impulse );
	}
		
	IVP_IF(0) {
	    IVP_U_Point center_of_triangle_ws;
	    IVP_U_Float_Point impulse_vector_ws;
	    {
		IVP_Cache_Object *cache_object = object->get_cache_object();
		cache_object->transform_position_to_world_coords(&area_center_part_under_os, &center_of_triangle_ws);
		cache_object->transform_vector_to_world_coords(&impulse_vector, &impulse_vector_ws);
		cache_object->remove_reference();
	    }
	    environment->add_draw_vector(&center_of_triangle_ws, &impulse_vector_ws,"",2);
	}
		
	//add surface content, which is under the water, to the corresponding global variable
	object_visible_surface_content_under += IVP_Inline_Math::fabsd(visible_surface_content_under);
		
	//calc cross product between impulse vector and area center of triangle and add it to the global variable
	IVP_U_Float_Point impulse_x_point;
	impulse_x_point.inline_calc_cross_product(&area_center_part_under_os, &impulse_vector);
	sum_impulse_x_point.add(&impulse_x_point);
		
		
	//calc the move-direction vector
	IVP_U_Float_Point move_direction_os;
	move_direction_os.set_orthogonal_part(&rel_speed_of_current_os, &normized_normal_os);
		
		//calc cross product between impulse_vector and move-direction vector and add it to the global variable
	IVP_U_Float_Point impulse_x_movevector;
	impulse_x_movevector.inline_calc_cross_product(&move_direction_os, &impulse_vector);
	sum_impulse_x_movevector.add(&impulse_x_movevector);
		
	//add impulse to global variable
	sum_impulse.add(&impulse_vector);
    }
	
    
	
}


void IVP_Buoyancy_Solver::compute_values_for_one_ledge(IVP_Real_Object *object,
						       const IVP_Compact_Ledge *current_ledge,
						       const IVP_U_Float_Hesse *surface_os,
						       const IVP_U_Float_Point *s_point) {
	
    int nr_of_triangles = current_ledge->get_n_triangles();  //fetch number of triangles in current_ledge
    const IVP_Compact_Triangle *triangle = current_ledge->get_first_triangle();  //fetch first triangle in current_ledge
	
    for (int i=nr_of_triangles; --i >= 0; triangle = triangle->get_next_tri()) {  //fetch next triangle of ledge
	compute_values_for_one_triangle(object, triangle, surface_os, s_point, current_ledge);
    }
}
