// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#if defined(LINUX) || defined(SUN) || (defined(__MWERKS__) && defined(__POWERPC__))
#   include <alloca.h>
#endif

#if !defined(WIN32)
#     pragma implementation "ivp_constraint_local.hxx"
#endif


#include <ivp_great_matrix.hxx>
#include <ivp_core_macros.hxx>

#include <ivp_template_constraint.hxx>
#include "ivp_constraint_local.hxx"

// if set, constraints pushes at the same point in ws
//#define PUSH_SAMEPOINT
#define IVP_CONSTRAINT_BORDER_SOFTEN  /* if defined, a heuristic is activated that reduces the springy behavior of limit */

// !!! Attention !!!
//                          +----------+
// m_as_f_bs describes      | bs in as |
//                          +----------+
// m_as_f_bs.get_position() is the center of bs in as
// the first column is the x-axes of bs in as
// the first row is the x-axes of as in bs

// Debug-Colors:
// 0 black
// 1 red
// 2 green
// 3 blue
// 4 magenta
// 5 yellow
// 6 cyan
// 7 orange
// 8 white

IVP_Constraint_Local_Anchor::IVP_Constraint_Local_Anchor() {
    rot = NULL;
}

IVP_Constraint_Local_Anchor::~IVP_Constraint_Local_Anchor() {
    //if (rot) delete rot;
}

IVP_Constraint_Local::IVP_Constraint_Local() {
    is_enabled = IVP_FALSE;
    this->activate();
}

IVP_Constraint_Local::IVP_Constraint_Local(const IVP_Template_Constraint &tmpl) {
    init(tmpl);
    this->activate();
}

void IVP_Constraint_Local::sort_translation_mapping() {
    short pos = 0;
    {
	for (short i = 0; i < 3; i++) {
	    if (fixed[i] == IVP_CONSTRAINT_AXIS_FIXED) {
		mapping_uRfs_f_Rfs.k[pos++] = (IVP_COORDINATE_INDEX) i;
	    }
	}
    }
    {
	for (short i = 0; i < 3; i++) {
	    if (fixed[i] == IVP_CONSTRAINT_AXIS_LIMITED) {
		mapping_uRfs_f_Rfs.k[pos++] = (IVP_COORDINATE_INDEX) i;
	    }
	}
    }
    {
	for (short i = 0; i < 3; i++) {
	    if (fixed[i] == IVP_CONSTRAINT_AXIS_FREE) {
		mapping_uRfs_f_Rfs.k[pos++] = (IVP_COORDINATE_INDEX) i;
	    }
	}
    }
    fixedtrans_dim = limitedtrans_dim = 0;
    if (fixed[0] == IVP_CONSTRAINT_AXIS_FIXED) fixedtrans_dim++;
    if (fixed[0] & IVP_CONSTRAINT_AXIS_LIMITED) limitedtrans_dim++;
    if (fixed[1] == IVP_CONSTRAINT_AXIS_FIXED) fixedtrans_dim++;
    if (fixed[1] & IVP_CONSTRAINT_AXIS_LIMITED) limitedtrans_dim++;
    if (fixed[2] == IVP_CONSTRAINT_AXIS_FIXED) fixedtrans_dim++;
    if (fixed[2] & IVP_CONSTRAINT_AXIS_LIMITED) limitedtrans_dim++;
    matrix_size = fixedtrans_dim + fixedrot_dim + limitedtrans_dim + limitedrot_dim;
}

void IVP_Constraint_Local::sort_rotation_mapping() {
    int pos = 0;
    {
	for (int i = 0; i < 3; i++) {
	    if (fixed[i + 3] == IVP_CONSTRAINT_AXIS_FIXED) {
		mapping_uRrs_f_Rrs.k[pos++] = (IVP_COORDINATE_INDEX) i;
	    }
	}
    }
    {
	for (int i = 0; i < 3; i++) {
	    if (fixed[i + 3] == IVP_CONSTRAINT_AXIS_LIMITED) {
		mapping_uRrs_f_Rrs.k[pos++] = (IVP_COORDINATE_INDEX) i;
	    }
	}
    }
    {
	for (int i = 0; i < 3; i++) {
	    if (fixed[i + 3] == IVP_CONSTRAINT_AXIS_FREE) {
		mapping_uRrs_f_Rrs.k[pos++] = (IVP_COORDINATE_INDEX) i;
	    }
	}
    }
    fixedrot_dim = limitedrot_dim = 0;
    if (fixed[3] == IVP_CONSTRAINT_AXIS_FIXED) fixedrot_dim++;
    if (fixed[3] & IVP_CONSTRAINT_AXIS_LIMITED) limitedrot_dim++;
    if (fixed[4] == IVP_CONSTRAINT_AXIS_FIXED) fixedrot_dim++;
    if (fixed[4] & IVP_CONSTRAINT_AXIS_LIMITED) limitedrot_dim++;
    if (fixed[5] == IVP_CONSTRAINT_AXIS_FIXED) fixedrot_dim++;
    if (fixed[5] & IVP_CONSTRAINT_AXIS_LIMITED) limitedrot_dim++;
    matrix_size = fixedtrans_dim + fixedrot_dim + limitedtrans_dim + limitedrot_dim;
}

void IVP_Constraint_Local::init(const IVP_Template_Constraint &tmpl) {
    norm = IVP_NORM_MAXIMUM;

    IVP_Real_Object *objectR = tmpl.objectR;
    IVP_Real_Object *objectA = tmpl.objectA;
    if (objectR && !objectR->get_core()->physical_unmoveable)
	cores_of_constraint_system.add(objectR->get_core());
    if (objectA && !objectA->get_core()->physical_unmoveable)
	cores_of_constraint_system.add(objectA->get_core());
    if (tmpl.m_Aos_f_Afs) (objectR ? objectR : objectA)->ensure_in_simulation();
    IVP_U_Matrix m_ws_f_Ros; if (objectR) objectR->get_m_world_f_object_AT(&m_ws_f_Ros); else m_ws_f_Ros.init();
    IVP_U_Matrix m_ws_f_Aos; if (objectA) objectA->get_m_world_f_object_AT(&m_ws_f_Aos); else m_ws_f_Aos.init();
    IVP_U_Matrix m_Ros_f_Aos; m_ws_f_Ros.mimult4(&m_ws_f_Aos, &m_Ros_f_Aos);
    IVP_U_Matrix m_Rcs_f_Ros; if (objectR) objectR->calc_m_core_f_object(&m_Rcs_f_Ros); else m_Rcs_f_Ros.init();
    IVP_U_Matrix m_Acs_f_Aos; if (objectA) objectA->calc_m_core_f_object(&m_Acs_f_Aos); else m_Acs_f_Aos.init();
    IVP_U_Matrix m_Rcs_f_Aos; m_Rcs_f_Ros.mmult4(&m_Ros_f_Aos, &m_Rcs_f_Aos);
    IVP_U_Matrix m_Rcs_f_Acs; m_Rcs_f_Aos.mi2mult4(&m_Acs_f_Aos, &m_Rcs_f_Acs);
    
    //m_Ros_f_Rfs->vimult4(&_centerT_Ros, &center_trans_Rfs);
    //m_Ros_f_Rfs->vimult3(&_centerR_Ros, &center_rot_Rfs);
    
    if (tmpl.m_Ros_f_Rfs) {
	m_Rcs_f_Ros.mmult4(tmpl.m_Ros_f_Rfs, &m_Rfs_f_Rcs); m_Rfs_f_Rcs.real_invert();
    } else {
	m_Rfs_f_Rcs.set_matrix(&m_Rcs_f_Ros);
	m_Rfs_f_Rcs.real_invert();
    }
    if (tmpl.m_Aos_f_Afs) {
	m_Acs_f_Aos.mmult4(tmpl.m_Aos_f_Afs, &m_Afs_f_Acs); m_Afs_f_Acs.real_invert();
    } else { // Afs is Rfs
	m_Rfs_f_Rcs.mmult4(&m_Rcs_f_Acs, &m_Afs_f_Acs);
    }
    if (tmpl.m_Ros_f_Rrs) {
	m_Rfs_f_Rcs.rot = new IVP_U_Matrix3;
	m_Rcs_f_Ros.mmult3(tmpl.m_Ros_f_Rrs, m_Rfs_f_Rcs.rot); m_Rfs_f_Rcs.rot->transpose3();
	IVP_U_Matrix3 m_fs_f_rs; m_Rfs_f_Rcs.mi2mult3(m_Rfs_f_Rcs.rot, &m_fs_f_rs);
	m_Afs_f_Acs.rot = new IVP_U_Matrix3;
        m_fs_f_rs.mimult3(&m_Afs_f_Acs, m_Afs_f_Acs.rot);
    } else {
	m_Rfs_f_Rcs.rot = NULL;
	m_Afs_f_Acs.rot = NULL;
    }
    
    m_Rfs_f_Rcs.object = objectR;
    m_Afs_f_Acs.object = objectA;

	
    fixed[0] = tmpl.axis_type[0];
    fixed[1] = tmpl.axis_type[1];
    fixed[2] = tmpl.axis_type[2];
    fixed[3] = tmpl.axis_type[3];
    fixed[4] = tmpl.axis_type[4];
    fixed[5] = tmpl.axis_type[5];

    fixedtrans_dim = fixedrot_dim = limitedtrans_dim = limitedrot_dim = 0;
    sort_translation_mapping();
    sort_rotation_mapping();
    
    force_factor = tmpl.force_factor; // Ausgleichsrate pro PSI-Zeiteinheit
    damp_factor_div_force = tmpl.damp_factor / force_factor; // Ausgleichsrate pro PSI-Zeiteinheit

    limited_axis_stiffness = tmpl.limited_axis_stiffness;
    IVP_BOOL use_force = IVP_FALSE;
    for (int i = 0; i < 6; i++){
	borderright_Rfs[i]      = tmpl.borderright_Rfs[i];
	borderleft_Rfs[i]       = tmpl.borderleft_Rfs[i];
	
	if (tmpl.maximpulse_type[i] != IVP_CFE_NONE){
	    use_force = IVP_TRUE;
	}
    }

    
    if (use_force) { // luke
	maxforce = new IVP_Constraint_Local_MaxImpulse();
	for (short i = 0; i < 6; i++) {
	    maxforce->halfimpulse[i] = tmpl.maximpulse[i];
	    maxforce->type[i] = tmpl.maximpulse_type[i];
	}
    } else {
	maxforce = NULL;
    }
    
    is_enabled = IVP_FALSE;
}

IVP_Constraint_Local::~IVP_Constraint_Local() {
    if (maxforce) P_DELETE( maxforce);
    if (m_Rfs_f_Rcs.rot) P_DELETE( m_Rfs_f_Rcs.rot);
    if (m_Afs_f_Acs.rot) P_DELETE( m_Afs_f_Acs.rot);
}

void IVP_Constraint_Local::core_is_going_to_be_deleted_event(IVP_Core *core){
    IVP_USE(core);
    P_DELETE_THIS(this);
}

IVP_Real_Object *IVP_Constraint_Local::get_objectR() {
    return m_Rfs_f_Rcs.object;
}

IVP_Real_Object *IVP_Constraint_Local::get_objectA() {
    return m_Afs_f_Acs.object;
}


void IVP_Constraint_Local::constraint_changed() {

}

// A 700MHz prozessor needs (10/7) ns per cycle
// A ballsocket constraint on a rope need
// 13860,44 ns on redbaron before optimization
// 9990,111 ns on redbaron before optimization
// 89,68650 ns on maul before optimization
// Calculates the impulses which are needed to relaxe the constraint completely till next PSI
void IVP_Constraint_Local::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> * /*core_list*/){
    IVP_FLOAT d_time = es->delta_time;

    IVP_Core *coreR = m_Rfs_f_Rcs.object ? m_Rfs_f_Rcs.object->get_core() : NULL;
    IVP_Core *coreA = m_Afs_f_Acs.object ? m_Afs_f_Acs.object->get_core() : NULL;
    IVP_ASSERT(!coreR || coreR->physical_unmoveable || coreR->movement_state != IVP_MT_NOT_SIM);
    IVP_ASSERT(!coreA || coreA->physical_unmoveable || coreA->movement_state != IVP_MT_NOT_SIM);

    // fs Rfs Afs ws Rcs Acs
    IVP_U_Matrix m_identity; m_identity.init();
    // Note: Optimization: use  m_Rcs_f_Acs
    // can mmults be avoided ??

    /********************************************************************************
     *	Description:	calculate the translation system 
     *  Output:         m_fs_f_Acs and m_fs_f_Rcs
     ********************************************************************************/
    const IVP_U_Matrix &m_ws_f_Rcs = coreR ? *coreR->get_m_world_f_core_PSI() : m_identity;
    const IVP_U_Matrix &m_ws_f_Acs = coreA ? *coreA->get_m_world_f_core_PSI() : m_identity;

    IVP_U_Matrix3 &m_fs_f_Rcs = m_Rfs_f_Rcs;
    IVP_U_Matrix3 m_fs_f_ws; m_fs_f_Rcs.mi2mult3(&m_ws_f_Rcs, &m_fs_f_ws); // #+# wahrscheinlich nur temporaer verwendet
    IVP_U_Matrix3 m_fs_f_Acs; m_fs_f_ws.mmult3(&m_ws_f_Acs, &m_fs_f_Acs);
    

    /********************************************************************************
     *	Description:	calculate the rotation system and pretension
     *  Output:         m_rs_f_Acs and m_rs_f_Rcs
     *                  drRA_rs
     ********************************************************************************/
    const IVP_U_Matrix3 &m_Rrs_f_Rcs = (m_Rfs_f_Rcs.rot) ? *(m_Rfs_f_Rcs.rot) : (IVP_U_Matrix3) m_Rfs_f_Rcs; //TL: cast (IVP_U_Matrix) needed for sun compiler
    const IVP_U_Matrix3 &m_Ars_f_Acs = (m_Afs_f_Acs.rot) ? *m_Afs_f_Acs.rot   : (IVP_U_Matrix3) m_Afs_f_Acs; //
    IVP_U_Matrix3 m_rs_f_Rcs_buffer;
    const IVP_U_Matrix3 *pm_rs_f_Rcs = &m_Rrs_f_Rcs;
    
    IVP_U_Matrix3 m_rs_f_ws, m_rs_f_Acs;
    
    IVP_U_Point drRA_now_rs; // orientation difference between both ?rs
    if (fixedrot_dim + limitedrot_dim == 1) { // cardan joint
        pm_rs_f_Rcs = &m_rs_f_Rcs_buffer;
	
	// The fixed axis of  Ars into Rrs:
	IVP_U_Point fixed_axisArs_Acs; m_Ars_f_Acs.get_row(mapping_uRrs_f_Rrs[IVP_INDEX_X], &fixed_axisArs_Acs);      // The fixed axis of objectA ...
	IVP_U_Point fixed_axisArs_ws;  m_ws_f_Acs.vmult3(&fixed_axisArs_Acs, &fixed_axisArs_ws);
	IVP_U_Point fixed_axisArs_Rcs; m_ws_f_Rcs.vimult3(&fixed_axisArs_ws, &fixed_axisArs_Rcs); // ... transformed into Rcs
	IVP_U_Point fixed_axisRrs_Rcs; m_Rrs_f_Rcs.get_row(mapping_uRrs_f_Rrs[IVP_INDEX_X], &fixed_axisRrs_Rcs); // The fixed reference axis of objectR in Rcs
	IVP_U_Point vertical_axisRA_Rcs; // The axis which is vertical to both axes
	vertical_axisRA_Rcs.calc_cross_product(&fixed_axisArs_Rcs, &fixed_axisRrs_Rcs); // what if vertical_axisRA_Rcs == nullvec?

	// find rs:
	// If (fixed_axisArs_Rcs, fixed_axisRrs_Rcs, vertical_axisRA_Rcs) is seen as a coordinate space m_rs_f_fixedaxisRcs,
	// then the x-axis of rs is on the line "y == -x" in this "fixedaxis"-system. y and z only orthonormized
	// Therefore, x_freeaxis == (1, 1, 0). => m_rs_f_Rcs * (1,0,0) == fixed_axisArs_Rfs - fixed_axisRrs_Rfs.

	// Add both axes. rsx_Rcs is parallel to rs_x only because both matrices are _ortho_normized.
        IVP_U_Point rsx_Rcs; rsx_Rcs.add(&fixed_axisArs_Rcs, &fixed_axisRrs_Rcs); 
	m_rs_f_Rcs_buffer.init_normized3_col(&rsx_Rcs, mapping_uRrs_f_Rrs[IVP_INDEX_X]); m_rs_f_Rcs_buffer.transpose3(); // #+# use init normized3_row or program init_normized3_inv
	pm_rs_f_Rcs->mi2mult3(&m_ws_f_Rcs, &m_rs_f_ws);
	m_rs_f_ws.mmult3(&m_ws_f_Acs, &m_rs_f_Acs);

	// Calculate orientation
	IVP_U_Point vertical_axisRA_Rrs; m_Rrs_f_Rcs.vmult3(&vertical_axisRA_Rcs, &vertical_axisRA_Rrs);

	// orientation in Rrs of the common Vertical -- Ausrichtung der gemeinsamen Senkrechten im Rrs:
        IVP_DOUBLE vertical_anglezy_Rrs = IVP_Inline_Math::atan2d(vertical_axisRA_Rrs.k[mapping_uRrs_f_Rrs[IVP_INDEX_Y]], vertical_axisRA_Rrs.k[mapping_uRrs_f_Rrs[IVP_INDEX_Z]]);

	IVP_U_Point vertical_axisRA_Acs; IVP_U_Point vertical_axisRA_Ars, vertical_axisRA_ws;
	m_ws_f_Rcs.vmult3(&vertical_axisRA_Rcs, &vertical_axisRA_ws);
	m_ws_f_Acs.vimult3(&vertical_axisRA_ws, &vertical_axisRA_Acs);
	m_Ars_f_Acs.vmult3(&vertical_axisRA_Acs, &vertical_axisRA_Ars);

	// Orientation in Ars of the common Vertical -- Ausrichtung der gemeinsamen Senkrechten im Ars:
        IVP_DOUBLE vertical_anglezy_Ars = IVP_Inline_Math::atan2d(vertical_axisRA_Ars.k[mapping_uRrs_f_Rrs[IVP_INDEX_Y]], vertical_axisRA_Ars.k[mapping_uRrs_f_Rrs[IVP_INDEX_Z]]);

	// The difference of orientations == the current twist:
	IVP_DOUBLE diff_axis = vertical_anglezy_Ars - vertical_anglezy_Rrs; 
	if (diff_axis > IVP_PI) diff_axis -= 2.0f * IVP_PI;
	if (diff_axis < -IVP_PI) diff_axis += 2.0f * IVP_PI;
	// IVP_DOUBLE qlen_rsx = rsx_Rcs.quad_length();  // 0 .. 4
	// if (qlen_rsx > 3 ) qlen_rsx = 3;
	drRA_now_rs.k[mapping_uRrs_f_Rrs[IVP_INDEX_X]] = diff_axis;   // hängt von known_axis ab
	drRA_now_rs.k[mapping_uRrs_f_Rrs[IVP_INDEX_Y]] = 0.0f;
	drRA_now_rs.k[mapping_uRrs_f_Rrs[IVP_INDEX_Z]] = 0.0f;
    } else {
	pm_rs_f_Rcs->mi2mult3(&m_ws_f_Rcs, &m_rs_f_ws); //  #+# move this lines into the next ifs 
	m_rs_f_ws.mmult3(&m_ws_f_Acs, &m_rs_f_Acs);
    }
    
    const IVP_U_Matrix3 &m_rs_f_Rcs = *pm_rs_f_Rcs;
    
    if (fixedrot_dim + limitedrot_dim == 3) { // same orientation
	IVP_U_Matrix3 m_rs_f_Ars; m_rs_f_Acs.mi2mult3(&m_Ars_f_Acs, &m_rs_f_Ars);
	
	IVP_U_Quat quaternion_relatedsystem;
	quaternion_relatedsystem.set_quaternion(&m_rs_f_Ars);
	drRA_now_rs.set(2.0f * IVP_Inline_Math::fast_asin(quaternion_relatedsystem.x),
		    2.0f * IVP_Inline_Math::fast_asin(quaternion_relatedsystem.y),
		    2.0f * IVP_Inline_Math::fast_asin(quaternion_relatedsystem.z));
	if ( quaternion_relatedsystem.w < 0.0f){
	    drRA_now_rs.mult(-1.0f);
	}
    } else if (fixedrot_dim + limitedrot_dim == 2) { // hinge
	// #+# free_axisArs_Acs und mit vimult3 nach free_axisArs_Rrs
	IVP_U_Matrix3 m_rs_f_Ars; m_rs_f_Acs.mi2mult3(&m_Ars_f_Acs, &m_rs_f_Ars);

	IVP_U_Point free_axisArs_Rrs;
	m_rs_f_Ars.get_col(mapping_uRrs_f_Rrs[IVP_INDEX_Z], &free_axisArs_Rrs);
	drRA_now_rs.set_to_zero();
	drRA_now_rs.k[mapping_uRrs_f_Rrs[IVP_INDEX_X]] = -IVP_Inline_Math::atan2d(
										free_axisArs_Rrs.k[mapping_uRrs_f_Rrs[IVP_INDEX_Y]],
										free_axisArs_Rrs.k[mapping_uRrs_f_Rrs[IVP_INDEX_Z]]);
	drRA_now_rs.k[mapping_uRrs_f_Rrs[IVP_INDEX_Y]] = IVP_Inline_Math::atan2d(
									       free_axisArs_Rrs.k[mapping_uRrs_f_Rrs[IVP_INDEX_X]],
									       free_axisArs_Rrs.k[mapping_uRrs_f_Rrs[IVP_INDEX_Z]]);
						     
    } else if (fixedrot_dim + limitedrot_dim == 0) { // free rotation
	drRA_now_rs.set_to_zero();
    }


    /********************************************************************************
     *	Description:	calculate the translation differences
     ********************************************************************************/
    // At this point drRA_rs is virtually the same as rRA_rs
    IVP_IF (0) {
	IVP_U_Float_Point world_vec;
	world_vec.set(0.0f,0.01f,0.0f);
	if (coreR) coreR->environment->add_draw_vector(m_ws_f_Rcs.get_position(), &world_vec, m_Rfs_f_Rcs.object->get_name(), 3);
	if (coreA) coreA->environment->add_draw_vector(m_ws_f_Acs.get_position(), &world_vec, m_Afs_f_Acs.object->get_name(), 3);
    }

    IVP_U_Point nullvec; nullvec.set_to_zero();
    IVP_U_Float_Point pointR_Rcs; m_Rfs_f_Rcs.vimult4(&nullvec, &pointR_Rcs);
    IVP_U_Float_Point pointA_Acs; m_Afs_f_Acs.vimult4(&nullvec, &pointA_Acs);
    IVP_U_Point pointR_ws; m_ws_f_Rcs.vmult4(&pointR_Rcs, &pointR_ws);
    IVP_U_Point pointA_ws; m_ws_f_Acs.vmult4(&pointA_Acs, &pointA_ws);
    IVP_U_Point pointR_Acs; m_ws_f_Acs.vimult4(&pointR_ws, &pointR_Acs);

    // All calculations will be done in fs and rs
    IVP_DOUBLE inv_dtime = es->i_delta_time;
  
    // better name it next_dvRA_fs and next_drRA_rs -- eigentlich next_dvRA_fs und next_drRA_rs    
    IVP_U_Point dvRA_fs; // desired differences of speed
    IVP_U_Point drRA_rs; // desired differences of angular velocity
    { // Calc desired velocity difference
        // Wie siehts im nächsten PSI aus?

        // Echte Kurvenberechnung ginge so:
        // aktuelle Position += coreX->speed * d_time
        // aktuelle Position += ... hmm, mueszte mit quaternions gehen, die man aufeinander addiert
        // oder indem man die Achse bestimmt, um die sich das Objekt dreht, und dann in der Rotationsebene
        // den neuen Punkt bestimmt 
        // Statt echter Kurvenberechnung mache ich nur eine lineare Annäherung.

        // Position:
        IVP_U_Float_Point  v_pointR_ws, v_pointA_ws;
	
	if (coreR){
	    coreR->get_surface_speed_on_test(&pointR_Rcs, &coreR->speed, &coreR->rot_speed, &v_pointR_ws);
	}else{
	    v_pointR_ws.set_to_zero();
	}
        if (coreA){
	    coreA->get_surface_speed_on_test(&pointA_Acs, &coreA->speed, &coreA->rot_speed, &v_pointA_ws);
	}else{
	    v_pointA_ws.set_to_zero();
	}
        IVP_U_Float_Point v_pointR_fs; m_fs_f_ws.vmult3(&v_pointR_ws, &v_pointR_fs);
        IVP_U_Float_Point v_pointA_fs; m_fs_f_ws.vmult3(&v_pointA_ws, &v_pointA_fs);
        // The velocity needed to reduce the position difference in the next PSI is needed. (dvRA)
	// current position == cpR, cpA; current velocity == vR, vA; d_time == t; next position == npR, npA.
	// dvRA == (npA - npR) / t  &&  npX == cpX + vX * t  =>
	// dvRA == (cpA - cpR) / t + vA - vR
        // the same calculation for rotation some lines below.
	{
	    IVP_U_Point dRA_now_fs;
	    IVP_U_Point dvRA_ws;    dvRA_ws.subtract(&pointA_ws, &pointR_ws);
	    m_fs_f_ws.vmult3(&dvRA_ws, &dRA_now_fs);

	    IVP_U_Point dv_fs; dv_fs.subtract( &v_pointA_fs, &v_pointR_fs);
	    dvRA_fs.set( &dRA_now_fs );
	    dvRA_fs.add_multiple( &dv_fs, d_time * damp_factor_div_force );

	    for (int i = 0; i < 3; i++) if (fixed[i] & IVP_CONSTRAINT_AXIS_LIMITED) {
		// Where A would be relative to R in next PSI:
		// npA - npR == dvRA * t
		if (dvRA_fs.k[i] < borderleft_Rfs[i]) {
		    dvRA_fs.k[i] = (dRA_now_fs.k[i] - borderleft_Rfs[i]) * limited_axis_stiffness + dv_fs.k[i] * d_time;

		    (int &) fixed[i] |= IVP_CONSTRAINT_AXIS_FIXED;
		} else if (dvRA_fs.k[i] > borderright_Rfs[i]) {
		    dvRA_fs.k[i] = (dRA_now_fs.k[i] - borderright_Rfs[i]) * limited_axis_stiffness + dv_fs.k[i] * d_time;
		    (int &) fixed[i] |= IVP_CONSTRAINT_AXIS_FIXED;
		} else {
		    (int &) fixed[i] &= ~IVP_CONSTRAINT_AXIS_FIXED;
		}
	    }
    	    dvRA_fs.mult(-force_factor * inv_dtime);
	}

        // Rotation: Attention: angles >  2 PI (?)
	IVP_U_Float_Point rR_rs, rA_rs;
	if (coreR) m_rs_f_Rcs.vmult3(&coreR->rot_speed, &rR_rs); else rR_rs.set_to_zero();
	if (coreA) m_rs_f_Acs.vmult3(&coreA->rot_speed, &rA_rs); else rA_rs.set_to_zero();
	
	// drRA_fs still contains the value for rRA_rs.
	{
	    IVP_U_Point drs_rs;	drs_rs.subtract( & rA_rs, &rR_rs );
	    drRA_rs.set( &drRA_now_rs );
	    drRA_rs.add_multiple( &drs_rs, d_time * damp_factor_div_force );

	    for (int i = 0; i < 3; i++){
		if (fixed[i + 3] & IVP_CONSTRAINT_AXIS_LIMITED) {
		    if (drRA_rs.k[i]  < borderleft_Rfs[i + 3]) {
			// dpos - rangenear 
			drRA_rs.k[i] = ( drRA_now_rs.k[i] - borderleft_Rfs[i + 3] ) * limited_axis_stiffness + drs_rs.k[i] * d_time;
			
			(int &) fixed[i + 3] |= IVP_CONSTRAINT_AXIS_FIXED;
			
		    } else if (drRA_rs.k[i] > borderright_Rfs[i + 3]) {
			drRA_rs.k[i] = ( drRA_now_rs.k[i] - borderright_Rfs[i + 3] ) * limited_axis_stiffness + drs_rs.k[i] * d_time;
			(int &) fixed[i + 3] |= IVP_CONSTRAINT_AXIS_FIXED;
		    } else {
			(int &) fixed[i + 3] &= ~IVP_CONSTRAINT_AXIS_FIXED;
		    }
		}
	    }
	    drRA_rs.mult(- force_factor * inv_dtime);
	}
	// the needed components (in RS and RS.rot) don't need to be extracted
	// due to the selective dimension of the bahaviour matrix
    }
    {
	int i;
	for (matrix_size = 0, i = 0; i < 6; i++){
	    matrix_size += fixed[i] & IVP_CONSTRAINT_AXIS_FIXED;
	}
    }
    
    if (fixedrot_dim == 1) {
	// Modification for cardan joints:
	// if both R-spaces form angles [90..180], the forces should be reduced

	IVP_U_Point fixed_axisArs_Acs; m_Ars_f_Acs.get_row(mapping_uRrs_f_Rrs[IVP_INDEX_X], &fixed_axisArs_Acs);      // Die feste Achse von objectA ...
	IVP_U_Point fixed_axisArs_ws;  m_ws_f_Acs.vmult3(&fixed_axisArs_Acs, &fixed_axisArs_ws);
	IVP_U_Point fixed_axisArs_Rcs; m_ws_f_Rcs.vimult3(&fixed_axisArs_ws, &fixed_axisArs_Rcs); // ... transformiert ins Rcs
	IVP_U_Point fixed_axisRrs_Rcs; m_Rrs_f_Rcs.get_row(mapping_uRrs_f_Rrs[IVP_INDEX_X], &fixed_axisRrs_Rcs); // Die feste Referenzachse von objectR im Rcs
	
	IVP_DOUBLE angleRA = IVP_Inline_Math::save_acosf(fixed_axisArs_Rcs.dot_product(&fixed_axisRrs_Rcs)); // vector angle
	IVP_DOUBLE slowrange = IVP_DOUBLE(IVP_PI - IVP_Inline_Math::fabsd(angleRA));  // 0 .. IVP_PI
        if (slowrange < 1.0f) drRA_rs.mult(slowrange);
    }
    
    // Testpushes. Vielleicht kann ich sie rausziehen, wenn die Resultate immer gleich sind?
    IVP_U_Point testnext_dvRA_fs[6], testnext_drRA_rs[6];

    // Translation
    {
	for (int i = 0; i < 3; i++) if (fixed[i] & IVP_CONSTRAINT_AXIS_FIXED) {
	    IVP_U_Float_Point testimpulseR_pointR_fs, testimpulseA_pointR_fs;
	    IVP_U_Float_Point testimpulseR_pointR_Rcs, testimpulseA_pointR_Acs;
	    IVP_U_Float_Point testimpulseR_pointR_ws, testimpulseA_pointR_ws;
	    testimpulseR_pointR_fs.set_to_zero();
	    testimpulseR_pointR_fs.k[i] = 1.0f;
	    testimpulseA_pointR_fs.set_negative(&testimpulseR_pointR_fs);  // #+# can be highly optimized, use get_row instead of vimult3
	    m_fs_f_Rcs.vimult3(&testimpulseR_pointR_fs, &testimpulseR_pointR_Rcs);
	    m_fs_f_Acs.vimult3(&testimpulseA_pointR_fs, &testimpulseA_pointR_Acs);
	    m_fs_f_ws.vimult3(&testimpulseR_pointR_fs, &testimpulseR_pointR_ws);
	    m_fs_f_ws.vimult3(&testimpulseA_pointR_fs, &testimpulseA_pointR_ws);
    /* So funktioniert die Berechnung unter der Annahme, daß sowohl coreR als auch coreA beweglich sind:
	    coreR->test_push_core(&pointR_Rcs, &testimpulseR_pointR_Rcs, &testimpulseR_pointR_ws, &dvR_ws, &drR_Rcs);
	    coreR->inline_get_surface_speed_on_test(&pointR_Rcs, &dvR_ws, &drR_Rcs, &dv_pointR_ws); 
        
	    tempvec.inline_calc_cross_product(pointR_Rcs, testimpulseR_pointR_Rcs);
	    drR_Rcs->pairwise_mult(tempvec, coreR->rot_inertia);
	    dvR_ws->inline_set_multiple(&testimpulseR_pointR_ws, coreR->inv_mass);
	    IVP_U_Point_PCore temppc;
	    temppc.inline_calc_cross_product(drR_Rcs, pointR_Rcs);
	    coreR->m_world_f_last_psi.vmult3(temppc, dv_pointR_ws);
	    dv_pointR_ws->add(dvR_ws);
        
	      coreA->test_push_core(&pointR_Acs, &testimpulseA_pointR_Acs, &testimpulseA_pointR_ws, &dvA_ws, &drA_Acs);
	    coreA->inline_get_surface_speed_on_test(&pointA_Acs, &dvA_ws, &drA_Acs, &dv_pointA_ws); 
    */

	    IVP_U_Float_Point dvR_ws, dvA_ws, drR_Rcs, drA_Acs, drR_rs, drA_rs;
	    if (!coreR || coreR->physical_unmoveable) {
		dvR_ws.set_to_zero();
		drR_Rcs.set_to_zero();
	    } else
		coreR->test_push_core(&pointR_Rcs, &testimpulseR_pointR_Rcs, &testimpulseR_pointR_ws, &dvR_ws, &drR_Rcs); // Push at pointR
	    if (!coreA || coreA->physical_unmoveable) {
		dvA_ws.set_to_zero();
		drA_Acs.set_to_zero();
	    } else {
    #ifdef PUSH_SAMEPOINT
		coreA->test_push_core(&pointR_Acs, &testimpulseA_pointR_Acs, &testimpulseA_pointR_ws, &dvA_ws, &drA_Acs); // Push at pointR
    #else
		coreA->test_push_core(&pointA_Acs, &testimpulseA_pointR_Acs, &testimpulseA_pointR_ws, &dvA_ws, &drA_Acs); // Push at pointA
    #endif
	    }
	    // Calculate the velocity changes of the impulse at the point
	    IVP_U_Float_Point dv_pointR_ws, dv_pointA_ws, dv_pointR_fs, dv_pointA_fs;
	    if (coreR){
		coreR->get_surface_speed_on_test(&pointR_Rcs, &dvR_ws, &drR_Rcs, &dv_pointR_ws);
	    }else{
		dv_pointR_ws.set_to_zero();
	    }
	    if (coreA){
		coreA->get_surface_speed_on_test(&pointA_Acs, &dvA_ws, &drA_Acs, &dv_pointA_ws);
	    }else{
		dv_pointA_ws.set_to_zero();
	    }
	    
	    // translation results
	    m_fs_f_ws.vmult3(&dv_pointR_ws, &dv_pointR_fs);
	    m_fs_f_ws.vmult3(&dv_pointA_ws, &dv_pointA_fs);
	    testnext_dvRA_fs[i].subtract(&dv_pointA_fs, &dv_pointR_fs);

	    // rotation results
	    m_rs_f_Rcs.vmult3(&drR_Rcs, &drR_rs);
	    m_rs_f_Acs.vmult3(&drA_Acs, &drA_rs);
	    testnext_drRA_rs[i].subtract(&drA_rs, &drR_rs);
	}
    }
    
    // Rotation
    {
	for (int i = 3; i < 6; i++) if (fixed[i] & IVP_CONSTRAINT_AXIS_FIXED) {
	    IVP_U_Float_Point testimpulseRrs_rs, testimpulseArs_rs;
	    IVP_U_Float_Point testimpulseRrs_Rcs, testimpulseArs_Acs;
	    testimpulseRrs_rs.set_to_zero();
	    testimpulseRrs_rs.k[i-3] = 1.0f;
	    testimpulseArs_rs.set_negative(&testimpulseRrs_rs);
	    m_rs_f_Rcs.vimult3(&testimpulseRrs_rs, &testimpulseRrs_Rcs);
	    m_rs_f_Acs.vimult3(&testimpulseArs_rs, &testimpulseArs_Acs);

    /* So funktioniert die Berechnung unter der Annahme, dasz sowohl coreR als auch coreA beweglich sind:
	    coreR->test_rot_push_core_multiple_cs(&testimpulseRrs_Rcs, 1.0, &drR_rot_Rcs);
	    coreR->inline_get_surface_speed_on_rotation_test_cs(&pointR_Rcs, &drR_rot_Rcs, &dv_pointR_Rcs);

	    drR_rot_Rcs->pairwise_mult(testimpulseRrs_Rcs, inv_rot_inertia); 
	    dv_pointR_Rcs->inline_calc_cross_product(drR_rot_Rcs, pointR_Rcs);
    */
	    IVP_U_Float_Point drR_rot_Rcs, drA_rot_Acs;
	    if (!coreR || coreR->physical_unmoveable) {
		drR_rot_Rcs.set_to_zero();
	    } else {
		coreR->test_rot_push_core_multiple_cs(&testimpulseRrs_Rcs, 1.0f, &drR_rot_Rcs);
	    }
	    if (!coreA || coreA->physical_unmoveable) {
		drA_rot_Acs.set_to_zero();
	    } else {
		coreA->test_rot_push_core_multiple_cs(&testimpulseArs_Acs, 1.0f, &drA_rot_Acs);
	    }
	    // Calculate the resulting surface speed changes of the impulse at the point
	    IVP_U_Float_Point dv_pointR_Rcs, dv_pointA_Acs, dv_pointR_fs, dv_pointA_fs;
	    IVP_U_Float_Point drR_rs, drA_rs;

	    // calc surface speed on rotation test:
	    if (coreR) {
		dv_pointR_Rcs.inline_calc_cross_product(&drR_rot_Rcs, &pointR_Rcs);
	    } else {
		dv_pointR_Rcs.set_to_zero();
	    }
	    if (coreA) {
		dv_pointA_Acs.inline_calc_cross_product(&drA_rot_Acs, &pointA_Acs);
	    } else {
		dv_pointA_Acs.set_to_zero();
	    }

	    // translation results
	    m_fs_f_Rcs.vmult3(&dv_pointR_Rcs, &dv_pointR_fs);
	    m_fs_f_Acs.vmult3(&dv_pointA_Acs, &dv_pointA_fs);
	    testnext_dvRA_fs[i].subtract(&dv_pointA_fs, &dv_pointR_fs);

	    // rotation results
	    m_rs_f_Rcs.vmult3(&drR_rot_Rcs, &drR_rs);
	    m_rs_f_Acs.vmult3(&drA_rot_Acs, &drA_rs);
	    testnext_drRA_rs[i].subtract(&drA_rs, &drR_rs);
	}
    }
    
    // Nun hab ich Matrizen der Größe 0 bis 6.
    // Noch nicht gemacht: Die unterschiedlichen Matrixgrößen beachten!
    // Matrix erschaffen und vimult3(dvRA, impulseR);
    IVP_Great_Matrix_Many_Zero mg_impulse_f_dvRA;
    mg_impulse_f_dvRA.columns = matrix_size;
    mg_impulse_f_dvRA.calc_aligned_row_len();
    int aligned_row_len = mg_impulse_f_dvRA.aligned_row_len;
    mg_impulse_f_dvRA.MATRIX_EPS = 1E-9f;
    IVP_ASSERT( aligned_row_len <= 8 );
#if defined(IVP_NO_ALLOCA)    
    IVP_DOUBLE matrix_values_buffer[ 6 * 8 + 3 ];
    IVP_DOUBLE result_vector_buffer[ 8 ];
    IVP_DOUBLE desired_vector_buffer[ 8 ];
    mg_impulse_f_dvRA.matrix_values = (IVP_DOUBLE *) &matrix_values_buffer[0];
    mg_impulse_f_dvRA.result_vector = (IVP_DOUBLE *) &result_vector_buffer[0];
    mg_impulse_f_dvRA.desired_vector = (IVP_DOUBLE *) &desired_vector_buffer[0];
#else    
    mg_impulse_f_dvRA.matrix_values = (IVP_DOUBLE *) alloca((matrix_size * aligned_row_len + IVP_VECFPU_SIZE - 1) * sizeof(IVP_DOUBLE));
    mg_impulse_f_dvRA.result_vector = (IVP_DOUBLE *) alloca(aligned_row_len * sizeof(IVP_DOUBLE));
    mg_impulse_f_dvRA.desired_vector = (IVP_DOUBLE *) alloca(aligned_row_len * sizeof(IVP_DOUBLE));
#endif
    //align matrix_values
    mg_impulse_f_dvRA.align_matrix_values();
    
    {
	unsigned int pos_i = 0;
	for (int i = 0; i < 6; i++) if (fixed[i] & IVP_CONSTRAINT_AXIS_FIXED) {  // waagrecht durch
	    unsigned int pos_j = 0;
	    int j;
	    // insert Translation results (vertical)
	    for (j = 0; j < 3; j++) if (fixed[j] & IVP_CONSTRAINT_AXIS_FIXED) { 
		mg_impulse_f_dvRA.matrix_values[pos_i + aligned_row_len * pos_j] = testnext_dvRA_fs[i].k[j];
		pos_j++;
	    }
	    // insert rotation results (vertical)
	    for (; j < 6; j++) if (fixed[j] & IVP_CONSTRAINT_AXIS_FIXED) { 
		mg_impulse_f_dvRA.matrix_values[pos_i + aligned_row_len * pos_j] = testnext_drRA_rs[i].k[j - 3];
		pos_j++;
	    }
	    // insert desired_vector
	    if (i < 3) {
		mg_impulse_f_dvRA.desired_vector[pos_i] = dvRA_fs.k[i];
	    } else {
		mg_impulse_f_dvRA.desired_vector[pos_i] = drRA_rs.k[i - 3];
	    }
	    pos_i++;
	}
    }

    ;
    // invert matrix and solve equation
    if (!mg_impulse_f_dvRA.solve_great_matrix_many_zero()){
        //printf("Couldn't solve constraint matrix between %s and %s!\n", m_Rfs_f_Rcs.object->get_name(), m_Afs_f_Acs.object->get_name());
	return;
    }
    // get the resulting impulse
    IVP_U_Float_Point impulseR_fs, impulserotR_rs, impulseA_fs, impulserotA_rs;
    {
	int pos_i = 0;
	for (int i = 0; i < 3; i++) {
	    if (fixed[i] & IVP_CONSTRAINT_AXIS_FIXED)
		impulseR_fs.k[i] = mg_impulse_f_dvRA.result_vector[pos_i];
	    else
		impulseR_fs.k[i] = 0.0f;
	    if (fixed[i] & IVP_CONSTRAINT_AXIS_FIXED)
		pos_i++;
	}
	for (int j = 3; j < 6; j++) {
	    if (fixed[j] & IVP_CONSTRAINT_AXIS_FIXED)
		impulserotR_rs.k[j - 3] = mg_impulse_f_dvRA.result_vector[pos_i];
	    else
		impulserotR_rs.k[j - 3] = 0.0f;
	    if (fixed[j] & IVP_CONSTRAINT_AXIS_FIXED)
		pos_i++;
	}
    }
    mg_impulse_f_dvRA.matrix_values = NULL;
    mg_impulse_f_dvRA.result_vector = NULL;
    mg_impulse_f_dvRA.desired_vector = NULL;    
    
    // Ideen:
    // klippen
    // modifizieren
    // veraenderte Wirkung berechnen
    // Impulse neu berechnen
    // oder so

    if (maxforce) {
	int in_range;
	switch (norm) {
	case IVP_NORM_MINIMUM:
	  /// #+# avoid divisionen !!!!!
	    in_range = (IVP_Inline_Math::fabsd(impulseR_fs.k[0] ) / maxforce->halfimpulse[0] +
			IVP_Inline_Math::fabsd(impulseR_fs.k[1] ) / maxforce->halfimpulse[1] +
			IVP_Inline_Math::fabsd(impulseR_fs.k[2] ) / maxforce->halfimpulse[2]) < 3.0f;
	    break;
	case IVP_NORM_EUCLIDIC:
	    /// vectoroperationen
	    {
		IVP_FLOAT einheitsforce_x = IVP_Inline_Math::fabsd(impulseR_fs.k[0] ) / maxforce->halfimpulse[0];
		IVP_FLOAT einheitsforce_y = IVP_Inline_Math::fabsd(impulseR_fs.k[1] ) / maxforce->halfimpulse[1];
		IVP_FLOAT einheitsforce_z = IVP_Inline_Math::fabsd(impulseR_fs.k[2] ) / maxforce->halfimpulse[2];
		IVP_FLOAT qlen = einheitsforce_x * einheitsforce_x + einheitsforce_y * einheitsforce_y + einheitsforce_z * einheitsforce_z;
		
		in_range = qlen * inv_dtime * inv_dtime < 1.0f;
	    }
	    break;
	case IVP_NORM_MAXIMUM:
	    in_range = IVP_Inline_Math::fabsd(impulseR_fs.k[0]) < maxforce->halfimpulse[0]
		&& IVP_Inline_Math::fabsd(impulseR_fs.k[1]) < maxforce->halfimpulse[1]
		&& IVP_Inline_Math::fabsd(impulseR_fs.k[2]) < maxforce->halfimpulse[2]
		&& IVP_Inline_Math::fabsd(impulserotR_rs.k[0]) < maxforce->halfimpulse[3]
		&& IVP_Inline_Math::fabsd(impulserotR_rs.k[1]) < maxforce->halfimpulse[4]
		&& IVP_Inline_Math::fabsd(impulserotR_rs.k[2]) < maxforce->halfimpulse[5];
	    break;
	}
        IVP_IF (0) {
	    printf("%f  %f  %f     ", IVP_Inline_Math::fabsd(impulseR_fs.k[0]) , IVP_Inline_Math::fabsd(impulseR_fs.k[1]) , IVP_Inline_Math::fabsd(impulseR_fs.k[2]));
	    printf("%f  %f  %f\n", IVP_Inline_Math::fabsd(impulserotR_rs.k[0]) , IVP_Inline_Math::fabsd(impulserotR_rs.k[1]) , IVP_Inline_Math::fabsd(impulserotR_rs.k[2]));
	}
	// norm < maxforce.half => nothing happens
	if (!in_range) {
	    // norm_x > maxforce.half_x => constraint breaks
	    IVP_FLOAT maxforce_factor = 1.0f;

	    /// #+# switch ausserhalb von loop
	    for (int i = 0; i < 3; i++) {
		short map[5] = {0, 1, 2, 0, 1};
		IVP_DOUBLE simplenorm_transdir, simplenorm_rotdir;
		switch (norm) {
		case IVP_NORM_MINIMUM:
		    simplenorm_transdir = IVP_Inline_Math::fabsd(impulseR_fs.k[0]) + IVP_Inline_Math::fabsd(impulseR_fs.k[1]) + IVP_Inline_Math::fabsd(impulseR_fs.k[2]);
		    break;
		case IVP_NORM_EUCLIDIC: 
		    simplenorm_transdir = IVP_Inline_Math::fabsd(impulseR_fs.k[map[i]]) + IVP_Inline_Math::fabsd(impulseR_fs.k[map[i + 1]]) * (1.0f / 2.41f) + IVP_Inline_Math::fabsd(impulseR_fs.k[map[i + 2]]) * (1.0f/ 2.41f);
		    break;
		case IVP_NORM_MAXIMUM:
		    simplenorm_transdir = IVP_Inline_Math::fabsd(impulseR_fs.k[i]);
		    simplenorm_rotdir = IVP_Inline_Math::fabsd(impulserotR_rs.k[i]);
		    break;
		}
		if (maxforce->halfimpulse[i]  <  simplenorm_transdir) {
		    switch (maxforce->type[i]) {
		    case IVP_CFE_CLIP: // constraint shall be too weak in this axis
			{
			    IVP_FLOAT newfactor = d_time * maxforce->halfimpulse[i] / simplenorm_transdir;
			    if (newfactor < maxforce_factor)
				maxforce_factor = newfactor;
			}
			break;
		    case IVP_CFE_BREAK: // constraint breaks in this axis
                        // Oh no, I can not simply set type[i] to IVP_CONSTRAINT_AXIS_FREE :-(
                        // Ich brech den Constraint mal ganz
                        // Maybe I should call a notify() function

				// Don't delete, deactivate
			get_environment()->fire_event_constraint_broken( this );
			deactivate();
			//P_DELETE_THIS(this);
			return;
			//free_translation_axis((IVP_COORDINATE_INDEX) i);
			IVP_IF (1) printf("Translationsachse %d ist gebrochen!\n", i);
			break;
		    case IVP_CFE_BEND: // Constraint verschiebt sich in diese Richtung
                        // Maybe I should call a notify() function
                        // Matrizen verschieben und verdrehen
                        // nur -- wie weit?
			// NOT implemented
			break;
		    case IVP_CFE_NONE:
			;
		    }
		}
		if (maxforce->halfimpulse[i + 3] / simplenorm_rotdir < 1.0f) {
			switch(maxforce->type[i+3]) {
		    case IVP_CFE_CLIP:
			break;
		    case IVP_CFE_BREAK:
			get_environment()->fire_event_constraint_broken( this );
			deactivate();
			//P_DELETE_THIS(this);
			return;
			//ee_rotation_axis((IVP_COORDINATE_INDEX) i);
			IVP_IF (1) printf("Rotationsachse %d ist gebrochen!\n", i);
			break;
		    case IVP_CFE_BEND:
			// NOT implemented
			break;
		    case IVP_CFE_NONE:
			;
		    }
		}
	    }
	    // ...
	    impulseR_fs.mult(maxforce_factor);
	    impulserotR_rs.mult(maxforce_factor);
	}
    }
    
    impulseA_fs.set_negative(&impulseR_fs);
    impulserotA_rs.set_negative(&impulserotR_rs);

    // attach the impulses to both objects
    IVP_U_Float_Point impulseR_ws, impulseR_Rcs, impulserotR_Rcs, impulseA_ws, impulseA_Acs, impulserotA_Acs;
    m_fs_f_Rcs.vimult3(&impulseR_fs, &impulseR_Rcs);
    m_fs_f_Acs.vimult3(&impulseA_fs, &impulseA_Acs);
    m_ws_f_Rcs.vmult3(&impulseR_Rcs, &impulseR_ws);
    m_ws_f_Acs.vmult3(&impulseA_Acs, &impulseA_ws);
    m_rs_f_Rcs.vimult3(&impulserotR_rs, &impulserotR_Rcs);
    m_rs_f_Acs.vimult3(&impulserotA_rs, &impulserotA_Acs);

    
    IVP_U_Float_Point result_dvRA_fs, result_drRA_rs; // for plausible check
    IVP_IF (0) { // check for plausible values
	IVP_U_Float_Point dvR_ws, dvA_ws, drR_Rcs, drA_Acs, drR_rot_Rcs, drA_rot_Acs;
        if (coreR && coreR->physical_unmoveable) {
	    dvR_ws.set_to_zero();
	    drR_Rcs.set_to_zero();
	    drR_rot_Rcs.set_to_zero();
	} else {
	    coreR->test_push_core(&pointR_Rcs, &impulseR_Rcs, &impulseR_ws, &dvR_ws, &drR_Rcs);
	    coreR->test_rot_push_core_multiple_cs(&impulserotR_Rcs, 1.0f, &drR_rot_Rcs);
        }
        if (coreA && coreA->physical_unmoveable) {
	    dvA_ws.set_to_zero();
	    drA_Acs.set_to_zero();
	    drA_rot_Acs.set_to_zero();
	} else {
	    coreA->test_push_core(&pointA_Acs, &impulseA_Acs, &impulseA_ws, &dvA_ws, &drA_Acs);
	    coreA->test_rot_push_core_multiple_cs(&impulserotA_Acs, 1.0f, &drA_rot_Acs);
	}
	result_dvRA_fs.subtract(&dvA_ws, &dvR_ws);
	m_fs_f_ws.vmult3(&result_dvRA_fs, &result_dvRA_fs);
	drR_Rcs.add(&drR_rot_Rcs);
	drA_Acs.add(&drA_rot_Acs);
	m_rs_f_Rcs.vmult3(&drR_Rcs, &drR_Rcs);
	m_rs_f_Acs.vmult3(&drA_Acs, &drA_Acs);
	result_drRA_rs.subtract(&drA_Acs, &drR_Rcs);
    }
    IVP_IF (0) {
	printf("%f [%s]-[%s]: IT=%f  IR=%f\n", coreR->environment->get_current_time().get_time(), m_Rfs_f_Rcs.object->get_name(), m_Afs_f_Acs.object->get_name(),
	       impulseR_Rcs.real_length(), impulserotR_Rcs.real_length());
    }
    IVP_DOUBLE debugfactor = 0.01f;
    if (coreR && !coreR->physical_unmoveable) {
	coreR->push_core(&pointR_Rcs, &impulseR_Rcs, &impulseR_ws);
	coreR->rot_push_core_cs(&impulserotR_Rcs);
	IVP_IF (1) { // show lines
	    impulseR_ws.mult(debugfactor * 2.0f * inv_dtime);
	    coreR->environment->add_draw_vector(&pointR_ws, &impulseR_ws, "", 2); // grün
	    IVP_U_Float_Point impulserotR_ws; m_ws_f_Rcs.vmult3(&impulserotR_Rcs, &impulserotR_ws);
	    impulserotR_ws.mult(debugfactor * 2.0f * inv_dtime);
	    coreR->environment->add_draw_vector(&pointR_ws, &impulserotR_ws, "", 5); // gelb
	}
    }
    if (coreA && !coreA->physical_unmoveable) {
#ifdef PUSH_SAMEPOINT
	coreA->push_core(&pointR_Acs, &impulseA_Acs, &impulseA_ws); // Push am pointR
#else
	coreA->push_core(&pointA_Acs, &impulseA_Acs, &impulseA_ws); // Push am pointA
#endif
	coreA->rot_push_core_cs(&impulserotA_Acs);
    }
}

#if 0
void IVP_Constraint_Local::change_fixed_object(IVP_Real_Object *objR) {
    // was wenn jetzt beide Objekte NULL werden?
    // cores_of_constraint_system anpassen
    if (objR) objR->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    IVP_U_Matrix m_identity; m_identity.init();
    const IVP_U_Matrix &m_ws_f_pRcs = m_Rfs_f_Rcs.object ? *m_Rfs_f_Rcs.object->get_core()->get_m_world_f_core_PSI() : m_identity;
    const IVP_U_Matrix &m_ws_f_nRcs = objR ? *objR->get_core()->get_m_world_f_core_PSI() : m_identity;
    m_Rfs_f_Rcs.object = objR;
    if (m_Rfs_f_Rcs.rot) {
	IVP_U_Matrix3 m_Rrs_f_ws; m_Rfs_f_Rcs.rot->mi2mult3(&m_ws_f_pRcs, &m_Rrs_f_ws);
	m_Rrs_f_ws.mmult3(&m_ws_f_nRcs, m_Rfs_f_Rcs.rot);
    }
    IVP_U_Matrix m_Rfs_f_ws; m_Rfs_f_Rcs.mi2mult4(&m_ws_f_pRcs, &m_Rfs_f_ws);
    m_Rfs_f_ws.mmult4(&m_ws_f_nRcs, &m_Rfs_f_Rcs);
}

void IVP_Constraint_Local::change_attached_object(IVP_Real_Object *objA) {
    // was wenn jetzt beide Objekte NULL werden?
    // cores_of_constraint_system anpassen
    if (objA) objA->ensure_in_simulation(); else m_Rfs_f_Rcs.object->ensure_in_simulation();
    IVP_U_Matrix m_identity; m_identity.init();
    const IVP_U_Matrix &m_ws_f_pAcs = m_Afs_f_Acs.object ? *m_Afs_f_Acs.object->get_core()->get_m_world_f_core_PSI() : m_identity;
    const IVP_U_Matrix &m_ws_f_nAcs = objA ? *objA->get_core()->get_m_world_f_core_PSI() : m_identity;
    m_Afs_f_Acs.object = objA;
    if (m_Afs_f_Acs.rot) {
	IVP_U_Matrix3 m_Ars_f_ws; m_Afs_f_Acs.rot->mi2mult3(&m_ws_f_pAcs, &m_Ars_f_ws);
	m_Ars_f_ws.mmult3(&m_ws_f_nAcs, m_Afs_f_Acs.rot);
    }
    IVP_U_Matrix m_Afs_f_ws; m_Afs_f_Acs.mi2mult4(&m_ws_f_pAcs, &m_Afs_f_ws);
    m_Afs_f_ws.mmult4(&m_ws_f_nAcs, &m_Afs_f_Acs);
}
#endif

void IVP_Constraint_Local::change_fixing_point_Ros(const IVP_U_Point *anchor) { // point_Ros_f_nRfs, point_nRfs_in_Ros
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    IVP_U_Matrix m_Rcs_f_Ros; if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->calc_m_core_f_object(&m_Rcs_f_Ros); else m_Rcs_f_Ros.init();
    IVP_U_Point point_Rcs_f_nRfs; m_Rcs_f_Ros.vmult4(anchor, &point_Rcs_f_nRfs); 
    IVP_U_Point point_Rfs_f_nRfs; m_Rfs_f_Rcs.vmult4(&point_Rcs_f_nRfs, &point_Rfs_f_nRfs); // point_nRfs_in_Rfs
    m_Rfs_f_Rcs.vv.subtract(&point_Rfs_f_nRfs);

/* if I would have to translate -- wenn ich übersetzen müßte:
    IVP_U_Point point_Rcs_f_nuRfs; m_Rcs_f_Ros.vmult4(anchor, &point_Rcs_f_nuRfs);
    IVP_U_Matrix3 m_uRfs_f_Rcs; mapping_uRfs_f_Rfs.mapply((IVP_U_Matrix3 *) &m_Rfs_f_Rcs, &m_uRfs_f_Rcs);
    IVP_U_Point point_nuRfs_f_Rcs; m_uRfs_f_Rcs.vmult3(&point_Rcs_f_nuRfs, &point_nuRfs_f_Rcs);
    point_nuRfs_f_Rcs.set_negative(&point_nuRfs_f_Rcs);
    mapping_uRfs_f_Rfs.viapply(&point_nuRfs_f_Rcs, &m_Rfs_f_Rcs.vv);
*/  
    m_Afs_f_Acs.vv.subtract(&point_Rfs_f_nRfs); // weil die Punkte so verschoben werden müssen, als lägen Rfs und Afs aufeinander.
}

void IVP_Constraint_Local::change_target_fixing_point_Ros(const IVP_U_Point *anchor) { // point_Ros_f_nRfs
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    IVP_U_Matrix m_Rcs_f_Ros; if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->calc_m_core_f_object(&m_Rcs_f_Ros); else m_Rcs_f_Ros.init();
    IVP_U_Point point_Rcs_f_nRfs; m_Rcs_f_Ros.vmult4(anchor, &point_Rcs_f_nRfs); 
    IVP_U_Point point_Rfs_f_nRfs; m_Rfs_f_Rcs.vmult4(&point_Rcs_f_nRfs, &point_Rfs_f_nRfs); // point_nRfs_in_Rfs
    m_Rfs_f_Rcs.vv.subtract(&point_Rfs_f_nRfs);
}

void IVP_Constraint_Local::change_translation_axes_Ros(const IVP_U_Matrix3 *trans_axes) { // m_Ros_f_nRfs, m_nRfs_in_Ros
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    IVP_U_Matrix m_Rcs_f_Ros; if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->calc_m_core_f_object(&m_Rcs_f_Ros); else m_Rcs_f_Ros.init();
    IVP_U_Matrix3 m_Rcs_f_nRfs; m_Rcs_f_Ros.mmult3(trans_axes, &m_Rcs_f_nRfs);
    IVP_U_Matrix3 m_Rfs_f_nRfs; m_Rfs_f_Rcs.mmult3(&m_Rcs_f_nRfs, &m_Rfs_f_nRfs);

    m_Rfs_f_Rcs.set_transpose3(&m_Rcs_f_nRfs);

    IVP_U_Matrix3 m_nAfs_f_Acs; m_Rfs_f_nRfs.mimult3(&m_Afs_f_Acs, &m_nAfs_f_Acs);

	IVP_U_Point tmp;
	m_Afs_f_Acs.vimult3( &m_Afs_f_Acs.vv, &tmp );
	m_nAfs_f_Acs.vmult3( &tmp, &m_Afs_f_Acs.vv );

    (IVP_U_Matrix3 &)m_Afs_f_Acs = m_nAfs_f_Acs;
}

void IVP_Constraint_Local::change_target_translation_axes_Ros(const IVP_U_Matrix3 *trans_axes) { // m_Ros_f_nRfs, m_nRfs_in_Ros
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    IVP_U_Matrix m_Rcs_f_Ros; if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->calc_m_core_f_object(&m_Rcs_f_Ros); else m_Rcs_f_Ros.init();
    IVP_U_Matrix3 m_Rcs_f_nRfs; m_Rcs_f_Ros.mmult3(trans_axes, &m_Rcs_f_nRfs);

    m_Rfs_f_Rcs.set_transpose3(&m_Rcs_f_nRfs);

}

void IVP_Constraint_Local::fix_translation_axis(IVP_COORDINATE_INDEX which) {
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    fixed[which] = IVP_CONSTRAINT_AXIS_FIXED;
    sort_translation_mapping();
}

void IVP_Constraint_Local::free_translation_axis(IVP_COORDINATE_INDEX which) {
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    fixed[which] = IVP_CONSTRAINT_AXIS_FREE;
    sort_translation_mapping();
}

void IVP_Constraint_Local::limit_translation_axis(IVP_COORDINATE_INDEX which, IVP_FLOAT border_left, IVP_FLOAT border_right) {
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation(); 
    fixed[which] = IVP_CONSTRAINT_AXIS_LIMITED;
    borderleft_Rfs[which] = border_left;
    borderright_Rfs[which] = border_right;
    sort_translation_mapping();
}

void IVP_Constraint_Local::change_max_translation_impulse(IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse) {
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    if (!maxforce) {
	maxforce = new IVP_Constraint_Local_MaxImpulse();
	maxforce->type[3] = maxforce->type[4] = maxforce->type[5] = IVP_CFE_NONE;
    }
    maxforce->halfimpulse[0] = maxforce->halfimpulse[1] = maxforce->halfimpulse[2] = impulse;
    maxforce->type[0] = maxforce->type[1] = maxforce->type[2] = impulsetype;
}

void IVP_Constraint_Local::change_rotation_axes_Ros(const IVP_U_Matrix3 *rot_axes) { // m_Ros_f_nRrs, m_nRrs_in_Ros
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    if (!m_Rfs_f_Rcs.rot) {
	m_Rfs_f_Rcs.rot = new IVP_U_Matrix3;
	m_Rfs_f_Rcs.rot->init3();
    }
	// #+# can be optimized a lot
    IVP_U_Matrix m_Rcs_f_Ros; if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->calc_m_core_f_object(&m_Rcs_f_Ros); else m_Rcs_f_Ros.init();
    IVP_U_Matrix3 m_Rcs_f_nRrs; m_Rcs_f_Ros.mmult3(rot_axes, &m_Rcs_f_nRrs);
    IVP_U_Matrix3 m_Rrs_f_nRrs; m_Rfs_f_Rcs.rot->mmult3(&m_Rcs_f_nRrs, &m_Rrs_f_nRrs);
    IVP_U_Matrix3 m_nRrs_f_Rcs; m_Rrs_f_nRrs.mimult3(m_Rfs_f_Rcs.rot, &m_nRrs_f_Rcs);
    *m_Rfs_f_Rcs.rot = m_nRrs_f_Rcs;
    
    IVP_U_Matrix3 m_nArs_f_Acs; m_Rrs_f_nRrs.mimult3(m_Afs_f_Acs.rot, &m_nArs_f_Acs);
    *m_Afs_f_Acs.rot = m_nArs_f_Acs;
}

void IVP_Constraint_Local::change_target_rotation_axes_Ros(const IVP_U_Matrix3 *rot_axes) { // m_Ros_f_nRrs
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    if (!m_Rfs_f_Rcs.rot) {
	m_Rfs_f_Rcs.rot = new IVP_U_Matrix3();
    }
	// #+# can be optimized a lot
    IVP_U_Matrix m_Rcs_f_Ros; if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->calc_m_core_f_object(&m_Rcs_f_Ros); else m_Rcs_f_Ros.init();
    IVP_U_Matrix3 m_Rcs_f_nRrs; m_Rcs_f_Ros.mmult3(rot_axes, &m_Rcs_f_nRrs);  // m_Rcs_f_nRrs inverted is the result
    *m_Rfs_f_Rcs.rot = m_Rcs_f_nRrs;
    m_Rfs_f_Rcs.rot->transpose3();
}

void IVP_Constraint_Local::fix_rotation_axis(IVP_COORDINATE_INDEX which) {
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    fixed[which + 3] = IVP_CONSTRAINT_AXIS_FIXED;
    sort_rotation_mapping();
}

void IVP_Constraint_Local::free_rotation_axis(IVP_COORDINATE_INDEX which) {
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    fixed[which + 3] = IVP_CONSTRAINT_AXIS_FREE;
    sort_rotation_mapping();
}

void IVP_Constraint_Local::limit_rotation_axis(IVP_COORDINATE_INDEX which, IVP_FLOAT border_left, IVP_FLOAT border_right) {
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    fixed[which + 3] = IVP_CONSTRAINT_AXIS_LIMITED;
    borderleft_Rfs[which + 3] = border_left;
    borderright_Rfs[which + 3] = border_right;
    sort_rotation_mapping();
}

void IVP_Constraint_Local::change_max_rotation_impulse(IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse) {
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    if (!maxforce) {
	maxforce = new IVP_Constraint_Local_MaxImpulse();
	maxforce->type[0] = maxforce->type[1] = maxforce->type[2] = IVP_CFE_NONE;
    }
    maxforce->halfimpulse[3] = maxforce->halfimpulse[4] = maxforce->halfimpulse[5] = impulse;
    maxforce->type[3] = maxforce->type[4] = maxforce->type[5] = impulsetype;
}


void IVP_Constraint_Local::change_Aos_to_relaxe_constraint() {
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    const IVP_U_Matrix *m_ws_f_Rcs; m_ws_f_Rcs = m_Rfs_f_Rcs.object->get_core()->get_m_world_f_core_PSI();
    const IVP_U_Matrix *m_ws_f_Acs; m_ws_f_Acs = m_Afs_f_Acs.object->get_core()->get_m_world_f_core_PSI();
    IVP_U_Matrix m_Rcs_f_Acs; m_ws_f_Rcs->mimult4(m_ws_f_Acs, &m_Rcs_f_Acs);
    m_Rfs_f_Rcs.mmult4(&m_Rcs_f_Acs, &m_Afs_f_Acs);
    if (m_Rfs_f_Rcs.rot){
	m_Rfs_f_Rcs.rot->mmult3(&m_Rcs_f_Acs, m_Afs_f_Acs.rot);
    }else{
	m_Afs_f_Acs.rot = NULL;
    }
}

void IVP_Constraint_Local::change_Ros_to_relaxe_constraint() {
    if (m_Rfs_f_Rcs.object) m_Rfs_f_Rcs.object->ensure_in_simulation(); else m_Afs_f_Acs.object->ensure_in_simulation();
    const IVP_U_Matrix *m_ws_f_Rcs; m_ws_f_Rcs = m_Rfs_f_Rcs.object->get_core()->get_m_world_f_core_PSI();
    const IVP_U_Matrix *m_ws_f_Acs; m_ws_f_Acs = m_Afs_f_Acs.object->get_core()->get_m_world_f_core_PSI();
    
    IVP_U_Matrix m_Acs_f_Rcs; m_ws_f_Acs->mimult4(m_ws_f_Rcs, &m_Acs_f_Rcs);
    m_Afs_f_Acs.mmult4(&m_Acs_f_Rcs, &m_Rfs_f_Rcs);
    if (m_Afs_f_Acs.rot){
	m_Afs_f_Acs.rot->mmult3(&m_Acs_f_Rcs, m_Rfs_f_Rcs.rot);
    }else{
	m_Rfs_f_Rcs.rot = NULL;
    }
}
