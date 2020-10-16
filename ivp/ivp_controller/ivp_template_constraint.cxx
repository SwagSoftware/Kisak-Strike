// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <ivp_template_constraint.hxx>
#include <ivp_constraint_local.hxx>

IVP_Template_Constraint::IVP_Template_Constraint() {
	force_factor = 1.0f;
	damp_factor = 1.0f;

    //type = IVP_CONSTRAINT_LOCAL;
    objectR = NULL; // for easier finding errors
    objectA = NULL;
    m_Ros_f_Rfs = NULL;
    m_Ros_f_Rrs = NULL;
    m_Aos_f_Afs = NULL;
    mm_Ros_f_Rfs.init();
    mm_Ros_f_Rrs.init3();
    mm_Aos_f_Afs.init();
    axis_type[0] = axis_type[1] = axis_type[2] = IVP_CONSTRAINT_AXIS_FIXED;
    axis_type[3] = axis_type[4] = axis_type[5] = IVP_CONSTRAINT_AXIS_FREE;
    borderleft_Rfs[0] = borderleft_Rfs[1] = borderleft_Rfs[2] = borderleft_Rfs[3] = borderleft_Rfs[4] = borderleft_Rfs[5] = 0.0;
    borderright_Rfs[0] = borderright_Rfs[1] = borderright_Rfs[2] = borderright_Rfs[3] = borderright_Rfs[4] = borderright_Rfs[5] = 0.0;
    maximpulse[0] = maximpulse[1] = maximpulse[2] = maximpulse[3] = maximpulse[4] = maximpulse[5] = 0.0;
    maximpulse_type[0] = maximpulse_type[1] = maximpulse_type[2] = maximpulse_type[3] = maximpulse_type[4] = maximpulse_type[5] = IVP_CFE_NONE;
    limited_axis_stiffness = .3f;
}

void IVP_Template_Constraint::set_constraint_Ros(IVP_Real_Object *_objR, const IVP_U_Point *_anchor_Ros, const IVP_U_Point *_known_axis_Ros,
						 unsigned _transdim, unsigned _rotdim, IVP_Real_Object *_objA, const IVP_U_Matrix *m_Rfs_f_Afs) {
    IVP_U_Matrix m_ws_f_Ros;
    if (_objR){
	_objR->get_m_world_f_object_AT(&m_ws_f_Ros);
    }else{
	m_ws_f_Ros.init();
    }
    IVP_U_Matrix m_ws_f_Aos;
    if (_objA){
	_objA->get_m_world_f_object_AT(&m_ws_f_Aos);
    }else{
	m_ws_f_Aos.init();
    }
    IVP_U_Matrix m_Aos_f_Ros; m_ws_f_Aos.mimult4(&m_ws_f_Ros, &m_Aos_f_Ros);
    
    if (_known_axis_Ros || _anchor_Ros) {
	m_Ros_f_Rfs = &mm_Ros_f_Rfs;
	
	// Keep an eye on the transdim/rotdim-combinations!
	if (!_known_axis_Ros) {
	    mm_Ros_f_Rfs.init3();
	} else if (_rotdim == 2 || _transdim == 2) {
	    mm_Ros_f_Rfs.init_normized3_col(_known_axis_Ros, IVP_INDEX_Z);
	} else if (_rotdim == 1 || _transdim == 1) {
	    mm_Ros_f_Rfs.init_normized3_col(_known_axis_Ros, IVP_INDEX_X);
	} else {
	    mm_Ros_f_Rfs.init3();
	}
	
	if (_anchor_Ros)
	    mm_Ros_f_Rfs.vv.set(_anchor_Ros);
	else
	    mm_Ros_f_Rfs.vv.set_to_zero();
    } else
	m_Ros_f_Rfs = NULL;

    m_Ros_f_Rrs = NULL;
    
    if (m_Rfs_f_Afs) {
	m_Aos_f_Afs = &mm_Aos_f_Afs;
	IVP_U_Matrix m_Aos_f_Rfs; m_Aos_f_Ros.mmult4(m_Ros_f_Rfs, &m_Aos_f_Rfs);
        m_Aos_f_Rfs.mmult4(m_Rfs_f_Afs, m_Aos_f_Afs);
    } else
	m_Aos_f_Afs = NULL;
    
    objectR = _objR;
    objectA = _objA;

    int transdim = _transdim;
    int rotdim = _rotdim;
    axis_type[0] = transdim-- > 0 ? IVP_CONSTRAINT_AXIS_FIXED : IVP_CONSTRAINT_AXIS_FREE;
    axis_type[1] = transdim-- > 0 ? IVP_CONSTRAINT_AXIS_FIXED : IVP_CONSTRAINT_AXIS_FREE;
    axis_type[2] = transdim-- > 0 ? IVP_CONSTRAINT_AXIS_FIXED : IVP_CONSTRAINT_AXIS_FREE;
    axis_type[3] = rotdim-- > 0 ? IVP_CONSTRAINT_AXIS_FIXED : IVP_CONSTRAINT_AXIS_FREE;
    axis_type[4] = rotdim-- > 0 ? IVP_CONSTRAINT_AXIS_FIXED : IVP_CONSTRAINT_AXIS_FREE;
    axis_type[5] = rotdim-- > 0 ? IVP_CONSTRAINT_AXIS_FIXED : IVP_CONSTRAINT_AXIS_FREE;
}

void IVP_Template_Constraint::set_constraint_ws(IVP_Real_Object *_objR, const IVP_U_Point *_anchor_ws, const IVP_U_Point *_known_axis_ws,
						unsigned _transdim, unsigned _rotdim, IVP_Real_Object *_objA, const IVP_U_Matrix *m_Rfs_f_Afs) {
    IVP_U_Matrix m_ws_f_Ros;
    if (_objR){
	_objR->get_m_world_f_object_AT(&m_ws_f_Ros);
    }else{
	m_ws_f_Ros.init();
    }
    IVP_U_Point point_Ros;
    if (_anchor_ws){
	m_ws_f_Ros.vimult4(_anchor_ws, &point_Ros);
    }else{
	point_Ros.set_to_zero();
    }
    IVP_U_Point known_axis_Ros;
    if (_known_axis_ws){
	m_ws_f_Ros.vimult3(_known_axis_ws, &known_axis_Ros);
    }else{
	known_axis_Ros.set_to_zero();
    }
    set_constraint_Ros(_objR, &point_Ros, &known_axis_Ros, _transdim, _rotdim, _objA, m_Rfs_f_Afs);
}

void IVP_Template_Constraint::set_stiffness_for_limited_axis(IVP_FLOAT stiffness){
    limited_axis_stiffness = stiffness;
}

// sort the coordinate-systems from tmpl into this.
void IVP_Template_Constraint::sort_coordinates(const IVP_Template_Constraint &tmpl) {
    // Translation matrix still missing...
    int pos = 0;
    int i;
    m_Ros_f_Rfs = &mm_Ros_f_Rfs;
    m_Aos_f_Afs = &mm_Aos_f_Afs;
    m_Ros_f_Rrs = &mm_Ros_f_Rrs;
    // Translation...
    for (i = 0; i < 3; i++) if (tmpl.axis_type[i] == IVP_CONSTRAINT_AXIS_FIXED) {
	if (tmpl.m_Ros_f_Rfs) {
	    IVP_U_Point p;	    tmpl.m_Ros_f_Rfs->get_col(IVP_COORDINATE_INDEX(i), &p);
	    m_Ros_f_Rfs->set_col( IVP_COORDINATE_INDEX(pos), &p);
	}
	if (tmpl.m_Aos_f_Afs) {
	    IVP_U_Point p;	    tmpl.m_Aos_f_Afs->get_col(IVP_COORDINATE_INDEX(i), &p);
	    m_Aos_f_Afs->set_col( IVP_COORDINATE_INDEX(pos), &p);
	}
        borderleft_Rfs[pos] = tmpl.borderleft_Rfs[i];
        borderright_Rfs[pos] = tmpl.borderright_Rfs[i];
	axis_type[pos] = tmpl.axis_type[i];
	maximpulse[pos] = tmpl.maximpulse[i];
	maximpulse_type[pos] = tmpl.maximpulse_type[i];
	borderleft_Rfs[pos + 3] = tmpl.borderleft_Rfs[i + 3];
	borderright_Rfs[pos + 3] = tmpl.borderright_Rfs[i + 3];
	axis_type[pos + 3] = tmpl.axis_type[i + 3];
	maximpulse[pos + 3] = tmpl.maximpulse[i + 3];
	maximpulse_type[pos + 3] = tmpl.maximpulse_type[i + 3];
	pos++;
    }
    for (i = 0; i < 3; i++) if (tmpl.axis_type[i] == IVP_CONSTRAINT_AXIS_LIMITED) {
	if (tmpl.m_Ros_f_Rfs) {
	    IVP_U_Point p;	    tmpl.m_Ros_f_Rfs->get_col(IVP_COORDINATE_INDEX(i), &p);
	    m_Ros_f_Rfs->set_col( IVP_COORDINATE_INDEX(pos), &p);
	}
	if (tmpl.m_Aos_f_Afs) {
	    IVP_U_Point p;	    tmpl.m_Aos_f_Afs->get_col(IVP_COORDINATE_INDEX(i), &p);
	    m_Aos_f_Afs->set_col( IVP_COORDINATE_INDEX(pos), &p);
	}
        borderleft_Rfs[pos] = tmpl.borderleft_Rfs[i];
        borderright_Rfs[pos] = tmpl.borderright_Rfs[i];

	axis_type[pos] = tmpl.axis_type[i];
	maximpulse[pos] = tmpl.maximpulse[i];
	maximpulse_type[pos] = tmpl.maximpulse_type[i];
	borderleft_Rfs[pos + 3] = tmpl.borderleft_Rfs[i + 3];
	borderright_Rfs[pos + 3] = tmpl.borderright_Rfs[i + 3];
	axis_type[pos + 3] = tmpl.axis_type[i + 3];
	maximpulse[pos + 3] = tmpl.maximpulse[i + 3];
	maximpulse_type[pos + 3] = tmpl.maximpulse_type[i + 3];
	pos++;
    }
    for (i = 0; i < 3; i++) if (tmpl.axis_type[i] == IVP_CONSTRAINT_AXIS_FREE) {
	if (tmpl.m_Ros_f_Rfs) {
	    IVP_U_Point p;	    tmpl.m_Ros_f_Rfs->get_col(IVP_COORDINATE_INDEX(i), &p);
	    m_Ros_f_Rfs->set_col( IVP_COORDINATE_INDEX(pos), &p);
	}
	if (tmpl.m_Aos_f_Afs) {
	    IVP_U_Point p;	    tmpl.m_Aos_f_Afs->get_col(IVP_COORDINATE_INDEX(i), &p);
	    m_Aos_f_Afs->set_col( IVP_COORDINATE_INDEX(pos), &p);
	}
        borderleft_Rfs[pos] = tmpl.borderleft_Rfs[i];
        borderright_Rfs[pos] = tmpl.borderright_Rfs[i];
	axis_type[pos] = tmpl.axis_type[i];
	maximpulse[pos] = tmpl.maximpulse[i];
	maximpulse_type[pos] = tmpl.maximpulse_type[i];
	borderleft_Rfs[pos + 3] = tmpl.borderleft_Rfs[i + 3];
	borderright_Rfs[pos + 3] = tmpl.borderright_Rfs[i + 3];
	axis_type[pos + 3] = tmpl.axis_type[i + 3];
	maximpulse[pos + 3] = tmpl.maximpulse[i + 3];
	maximpulse_type[pos + 3] = tmpl.maximpulse_type[i + 3];
	pos++;
    }
    if (tmpl.m_Ros_f_Rfs) {
	IVP_U_Point col0;	m_Ros_f_Rfs->get_col(IVP_INDEX_X,&col0);
	IVP_U_Point col1;	m_Ros_f_Rfs->get_col(IVP_INDEX_Y,&col1);
	IVP_U_Point cross;	cross.calc_cross_product(&col0, &col1);
	m_Ros_f_Rfs->set_col(IVP_INDEX_Z, &cross);
	
        m_Ros_f_Rfs->vv.set(&tmpl.m_Ros_f_Rfs->vv);
    } else
	m_Ros_f_Rfs = NULL;
    if (tmpl.m_Aos_f_Afs) {
	IVP_U_Point col0;	m_Aos_f_Afs->get_col(IVP_INDEX_X,&col0);
	IVP_U_Point col1;	m_Aos_f_Afs->get_col(IVP_INDEX_Y,&col1);
	IVP_U_Point cross;	cross.calc_cross_product(&col0, &col1);
	m_Aos_f_Afs->set_col(IVP_INDEX_Z, &cross);
	m_Aos_f_Afs->vv.set(&tmpl.m_Aos_f_Afs->vv);
    } else
	m_Aos_f_Afs = NULL;
    
    // Rotation...
    if (tmpl.m_Ros_f_Rrs) {
	pos = 0;
	for (i = 0; i < 3; i++) if (tmpl.axis_type[i + 3] == IVP_CONSTRAINT_AXIS_FIXED) {
	    IVP_U_Point p;	    tmpl.m_Ros_f_Rrs->get_col(IVP_COORDINATE_INDEX(i), &p);
	    m_Ros_f_Rrs->set_col( IVP_COORDINATE_INDEX(pos), &p);

	    borderleft_Rfs[pos + 3] = tmpl.borderleft_Rfs[i + 3];
	    borderright_Rfs[pos + 3] = tmpl.borderright_Rfs[i + 3];
	    axis_type[pos + 3] = tmpl.axis_type[i + 3];
	    maximpulse[pos + 3] = tmpl.maximpulse[i + 3];
	    maximpulse_type[pos + 3] = tmpl.maximpulse_type[i + 3];
	    pos++;
	}
	for (i = 0; i < 3; i++) if (tmpl.axis_type[i + 3] == IVP_CONSTRAINT_AXIS_LIMITED) {
	    IVP_U_Point p;	    tmpl.m_Ros_f_Rrs->get_col(IVP_COORDINATE_INDEX(i), &p);
	    m_Ros_f_Rrs->set_col( IVP_COORDINATE_INDEX(pos), &p);
	    borderleft_Rfs[pos + 3] = tmpl.borderleft_Rfs[i + 3];
	    borderright_Rfs[pos + 3] = tmpl.borderright_Rfs[i + 3];
	    axis_type[pos + 3] = tmpl.axis_type[i + 3];
	    maximpulse[pos + 3] = tmpl.maximpulse[i + 3];
	    maximpulse_type[pos + 3] = tmpl.maximpulse_type[i + 3];
	    pos++;
	}
	for (i = 0; i < 3; i++) if (tmpl.axis_type[i + 3] == IVP_CONSTRAINT_AXIS_FREE) {
	    IVP_U_Point p;	    tmpl.m_Ros_f_Rrs->get_col(IVP_COORDINATE_INDEX(i), &p);
	    m_Ros_f_Rrs->set_col( IVP_COORDINATE_INDEX(pos), &p);
	    borderleft_Rfs[pos + 3] = tmpl.borderleft_Rfs[i + 3];
	    borderright_Rfs[pos + 3] = tmpl.borderright_Rfs[i + 3];
	    axis_type[pos + 3] = tmpl.axis_type[i + 3];
	    maximpulse[pos + 3] = tmpl.maximpulse[i + 3];
	    maximpulse_type[pos + 3] = tmpl.maximpulse_type[i + 3];
	    pos++;
	}
	{
	    IVP_U_Point col0;	m_Ros_f_Rrs->get_col(IVP_INDEX_X,&col0);
	    IVP_U_Point col1;	m_Ros_f_Rrs->get_col(IVP_INDEX_Y,&col1);
	    IVP_U_Point cross;	cross.calc_cross_product(&col0, &col1);
	    m_Ros_f_Rrs->set_col(IVP_INDEX_Z, &cross);
	}
    } else
	m_Ros_f_Rrs = NULL;
    
}



void IVP_Template_Constraint::set_reference_object(IVP_Real_Object *objR) {
    objectR = objR;
}

void IVP_Template_Constraint::set_attached_object(IVP_Real_Object *objA) {
    objectA = objA;
}

void IVP_Template_Constraint::set_fixing_point_Ros(const IVP_U_Point *anchor) { // point_Ros_f_uRfs
    m_Ros_f_Rfs = &mm_Ros_f_Rfs;
    m_Ros_f_Rfs->vv.set(anchor);
}

void IVP_Template_Constraint::set_translation_axes_Ros(const IVP_U_Matrix3 *trans_axes) { // m_Ros_f_uRfs
    m_Ros_f_Rfs = &mm_Ros_f_Rfs;
    *(IVP_U_Matrix3 *) &mm_Ros_f_Rfs = *trans_axes;
}


void IVP_Template_Constraint::set_translation_axes_as_object_space() {
    m_Ros_f_Rfs = NULL;
}

void IVP_Template_Constraint::fix_translation_axis(IVP_COORDINATE_INDEX which) {
    axis_type[which] = IVP_CONSTRAINT_AXIS_FIXED;
}

void IVP_Template_Constraint::free_translation_axis(IVP_COORDINATE_INDEX which) {
    axis_type[which] = IVP_CONSTRAINT_AXIS_FREE;
}

void IVP_Template_Constraint::limit_translation_axis(IVP_COORDINATE_INDEX which, IVP_FLOAT border_left, IVP_FLOAT border_right) {
    axis_type[which] = IVP_CONSTRAINT_AXIS_LIMITED;
    borderleft_Rfs[which] = border_left;
    borderright_Rfs[which] = border_right;
}

void IVP_Template_Constraint::set_max_translation_impulse(IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse) {
    maximpulse[0] = maximpulse[1] = maximpulse[2] = impulse;
    maximpulse_type[0] = maximpulse_type[1] = maximpulse_type[2] = impulsetype;
}

void IVP_Template_Constraint::set_max_translation_impulse(IVP_COORDINATE_INDEX coord, IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse) {
    maximpulse[coord] = impulse;
    maximpulse_type[coord] = impulsetype;
}

void IVP_Template_Constraint::set_rotation_axes_Ros(const IVP_U_Matrix3 *rot_axes) {
    m_Ros_f_Rrs = &mm_Ros_f_Rrs;
    mm_Ros_f_Rrs = *rot_axes;
}

void IVP_Template_Constraint::set_rotation_axes_as_translation_axes() {
    m_Ros_f_Rrs = NULL;
}

void IVP_Template_Constraint::fix_rotation_axis(IVP_COORDINATE_INDEX which) {
    axis_type[which + 3] = IVP_CONSTRAINT_AXIS_FIXED;
}

void IVP_Template_Constraint::free_rotation_axis(IVP_COORDINATE_INDEX which) {
    axis_type[which + 3] = IVP_CONSTRAINT_AXIS_FREE;
}

void IVP_Template_Constraint::limit_rotation_axis(IVP_COORDINATE_INDEX which, IVP_FLOAT border_left, IVP_FLOAT border_right) {
    axis_type[which + 3] = IVP_CONSTRAINT_AXIS_LIMITED;
    borderleft_Rfs[which + 3] = border_left;
    borderright_Rfs[which + 3] = border_right;
}

void IVP_Template_Constraint::set_max_rotation_impulse(IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse) {
    maximpulse[3] = maximpulse[4] = maximpulse[5] = impulse;
    maximpulse_type[3] = maximpulse_type[4] = maximpulse_type[5] = impulsetype;
}

void IVP_Template_Constraint::set_max_rotation_impulse(IVP_COORDINATE_INDEX coord, IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse) {
    maximpulse[coord + 3] = impulse;
    maximpulse_type[coord + 3] = impulsetype;
}

void IVP_Template_Constraint::set_attached_translation_axes_Aos(const IVP_U_Matrix3 *rot_attached) {
    m_Aos_f_Afs = &mm_Aos_f_Afs;
    *(IVP_U_Matrix3 *) &mm_Aos_f_Afs = *rot_attached;
}

void IVP_Template_Constraint::set_attached_fixing_point_Aos(const IVP_U_Point *trans_attached) {
    m_Aos_f_Afs = &mm_Aos_f_Afs;
    m_Aos_f_Afs->vv.set(trans_attached);
}

void IVP_Template_Constraint::set_constraint_is_relaxed() {
    m_Aos_f_Afs = NULL;
}

IVP_Constraint *IVP_Environment::create_constraint(const IVP_Template_Constraint *tmpl) {
    if (!tmpl->objectR && !tmpl->objectA)
	return NULL;
        
    IVP_Constraint *newconstraint = new IVP_Constraint_Local(*tmpl);
    
    return newconstraint;
}
