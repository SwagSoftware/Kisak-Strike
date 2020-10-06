// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#include <ivp_constraint.hxx>
#include <ivp_template_constraint.hxx>

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void IVP_Constraint::core_is_going_to_be_deleted_event(IVP_Core *) {
    P_DELETE_THIS(this);
}

IVP_Constraint::IVP_Constraint() {
    is_enabled = IVP_TRUE;
}

IVP_DOUBLE IVP_Constraint::get_minimum_simulation_frequency(){
    //return get_objectF()->get_core()->environment->get_delta_PSI_time();
    return 1.0f;
}

IVP_Environment *IVP_Constraint::get_environment() {
    return cores_of_constraint_system.element_at(0)->environment;
}

void IVP_Constraint::activate() {
    if (!is_enabled) {
	is_enabled = IVP_TRUE;
	get_environment()->get_controller_manager()->announce_controller_to_environment(this);
    }
}

void IVP_Constraint::deactivate() {
    if (is_enabled) {
	is_enabled = IVP_FALSE;
	get_environment()->get_controller_manager()->remove_controller_from_environment(this, IVP_FALSE);
    }
}

IVP_Constraint::~IVP_Constraint() {
    //get_environment().get_controller_manager()->remove_controller_from_environment(this, IVP_FALSE);
    if (is_enabled) {
	IVP_Controller_Manager::remove_controller_from_environment(this,IVP_TRUE); //delete silently
    }
}

void IVP_Constraint::change_fixing_point_Ros(const IVP_U_Point *) { printf("You are changing the fixing point of a constraint which does not exist.\n");}
void IVP_Constraint::change_target_fixing_point_Ros(const IVP_U_Point *) { printf("You are changing the fixing point of a constraint which does not exist.\n");}
void IVP_Constraint::change_translation_axes_Ros(const IVP_U_Matrix3 * /*m_Ros_f_Rfs*/) {printf("You are changing the translation axes of a constraint which does not exist.\n");}
void IVP_Constraint::change_target_translation_axes_Ros(const IVP_U_Matrix3 *m_Ros_f_Rfs) {IVP_USE(m_Ros_f_Rfs); printf("You are changing the translation axes of a constraint which does not exist.\n");}
void IVP_Constraint::fix_translation_axis(IVP_COORDINATE_INDEX which) {IVP_USE(which); printf("You are fixing a translation axis of a constraint which does not exist.\n");}
void IVP_Constraint::free_translation_axis(IVP_COORDINATE_INDEX which) {IVP_USE(which); printf("You are freeing a translation axis of a constraint which does not exist.\n");}
void IVP_Constraint::limit_translation_axis(IVP_COORDINATE_INDEX which, IVP_FLOAT border_left, IVP_FLOAT border_right) {IVP_USE(which); IVP_USE(border_left); IVP_USE(border_right); printf("You are limiting a translation axis of a constraint which does not exist.\n");}
void IVP_Constraint::change_max_translation_impulse(IVP_CONSTRAINT_FORCE_EXCEED forcetype, IVP_FLOAT force) {IVP_USE(forcetype); IVP_USE(force); printf("You are changing a maxforce define of a constraint which does not exist.\n");}
void IVP_Constraint::change_rotation_axes_Ros(const IVP_U_Matrix3 *rot_axes) {IVP_USE(rot_axes); printf("You are changing the rotation axes of a constraint which does not exist.\n");}
void IVP_Constraint::change_target_rotation_axes_Ros(const IVP_U_Matrix3 *rot_axes) {IVP_USE(rot_axes); printf("You are changing the rotation axes of a constraint which does not exist.\n");}
void IVP_Constraint::fix_rotation_axis(IVP_COORDINATE_INDEX which) {IVP_USE(which); printf("You are fixing a rotation axis of a constraint which does not exist.\n");}
void IVP_Constraint::free_rotation_axis(IVP_COORDINATE_INDEX which) {IVP_USE(which); printf("You are freeing a rotation axis of a constraint which does not exist.\n");}
void IVP_Constraint::limit_rotation_axis(IVP_COORDINATE_INDEX which, IVP_FLOAT border_left, IVP_FLOAT border_right) {IVP_USE(which); IVP_USE(border_left); IVP_USE(border_right); printf("You are limiting a rotation axis of a constraint which does not exist.\n");}
void IVP_Constraint::change_max_rotation_impulse(IVP_CONSTRAINT_FORCE_EXCEED forcetype, IVP_FLOAT force) {IVP_USE(forcetype); IVP_USE(force); printf("You are changing a maxtorque define of a constraint which does not exist.\n");}

void IVP_Constraint::change_Aos_to_relaxe_constraint() {printf("You are repositioning a constraint which does not exist.\n");}
void IVP_Constraint::change_Ros_to_relaxe_constraint() {printf("You are repositioning a constraint which does not exist.\n");}
