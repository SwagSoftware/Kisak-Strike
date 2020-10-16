// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#ifndef WIN32
#	pragma implementation "ivp_material.hxx"
#endif
#include <ivp_material.hxx>


/////// combine both (combined) material props in a contact situation

IVP_DOUBLE IVP_Material_Manager::get_friction_factor(IVP_Contact_Situation *sit)
{
    return sit->materials[0]->get_friction_factor() *
	   sit->materials[1]->get_friction_factor();
}

IVP_DOUBLE IVP_Material_Manager::get_elasticity(IVP_Contact_Situation *sit)
{
    return sit->materials[0]->get_elasticity() *
	   sit->materials[1]->get_elasticity();
}

IVP_DOUBLE IVP_Material_Manager::get_adhesion(IVP_Contact_Situation *sit)
{
    return sit->materials[0]->get_adhesion() +
	   sit->materials[1]->get_adhesion();
}

IVP_Material_Manager::IVP_Material_Manager(IVP_BOOL delete_on_env_delete_in){
    delete_on_env_delete = delete_on_env_delete_in;
}

IVP_Material::~IVP_Material(){

}

IVP_Material *IVP_Material_Manager::get_material_by_index(const IVP_U_Point * /*world_position*/, int /*index*/){ // returns the material, if material index in compact_triangle != 0
  static IVP_Material_Simple *simple = new IVP_Material_Simple(0.5f,0.5f);
  return simple;
}

IVP_DOUBLE IVP_Material_Simple::get_friction_factor()
{
    return friction_value;
}

IVP_DOUBLE IVP_Material_Simple::get_elasticity()
{
    return elasticity;
}

// INTERN_START
IVP_DOUBLE IVP_Material_Simple::get_adhesion()
{
    return adhesion;
}
// INTERN_END

IVP_Material_Simple::IVP_Material_Simple(IVP_DOUBLE friction, IVP_DOUBLE elas){
    friction_value = friction;
    elasticity = elas;
    adhesion = 0.0f;
}

IVP_Material_Simple::~IVP_Material_Simple(){

}

IVP_Material_Simple::IVP_Material_Simple(){
    friction_value = 0.0f;
    elasticity = 0.5f;
    adhesion = 0.0f;

}
const char *IVP_Material_Simple::get_name(){
    return "Simple material";
}
