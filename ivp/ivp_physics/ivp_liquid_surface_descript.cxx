// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#ifndef WIN32
#	pragma implementation "ivp_liquid_surface_descript.hxx"
#endif

#include <ivp_physics.hxx>
#include <ivp_liquid_surface_descript.hxx>


void IVP_Liquid_Surface_Descriptor_Simple::calc_liquid_surface( IVP_Environment * /*environment*/,
								IVP_Core * /*core*/,
								IVP_U_Float_Hesse *surface_normal_out,
								IVP_U_Float_Point *abs_speed_of_current_out) {
    surface_normal_out->set(&surface);
    surface_normal_out->hesse_val = surface.hesse_val;
    abs_speed_of_current_out->set(&abs_speed_of_current);
}


IVP_Liquid_Surface_Descriptor_Simple::IVP_Liquid_Surface_Descriptor_Simple(const IVP_U_Float_Hesse *surface_in, const IVP_U_Float_Point *abs_speed_of_current_in) {
    surface.set(surface_in);
    surface.hesse_val = surface_in->hesse_val;
    abs_speed_of_current.set(abs_speed_of_current_in);
}
