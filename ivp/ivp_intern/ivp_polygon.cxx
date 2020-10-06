// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.



#ifndef WIN32
#	pragma implementation "ivp_polygon.hxx"
#endif

#include <ivp_physics.hxx>


#include <ivp_templates.hxx>
#include <ivp_listener_object.hxx>



IVP_Polygon::IVP_Polygon(IVP_Cluster *cluster, IVP_SurfaceManager *surface_manager_, const IVP_Template_Real_Object *templ,
     const IVP_U_Quat *q_world_f_obj, const IVP_U_Point *position)
    : IVP_Real_Object(cluster, surface_manager_, templ, q_world_f_obj, position)
{
  this->set_type(IVP_POLYGON);
  this->extra_radius = templ->extra_radius;
  this->init_object_core( cluster->get_environment(), templ);
    
  {	// fire global 'object created' event
    IVP_Event_Object event_created;
    event_created.real_object = this;
    event_created.environment = physical_core->environment;
    physical_core->environment->fire_event_object_created(&event_created);
  }
}

