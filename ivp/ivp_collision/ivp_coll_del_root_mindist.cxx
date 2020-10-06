#if !defined(WIN32)
#   pragma implementation "ivp_collision.hxx"
#endif

#include <ivp_physics.hxx>
#include <ivp_mindist.hxx>
#include <ivp_mindist_intern.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_clustering_longrange.hxx>

void IVP_Collision_Delegator_Root_Mindist::object_is_removed_from_collision_detection(IVP_Real_Object *o){
  IVP_OV_Element *ov = o->get_ov_element();
  for (int i= ov->collision_fvector.len()-1; i>=0;i--){
      IVP_Collision *coll = ov->collision_fvector.element_at(i);
      coll->delegator_is_going_to_be_deleted_event(this); // deletes oo_collision removes itself from this hash
  }
}

void IVP_Collision_Delegator_Root_Mindist::environment_is_going_to_be_deleted_event(IVP_Environment *){
  P_DELETE_THIS(this);
}

void IVP_Collision_Delegator_Root_Mindist::collision_is_going_to_be_deleted_event(class IVP_Collision *t){
  IVP_Real_Object *objs[2];
  t->get_objects(objs);
  IVP_OV_Element *ov0 = objs[0]->get_ov_element();
  IVP_OV_Element *ov1 = objs[1]->get_ov_element();
  ov0->remove_oo_collision(t);
  ov1->remove_oo_collision(t);
}


IVP_Collision *IVP_Collision_Delegator_Root_Mindist::delegate_collisions_for_object(IVP_Real_Object *obj0, IVP_Real_Object *obj1) {

  IVP_Collision *coll;
#if 1
  const IVP_Compact_Ledge *l0 = obj0->get_surface_manager()->get_single_convex();
  const IVP_Compact_Ledge *l1 = obj1->get_surface_manager()->get_single_convex();
  
  if (  l0 && l1 ){
    const IVP_Compact_Triangle *tri0 = l0->get_first_triangle();
    const IVP_Compact_Triangle *tri1 = l1->get_first_triangle();

    IVP_Environment *env = obj0->get_environment();

    IVP_Mindist *md; // #+# check for bounding boxes using get_all_ledges_within_radius
    if ( !l0->is_terminal() || !l1->is_terminal()){
	md = new IVP_Mindist_Recursive(env, this);
    }else{
	md = new IVP_Mindist(env, this);
    }
    md->init_mindist(obj0, obj1, tri0->get_first_edge(),tri1->get_first_edge());
    coll = md;
  }else{      
    coll = new IVP_OO_Watcher(this,obj0,obj1);
  }
  
#else
    coll = new IVP_OO_Watcher(this,obj0,obj1);
#endif  
  
  IVP_OV_Element *ov0 = obj0->get_ov_element();
  IVP_OV_Element *ov1 = obj1->get_ov_element();
  ov0->add_oo_collision(coll);
  ov1->add_oo_collision(coll);
  return coll;
}

IVP_Collision_Delegator_Root_Mindist::~IVP_Collision_Delegator_Root_Mindist(){


}

IVP_Collision_Delegator_Root_Mindist::IVP_Collision_Delegator_Root_Mindist(){
  P_MEM_CLEAR_M4(this);
}
