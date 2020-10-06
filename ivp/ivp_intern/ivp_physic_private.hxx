// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef WIN32
#	pragma interface
#endif

class IVP_Cluster;
class IVP_Environment;
class IVP_Core;
class IVP_Core_Collision;
class IVP_Mindist;
class IVP_Object_Callback_Table_Hash;
class IVP_Collision_Callback_Table_Hash;
class IVP_Listener_Object;
class IVP_Event_Object;

#define IVP_DEBRIS_DELAY_NUM 10  // possible extra lookahead is -2


class IVP_Cluster_Manager {
    IVP_Cluster *root_cluster;
    IVP_Environment *environment;

    IVP_Object_Callback_Table_Hash *obj_callback_hash;
    IVP_Collision_Callback_Table_Hash *collision_callback_hash;

    IVP_Real_Object *an_object_to_be_checked; //_by_send_unused_objects_to_universe_manager;
    IVP_Real_Object *get_next_real_object_in_cluster_tree(IVP_Object *object); // returns an_object_to_be_checked++

    int number_of_real_objects;

public:    
    IVP_Cluster *get_root_cluster();
    IVP_Cluster_Manager(IVP_Environment *env);
    ~IVP_Cluster_Manager();

    void check_for_unused_objects(class IVP_Universe_Manager *um);

    void fire_event_object_created(IVP_Event_Object *event_obj);
    void fire_event_object_deleted(IVP_Event_Object *event_obj);
    void fire_event_object_frozen (IVP_Event_Object *event_obj);
    void fire_event_object_revived(IVP_Event_Object *event_obj);


    void add_listener_object(IVP_Real_Object *real_object, IVP_Listener_Object *listener);
    void remove_listener_object(IVP_Real_Object *real_object, IVP_Listener_Object *listener);



    void fire_event_pre_collision(IVP_Real_Object *ro, IVP_Event_Collision *event_obj);
    void fire_event_post_collision(IVP_Real_Object *ro, IVP_Event_Collision *event_obj);
    void fire_event_collision_object_deleted(IVP_Real_Object *ro);
    void fire_event_friction_created(IVP_Real_Object *real_object, class IVP_Event_Friction *event_friction);
    void fire_event_friction_deleted(IVP_Real_Object *real_object, class IVP_Event_Friction *event_friction);


    void add_listener_collision(IVP_Real_Object *real_object, IVP_Listener_Collision *listener);
    void remove_listener_collision(IVP_Real_Object *real_object, IVP_Listener_Collision *listener);

	
    void add_object(IVP_Real_Object *real_object);
    void remove_object(IVP_Real_Object *real_object); // tells the Cluster Manager that an object is going to be deleted
};









