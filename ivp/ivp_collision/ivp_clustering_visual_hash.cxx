// Copyright (C) Ipion Software GmbH 2000. All rights reserved.

// IVP_EXPORT_PRIVATE

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/ 


// IVP includes
#include <ivp_physics.hxx>
#include <ivu_vhash.hxx>

#include <ivp_clustering_visual_hash.hxx>


/*******************************************************************************
 *******************************************************************************
 *
 *  CLUSTERING VISUALIZER OBJECT HASH - METHODS
 *
 *******************************************************************************
 ******************************************************************************/

IVP_BOOL IVP_Clustering_Visualizer_Object_Hash::compare(void *elem0, void *elem1) const {

    IVP_Real_Object *obj0 = (IVP_Real_Object *)elem0;
    IVP_Real_Object *obj1 = (IVP_Real_Object *)elem1;
    
    if ( obj0 != obj1 ) return(IVP_FALSE);
    
    return(IVP_TRUE);
}

int IVP_Clustering_Visualizer_Object_Hash::obj_to_index(IVP_Real_Object *obj) {

    return hash_index( (char *)obj, sizeof(obj));
};

void IVP_Clustering_Visualizer_Object_Hash::add(IVP_Real_Object *obj) {

    add_elem(obj, obj_to_index(obj));
}

IVP_Real_Object *IVP_Clustering_Visualizer_Object_Hash::remove(IVP_Real_Object *obj) {

    return (IVP_Real_Object *)remove_elem(obj, obj_to_index(obj));
}

IVP_Real_Object *IVP_Clustering_Visualizer_Object_Hash::find(IVP_Real_Object *obj) {

    return (IVP_Real_Object *)find_elem(obj, obj_to_index(obj));
}

IVP_Clustering_Visualizer_Object_Hash::IVP_Clustering_Visualizer_Object_Hash(int create_size) : IVP_VHash(create_size) {

    return;
}

IVP_Clustering_Visualizer_Object_Hash::~IVP_Clustering_Visualizer_Object_Hash() {

    return;
}


