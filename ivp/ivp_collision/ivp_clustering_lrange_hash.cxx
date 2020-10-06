// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#include <ivu_vhash.hxx>
#include <ivp_clustering_longrange.hxx>
#include <ivp_clustering_lrange_hash.hxx>


IVP_ov_tree_hash::~IVP_ov_tree_hash() {;}

int IVP_ov_tree_hash::node_to_index(IVP_OV_Node *node)
{
    return hash_index( (char *)&node->data, sizeof(node->data));
}


IVP_BOOL IVP_ov_tree_hash::compare(void *elem0, void *elem1) const
{
    IVP_OV_Node *node0 = (IVP_OV_Node *)elem0;
    IVP_OV_Node *node1 = (IVP_OV_Node *)elem1;

    if ( node0->data.rasterlevel != node1->data.rasterlevel) return(IVP_FALSE);
    
    if ( node0->data.x != node1->data.x) return(IVP_FALSE);
    if ( node0->data.y != node1->data.y) return(IVP_FALSE);
    if ( node0->data.z != node1->data.z) return(IVP_FALSE);

    return(IVP_TRUE);
}





