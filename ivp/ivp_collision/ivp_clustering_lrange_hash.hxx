// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PROTECTED

class IVP_ov_tree_hash : protected IVP_VHash
{
protected:
    IVP_BOOL compare(void *elem0, void *elem1) const;
    int      node_to_index(IVP_OV_Node *node);

public:
    void add_node(IVP_OV_Node *node)
    {
	add_elem(node, node_to_index(node));
    };

    IVP_OV_Node *remove_node(IVP_OV_Node *node)
    {
	return (IVP_OV_Node *)remove_elem(node, node_to_index(node));
    };

    IVP_OV_Node *find_node(IVP_OV_Node *node)
    {
	return (IVP_OV_Node *)find_elem(node, node_to_index(node));
    };
  

    ~IVP_ov_tree_hash();
    IVP_ov_tree_hash(int create_size) : IVP_VHash(create_size) {;};
};


