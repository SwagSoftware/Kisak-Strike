// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#define SORT_MINDIST_ELEMENTS

union IVV_Cluster_Min_Hash_Key {
    struct {
	unsigned int s1:16;
	unsigned int s2:16;
    } spheres;
    unsigned long int key; //lwss - x64 fixes
};


class IVV_Cluster_Min_Hash_Elem;

class IVV_Cluster_Min_Hash_Elem {
public:
    IVV_Cluster_Min_Hash_Elem *next;
    IVP_DOUBLE	value;
#if defined(SORT_MINDIST_ELEMENTS)
    int 	cmp_index;
#endif    
    void 	*elem;
    ~IVV_Cluster_Min_Hash_Elem();
};

class IVV_Cluster_Min_Hash {
    friend class IVV_Cluster_Min_Hash_Enumerator;
    inline int hash_index(const char *key) const;
    static unsigned int crctab[];
    unsigned int	size;
    IVV_Cluster_Min_Hash_Elem **stadel;
    IVV_Cluster_Min_Hash_Elem **min_per_array_pos;
    IVV_Cluster_Min_Hash_Elem **elems;
    void min_added_at_index(IVV_Cluster_Min_Hash_Elem *elem, int i);
    void min_removed_at_index(IVV_Cluster_Min_Hash_Elem *elem, int i);
    
public:
    int			counter;
    
    IVV_Cluster_Min_Hash(int size=256);	// size must be 2^x
    ~IVV_Cluster_Min_Hash();
    
    void add(void *elem,IVP_DOUBLE value);
    void change_value(void *elem,IVP_DOUBLE new_value);
    
    void *find_min_elem(){	if (!stadel[1]) return NULL; else return stadel[1]->elem; };
    IVP_DOUBLE find_min_value(){	return stadel[1]->value; };
    int  is_elem(void *elem);
    void remove(void *elem);
    void remove_min();
};


class IVV_Cluster_Min_Hash_Enumerator {
    IVV_Cluster_Min_Hash *min_hash;
    IVV_Cluster_Min_Hash_Elem *loop_elem;
    int		loop_index;
public:
    IVV_Cluster_Min_Hash_Enumerator( IVV_Cluster_Min_Hash *mh){
	min_hash = mh;
	loop_elem = 0;
	loop_index = -1;
    }
    
    void *get_next_element(){
	if (loop_elem) {
	    loop_elem = loop_elem->next;
	}	
	while(!loop_elem){
	    loop_index++;
	    if (loop_index >= (int)min_hash->size) return NULL;
	    loop_elem = min_hash->elems[loop_index];
	}
	return loop_elem->elem;
    }
    
    void *get_next_element_lt(IVP_DOUBLE max_limit){
	while (1){
	    if (loop_elem) {
		loop_elem = loop_elem->next;
	    }	
	    while(!loop_elem){
		loop_index++;
		if (loop_index >= (int)min_hash->size) return NULL;
		loop_elem = min_hash->elems[loop_index];
	    }
	    if (loop_elem->value < max_limit){
		return loop_elem->elem;
	    }
	}
    }
};






























