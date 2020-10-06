// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include "ivv_cluster_min_hash.hxx"

extern unsigned int IVP_Hash_crctab[];

inline int IVV_Cluster_Min_Hash::hash_index(const char *key)const{
	unsigned int c;		
	unsigned int index = 0xffffffffL;
	int i;
	for (i=sizeof(void *)-1;i>=0;i--){
	    c = *((unsigned char *)(key++));
	    index = IVP_Hash_crctab[((int) index ^ c) & 0xff] ^ (index >> 8);
	}
	index = index & (size-1);
	return index;
    };

IVV_Cluster_Min_Hash::IVV_Cluster_Min_Hash(int sizei){
    size = sizei;
    int i;
    for (i=0;i<32;i++){
	if ( (1<<i) == sizei) break;
    }
    IVP_ASSERT (i!=32); // must be power 2
    
    elems = (IVV_Cluster_Min_Hash_Elem **)p_calloc(sizeof(void *),size);
    stadel = (IVV_Cluster_Min_Hash_Elem **)p_calloc(sizeof(void *),size*2);
    min_per_array_pos = stadel + size;
    counter = 0;
}

IVV_Cluster_Min_Hash_Elem::~IVV_Cluster_Min_Hash_Elem(){
    delete next;
}

IVV_Cluster_Min_Hash::~IVV_Cluster_Min_Hash(){
    unsigned int i;
    for (i=0;i<size;i++){
	if (!elems[i]) continue;
	delete elems[i];
    }
    P_FREE(elems);
    P_FREE(stadel);
}

/**
 *     	elems:		array of unsorted linked list
 *	min_per_array_pos	parallel array of minimal elements per linked list
 *
 *	stadel:		binary tree of minimal elements
 *			stadel[1] == root
 *			stadel[2-3] next level
 *			stadel[4-7] ""
 *			...
 *			stadel[size/2 - size-1]
 *			stadel[size - 2size-1] == min_per_array_pos[0 .. size-1] (same memory)

 */

// a new minimum element at a given hash index was set, so update stadel
void  IVV_Cluster_Min_Hash::min_added_at_index(IVV_Cluster_Min_Hash_Elem *elem, int i){
    unsigned int height;
    IVP_DOUBLE val = elem->value;
    i += size;
    stadel[i] = elem;	// transport new min element from bottom to top till smaller element is found
    for (height = size>>1;height > 0;height>>=1){		// stadel, go from bottom to to top
	i = i>>1;
	if (stadel[i] && val >= stadel[i]->value){
	    break;
	}
	stadel[i] = elem;
    }
}

// the minimum element at an array position has been removed
// search new minimum in linked list.

void IVV_Cluster_Min_Hash::min_removed_at_index(IVV_Cluster_Min_Hash_Elem *elem, int i){
    {	// search hash index position for new minimal element
	IVV_Cluster_Min_Hash_Elem *new_min = elems[i];
	IVV_Cluster_Min_Hash_Elem *el;
	el = new_min;
	if (el){
	    for (el =el->next;el;el=el->next){
#if defined( SORT_MINDIST_ELEMENTS )
		if (el->value < new_min->value ||
		    (el->value == new_min->value && el->cmp_index < new_min->cmp_index)){
		    new_min = el;
		}
#else
		if (el->value < new_min->value){
		    new_min = el;
		}
#endif		
	    }
	}
	min_per_array_pos[i] = new_min;
    }

    // go from bottom to top and trace path of elem
    // as long as element is found update stadel
    unsigned int height;
    i += size;
    for (height = size>>1;height > 0;height>>=1){		// stadel, go from bottom to top
	i = i>>1;
	if (elem != stadel[i]){	// finito
	    break;
	}
	int j = i*2;
	if (!stadel[j]){
	    stadel[i] = stadel[j+1];
	}else if (!stadel[j+1]){
	    stadel[i] = stadel[j];
	}else{
#if defined( SORT_MINDIST_ELEMENTS )
	if (stadel[j]->value < stadel[j+1]->value ||
	    (stadel[j]->value == stadel[j+1]->value && stadel[j]->cmp_index < stadel[j+1]->cmp_index))
#else
	if (stadel[j]->value < stadel[j+1]->value)
#endif	    

	    {
		stadel[i] = stadel[j];
	    }else{
		stadel[i] = stadel[j+1];
	    }
	}
    }
}

void IVV_Cluster_Min_Hash::add(void *elem, IVP_DOUBLE val){
    int i = hash_index((char *)&elem);
    IVV_Cluster_Min_Hash_Elem *el = new IVV_Cluster_Min_Hash_Elem();//(IVV_Cluster_Min_Hash_Elem *)p_malloc(sizeof(IVV_Cluster_Min_Hash_Elem));
#if defined(SORT_MINDIST_ELEMENTS)
    static int sort_counter = 1;
    el->cmp_index = sort_counter++;
#endif    
    el->elem = elem;
    el->next = elems[i];
    elems[i] = el;
    el->value = val;
    counter++;
    if (!min_per_array_pos[i] || val < min_per_array_pos[i]->value){
	this->min_added_at_index(el,i);
    }
}

/** update element in min_hash */
void IVV_Cluster_Min_Hash::change_value(void *elem, IVP_DOUBLE val){

    remove(elem);
    add(elem,val);
    return;
    int i = hash_index((char *)&elem);
    IVV_Cluster_Min_Hash_Elem *el;
    for (el = elems[i];el;el=el->next){
	if ( el->elem == elem){
	    break;
	}
    }
    IVP_ASSERT(el != NULL);

    if (val < el->value){	// we got smaller
	el->value = val;	// update stadel
	if (val <= min_per_array_pos[i]->value){	// new mimimum found
	    min_added_at_index(el,i);
	}
    }else{	// we became bigger
	el->value = val;	// update stadel
	if (el == min_per_array_pos[i]){	// we were a minimum, so get rid of it
	    min_removed_at_index(el,i);	  // let's look for a new minimum
	}
    }
}

/** try to remove element from min_hash */
void IVV_Cluster_Min_Hash::remove(void *elem){
    int i = hash_index((char *)&elem);
    IVV_Cluster_Min_Hash_Elem *el,*last_el;
    last_el = 0;
    for (el = elems[i];el;el=el->next){
	if ( el->elem == elem){
	    if (last_el){
		last_el->next = el->next;
	    }else{
		elems[i] = el->next;
	    }
	    el->next = 0;
	    if (el == min_per_array_pos[i]){
		min_removed_at_index(el,i);
	    }
	    P_DELETE(el);
	    counter--;
	    return;
	}
	last_el = el;
    }
}

int IVV_Cluster_Min_Hash::is_elem(void *elem){
    int i = hash_index((char *)&elem);
    IVV_Cluster_Min_Hash_Elem *el;
    for (el = elems[i];el;el=el->next){
	if ( el->elem == elem){
	    return 1;
	}
    }
    return 0;
}

void IVV_Cluster_Min_Hash::remove_min(){
    IVV_Cluster_Min_Hash_Elem *min = (IVV_Cluster_Min_Hash_Elem *)this->find_min_elem();
    if (min){
	this->remove(min);
    }
}





