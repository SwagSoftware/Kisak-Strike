// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PROTECTED

#ifndef IVP_DIFF_HASH_INCLUDED
#define IVP_DIFF_HASH_INCLUDED


/****************************************************************
* Description:	    resorts input base vector according to the
*		    following rules
*		    if an input entry is found it is moved to 
*		    the start of the vector
* Input		    CMP is a class which defines an inline calc_hash_index
*		    and in inline CMP::are_equal( BASE, SEARCH)
*		    FVECTOR is a IVU_FVector<BASE>
*		    BASE is vector element class
*		    SEARCH is the class used as a key and is passed to CMP::xxx()
* Note              Hash Key is based on 2 SEARCH elements:
*			search:   the element to search for
*			reference: the reference element
****************************************************************/

#undef BASE //lwss- weird macro taking over here.

template<class CMP,class FVECTOR,class BASE,class SEARCH>
class IVP_Diff_Hash {
public:
    short *hash_to_vector_index;
    int buffersize_minus_one;
    IVP_BOOL private_buffer;
    FVECTOR *base_vector;
    int	    n_found_objects;

    IVP_Diff_Hash( FVECTOR *base, short *buffer, int buffer_size, SEARCH *reference ){
	if (buffer_size <= base->len()){
	    private_buffer = IVP_TRUE;
	    int x = buffer_size;
	    while ( x <= (base->len()<<1) ) x <<=1;
	    buffersize_minus_one = x-1;
	    hash_to_vector_index = (short *)p_malloc( x * sizeof(short) );
	}else{
	    private_buffer = IVP_FALSE;
	    buffersize_minus_one = buffer_size - 1;
	    hash_to_vector_index = buffer;
	}
	n_found_objects = 0;
	base_vector = base;
	for (int i = buffersize_minus_one>>1; i>=0; i--){
	    ((int *)hash_to_vector_index)[i] = 0xffffffff;
	}

	{
	    // insert all existing elements
	    for (int i = base_vector->len()-1; i>=0;i--){
		BASE *b = base_vector->element_at(i);
		for (int index = CMP::calc_hash_index(b,reference);;	index++){
		    index &= buffersize_minus_one;
		    if (hash_to_vector_index[index] == -1){ // free place found
			hash_to_vector_index[index]  = i;
			break;
		    }
		}
	    }
	}
    }

    // returns true if element is already in base vector and resorts basevector
    // accordingly if found
    // reference is the reference element
    BASE *check_element( SEARCH *elem, SEARCH *reference ){
	for (int index = CMP::calc_hash_index(elem);;index++){
	    index &= buffersize_minus_one;
	    int vector_index = hash_to_vector_index[index];
	    if (vector_index == -1){
		return NULL;
	    }
	    BASE *base_elem = base_vector->element_at(vector_index);
	    if ( !CMP::are_equal( base_elem, elem )){
		continue;
	    }
	    // now we found the element 
	    IVP_ASSERT( vector_index >= n_found_objects);
	    if (vector_index > n_found_objects){
		// swap elems in base vector:  n_found_objects : vector_index
		BASE *nfb = base_vector->element_at(n_found_objects);
		// search and mark existing position
		for ( int i = CMP::calc_hash_index(nfb,reference);; i++){
		    i &= buffersize_minus_one;
		    if (hash_to_vector_index[i] != n_found_objects){
			continue;
		    }
		    hash_to_vector_index[i] = vector_index;
		    hash_to_vector_index[index] = n_found_objects;
		    base_vector->swap_elems( n_found_objects, vector_index );
		    break;
		}
	    }
	    n_found_objects++;
	    return base_elem;
	}
    }


    ~IVP_Diff_Hash(){
	if (private_buffer){
	    P_FREE(hash_to_vector_index);
	}
    }
};


#endif
