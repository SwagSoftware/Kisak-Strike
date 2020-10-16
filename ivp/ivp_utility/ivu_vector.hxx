// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef IVP_VECTOR_INCLUDED
#define IVP_VECTOR_INCLUDED

/********************************************************************************
 *	Class:	       	IVP_U_Vector_Base	
 *	Description:	A vector is an array of pointers
 *                      the length of this array is dynamically increased when
 *                      elements are inserted
 ********************************************************************************/

class IVP_U_Vector_Base
{
public:
    unsigned short memsize;
    unsigned short n_elems;
    void **elems;
    void increment_mem();
};

template<class T>
class IVP_U_Vector: public IVP_U_Vector_Base {
public:
    void ensure_capacity(){
	if (n_elems>=memsize){
	    this->increment_mem();
	}
    };
protected:
    //  special vector with preallocated elems
    IVP_U_Vector(void **ielems, int size){
	IVP_ASSERT (ielems == (void **)(this +1));
	elems = ielems;
	memsize = size;
	n_elems = 0;
    }
    
public:
    IVP_U_Vector(int size = 0){
	memsize = size;
	n_elems = 0;
	if (size){		// will be optimized by most compilers
	    elems = (void **)p_malloc(size*sizeof(void *));
	}else{	
	    elems = (void **)NULL;
	}
    };
    
    void clear(){
	if ( elems != (void **) (this+1)){
	    void *dummy=(void *)elems;
	    P_FREE( dummy);
	    elems=0;
	    memsize = 0;
	}
	n_elems = 0;
    };

    
    void remove_all(){
	n_elems = 0;
    };
    
    ~IVP_U_Vector(){
	this->clear();
    };
    
    int len() const {
	return n_elems;
    };
    
    int index_of(T *elem){
	int i;
	for (i=n_elems-1;i>=0;i--){
	    if (elems[i] == elem) break;
	}
	return i;
    };
    
    int add(T *elem){
	ensure_capacity();
	//IVP_ASSERT( index_of(elem) == -1);
	elems[n_elems] = (void *)elem;
	return n_elems++;
    };

    int install(T *elem){
	int old_index = index_of(elem);
	if ( old_index != -1) return old_index;
	ensure_capacity();
	elems[n_elems] = (void *)elem;
	return n_elems++;
    };

    void swap_elems(int index1, int index2){
	IVP_ASSERT((index1>=0)&&(index1<n_elems));
	IVP_ASSERT((index2>=0)&&(index2<n_elems));
	void *buffer = elems[index1];
	elems[index1] = elems[index2];
	elems[index2] = buffer;
	return;
    }     
    
    void insert_after(int index, T *elem){
	IVP_ASSERT((index>=0)&&(index<n_elems));
	index++;
	ensure_capacity();
	int j = n_elems;
	while(j>index){
	    elems[j] = elems[--j];
	}
	elems[index] = (void *)elem;
	n_elems++;
    };

    
    void remove_at(int index){
	IVP_ASSERT((index>=0)&&(index<n_elems));
	int j = index;
	while(j<n_elems-1){
	    elems[j] = elems[j+1];
	    j++;
	}
	n_elems--;
    };

    void reverse() {
	for (int i=0; i<(this->n_elems/2); i++) {
	    this->swap_elems(i, this->n_elems-1-i);
	}
	return;
    }

    void remove_at_and_allow_resort(int index){
	IVP_ASSERT((index>=0)&&(index<n_elems));
	n_elems--;
	elems[ index ] = elems[ n_elems ];
    };

    void remove_allow_resort(T *elem){
	int index = this->index_of(elem);
	IVP_ASSERT(index>=0);
	n_elems--;
	elems[ index ] = elems[ n_elems ];
    };
    
    void remove(T *elem){
	int index = this->index_of(elem);
	IVP_ASSERT(index>=0);
	n_elems--;
	while (index < n_elems){
	    elems[index] = (elems+1)[index];
	    index++;
	}
    };

    T* element_at(int index) const {
	IVP_ASSERT(index>=0 && index < n_elems);
	return (T *)elems[index];
    };
};


// For threadsave usage
template<class T>
class IVP_U_Vector_Enumerator {
    int index;
public:
    inline IVP_U_Vector_Enumerator( IVP_U_Vector<T> *vec){
	index = 0; //vec->n_elems-1;
    }

    T *get_next_element( IVP_U_Vector<T> *vec){
	if (index >= vec->n_elems) return NULL;
	//if (index < 0) return NULL;
	return vec->element_at(index--);
    }
};


//vector of cores with 2 default elements already alloced
class IVP_Vector_of_Cores_2: public IVP_U_Vector<class IVP_Core> {
    IVP_Core *elem_buffer[2];
public:
    IVP_Vector_of_Cores_2(): IVP_U_Vector<IVP_Core>( (void **)&elem_buffer[0],2 ){;};
};

#endif

