// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef _IVP_BIGVECTOR_INCLUDED
#define _IVP_BIGVECTOR_INCLUDED

/********************************************************************************
 *	File:	       	ivu_physics.hxx	
 *	Description:	A vector is an array of pointers
 *                      the length of this array is dynamically increased when
 *                      elements are inserted
 ********************************************************************************/

class IVP_U_BigVector_Base
{
public:
    int memsize;
    int n_elems;
    void **elems;
    void increment_mem();
};

template<class T>
class IVP_U_BigVector: public IVP_U_BigVector_Base {
    void ensure_capacity(){
	if (n_elems>=memsize){
	    this->increment_mem();
	}
    };
protected:
    //  special vector with preallocated elems
    IVP_U_BigVector(void **ielems, int size){
	IVP_ASSERT (ielems == (void **)(this +1));
	elems = ielems;
	memsize = size;
	n_elems = 0;
    }
    
public:
    IVP_U_BigVector(int size = 0){
	memsize = size;
	n_elems = 0;
	if (size){		// will be optimized by most compilers
	    elems = (void **)p_malloc(size*sizeof(void *));
	}else{	
	    elems = NULL;
	}
    };
    
    void clear(){
	if ( elems != (void **) (this+1)){
	    void *dummy=(void *)elems;
	    P_FREE(dummy);
	    elems=0;
	    memsize = 0;
	}
	n_elems = 0;
    };

    void exchange_vector_elems( int first, int second ) {
        void *temp;
	temp=elems[first];
	elems[first]=elems[second];
	elems[second]=temp;
    };
  
    void remove_all(){
	n_elems = 0;
    };
    
    ~IVP_U_BigVector(){
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
	IVP_ASSERT( index_of(elem) == -1);
	elems[n_elems] = (void *)elem;
	return n_elems++;
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



    int install(T *elem){
	int position = index_of(elem);
	if ( position>=0) return position;
	ensure_capacity();
	elems[n_elems] = (void *)elem;
	return n_elems++;
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
class IVP_U_BigVector_Enumerator {
    int index;
public:
    inline IVP_U_BigVector_Enumerator( IVP_U_BigVector<T> *vec){
	index = 0; //vec->n_elems-1;
    }

    T *get_next_element( IVP_U_BigVector<T> *vec){
	if (index >= vec->n_elems) return NULL;
	//if (index < 0) return NULL;
	return vec->element_at(index--);
    }
};

#endif
