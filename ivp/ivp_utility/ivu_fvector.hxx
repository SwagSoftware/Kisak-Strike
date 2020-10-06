
//IVP_EXPORT_PUBLIC

#ifndef IVP_FVECTOR_INCLUDED
#define IVP_FVECTOR_INCLUDED



/********************************************************************************
 *	Class:	       	IVP_U_FVector
 *	Description:	A special vector which allows a very fast deletion
 *			operation,
 *			The vector elements can only be inserted into N (currently set to the fixed value of 2
 *			FVectors and needs to implement int get_fvector_index(int pos) (where pos = 0,1) and
 *			void set_fvector_index(int old_index, int new_index)
 ********************************************************************************/

template<class T>
class IVP_U_FVector: public IVP_U_Vector_Base {
    void ensure_capacity(){
	if (n_elems>=memsize){
	    this->increment_mem();
	}
    };
protected:
    //  special vector with preallocated elems
    IVP_U_FVector(void **ielems, int size){
	IVP_ASSERT (ielems == (void **)(this +1));
	elems = ielems;
	memsize = size;
	n_elems = 0;
    }
    
public:
    IVP_U_FVector(int size = 0){
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
	    P_FREE(elems);
	    memsize = 0;
	}
	n_elems = 0;
    };
    
    void remove_all(){
	n_elems = 0;
    };
    
    ~IVP_U_FVector(){
	this->clear();
    };
    
    int len() const {
	return n_elems;
    };
    
    int index_of(T *elem){
	int i0 = elem->get_fvector_index(0);
	if ( i0 >=0 && i0 < n_elems && element_at(i0) == elem) return i0;
	IVP_ASSERT ( elem->get_fvector_index(1) <0 ||  elem->get_fvector_index(1) >= n_elems || element_at(elem->get_fvector_index(1)) == elem);
	return elem->get_fvector_index(1);
    };
    
    int add(T *elem){
	ensure_capacity();
	IVP_ASSERT( index_of(elem) == -1);
	elems[n_elems] = (void *)elem;
	elem->set_fvector_index(-1, n_elems);
	return n_elems++;
    };

    void swap_elems(int index1, int index2){
	IVP_ASSERT((index1>=0)&&(index1<n_elems));
	IVP_ASSERT((index2>=0)&&(index2<n_elems));
	T *a = (T*)elems[index1];
	T *b = (T*)elems[index2];
	elems[index1] = b;
	elems[index2] = a;
	a->set_fvector_index(index1,index2);	
	b->set_fvector_index(index2,index1);
	return;
    }    

    void remove_allow_resort(T *elem){
	int index = this->index_of(elem);
	IVP_ASSERT(index>=0);
	n_elems--;
	if ( n_elems > index){
	    T *e = (T*) elems[n_elems];
	    elems[ index ] = e;
	    e->set_fvector_index(n_elems, index);
	}
	elem->set_fvector_index(index, -1);
    };
    

    T* element_at(int index) const {
	IVP_ASSERT(index>=0 && index < n_elems);
	return (T *)elems[index];
    };
};

#endif
