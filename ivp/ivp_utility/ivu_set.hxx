// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef IVP_SET_INCLUDED
#define IVP_SET_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#ifndef _IVP_VHASH_INCLUDED
#    include "ivu_vhash.hxx"
#endif

/********************************************************************************
 *	Class:	       	IVP_U_Set.hxx	
 *	Description:	Set of pointers
 ********************************************************************************/
// For threadsave usage
template<class T>
class IVP_U_Set_Enumerator {
    int index;

public:
    inline IVP_U_Set_Enumerator( IVP_VHash *vec) {
	this->index = vec->len()-1;
    }

    inline T *get_next_element( IVP_VHash *vec) {
	while (1) {
	    if ( this->index < 0 ) return NULL;
	    T *res = (T*)vec->element_at(this->index--);
	    if (res) return(res);
	}
    }
};


template<class T>
class IVP_U_Set: public IVP_VHash {
  //friend class IVP_U_Set_Enumerator;
protected:
    IVP_BOOL compare(void *elem0, void *elem1) const { return (elem0 == elem1)?IVP_TRUE:IVP_FALSE; }
    int      elem_to_index(T *elem){ return fast_hash_index((long) elem); }
public:
  void add_element(T *elem){
      IVP_VHash::add_elem(elem, elem_to_index(elem));
  };

  void remove_element(T *elem)    {
      IVP_VHash::remove_elem(elem, elem_to_index(elem));
    };

  // returns NULL or elem
  T *find_element(T *elem)  {
    return (T *)IVP_VHash::find_elem(elem, elem_to_index(elem));
  };

  void install_element(T *elem){
    int index = elem_to_index(elem);
    T *hit = (T *)IVP_VHash::find_elem(elem, index );
    if (hit) return;

    IVP_VHash::add_elem(elem, index);
  };

  int n_elems() { return nelems; }
  
  ~IVP_U_Set(){;};
  IVP_U_Set(int init_size) : IVP_VHash(init_size) { ;};
};

template <class T> class IVP_U_Set_Active;

template<class T>
class IVP_Listener_Set_Active {
public:
  virtual void element_added(IVP_U_Set_Active<T> *set, T *elem) = 0;
  virtual void element_removed(IVP_U_Set_Active<T> *set, T *elem) = 0;
  virtual void pset_is_going_to_be_deleted(IVP_U_Set_Active<T> *set) = 0;   // Note: pset is not removing elements when deleted
};

template <class T>
class IVP_U_Set_Active : public IVP_U_Set<T> {
protected:
  IVP_U_Vector<IVP_Listener_Set_Active<T> > listeners;
public:
  void add_element(T *elem){
      IVP_U_Set<T>::add_element(elem);
      for (int i = listeners.len()-1; i>=0;i--){
	IVP_Listener_Set_Active<T> *l = listeners.element_at(i);
	l->element_added(this,elem);
      }
  };

  void install_element(T *elem){
    int index = elem_to_index(elem);
    T *hit = (T *)IVP_VHash::find_elem(elem, index );
    if (hit) return;

    IVP_U_Set<T>::add_element(elem);
      for (int i = listeners.len()-1; i>=0;i--){
	IVP_Listener_Set_Active<T> *l = listeners.element_at(i);
	l->element_added(this,elem);
      }
  };

  
  void remove_element(T *elem)    {
      IVP_U_Set<T>::remove_element(elem);
      for (int i = listeners.len()-1; i>=0;i--){
	IVP_Listener_Set_Active<T> *l = listeners.element_at(i);
	l->element_removed(this,elem);
      }
    };

  ~IVP_U_Set_Active(){
    for (int i = listeners.len()-1; i>=0;i--){
      IVP_Listener_Set_Active<T> *l = listeners.element_at(i);
      l->pset_is_going_to_be_deleted(this);
    }
  };
  IVP_U_Set_Active(int init_size) : IVP_U_Set<T>(init_size) {;};

    void add_listener_set_active( IVP_Listener_Set_Active<T> *lis ){
	listeners.add(lis);
    }

    void remove_listener_set_active( IVP_Listener_Set_Active<T> *lis ){
	listeners.remove(lis);
    }

};

#endif
