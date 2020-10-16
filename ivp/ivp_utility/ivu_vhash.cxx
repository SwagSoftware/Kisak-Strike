// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <ivu_hash.hxx>
#include <ivu_vhash.hxx>

#ifndef WIN32
#	pragma implementation "ivu_set.hxx"
#endif

#include <ivu_set.hxx>



IVP_VHash::IVP_VHash(int size_i){
  IVP_IF(1){
    int x = size_i;
    while ( !(x&1) ) x = x>>1;
    IVP_ASSERT(x==1);// size must be 2**x
  }

  size_mm = size_i-1;
  nelems = 0;
  dont_free = IVP_FALSE;
  elems = (IVP_VHash_Elem *)p_calloc(size_i, sizeof(IVP_VHash_Elem));
}

void IVP_VHash::activate(int size_i){
    IVP_ASSERT( elems == NULL);
    IVP_IF(1){
	int x = size_i;
	while ( !(x&1) ) x = x>>1;
	IVP_ASSERT(x==1);// size must be 2**x
    }
    size_mm = size_i-1;
    nelems = 0;
    dont_free = IVP_FALSE;
    elems = (IVP_VHash_Elem *)p_calloc(size_i, sizeof(IVP_VHash_Elem));
}


void IVP_VHash::deactivate(){
    IVP_ASSERT( nelems == 0);
    if (!dont_free){
	P_FREE(elems);
    }
    size_mm = -1;
}


void IVP_VHash::garbage_collection(int preferred_size){
    if (preferred_size < n_elems()) return;
    int new_size = size_mm + 1;
    while ( preferred_size * 2 + 1 < new_size) {
	new_size /= 2;
    }
    this->rehash(new_size);
}

IVP_VHash::IVP_VHash(IVP_VHash_Elem *static_elems, int size_i){
  IVP_IF(1){
    int x = size_i;
    if (x){
	while ( !(x&1) ) x = x>>1;
	IVP_ASSERT(x==1);// size must be 2**x
    }
  }

  size_mm = size_i-1;
  nelems = 0;
  dont_free = IVP_TRUE;
  elems = static_elems;
}


IVP_VHash::~IVP_VHash(){
  if (!dont_free){
    P_FREE(elems);
  }
}

void IVP_VHash::untouch_all(){
    int i;
    for (i=size_mm;i>=0; i--){
	elems[i].hash_index &= (IVP_VHASH_TOUCH_BIT-1);
    }	
}

void IVP_VHash::rehash(int new_size){
  IVP_VHash_Elem *old_elems = elems;
  int old_size = size_mm+1;

  // get new elements
  size_mm = new_size-1;
  elems = (IVP_VHash_Elem *)p_calloc(new_size, sizeof(IVP_VHash_Elem));

  IVP_VHash_Elem *e = old_elems;
  int old_nelems = nelems;
  for (int i = old_size-1;i>=0;i--){
    if (e->elem){
	this->add_elem(e->elem, e->hash_index);	// keeps touch bit,
    }
    e++;
  }
  nelems = old_nelems;
  
  if ( !dont_free){
    P_FREE(old_elems);
  }
  dont_free = IVP_FALSE;
};

// keeps touch bit
void IVP_VHash::add_elem(const void *elem, int hash_index){
  // check for size
  if ( int(nelems + nelems) > size_mm){
    this->rehash(size_mm+size_mm+2);
  }
  IVP_IF(1){
      //check();
      IVP_ASSERT (!find_elem(elem,hash_index));
  }

  int index = hash_index & size_mm;
  int pos = index;
  nelems++;
  // search a free place to put the elem
  for ( ; ; pos = (pos+1)&size_mm ){
    IVP_VHash_Elem *e = &elems[pos];
    if (!e->elem) break;
    int e_index = e->hash_index & size_mm;
    if (index>=e_index) continue;

    const void *h_e = e->elem;
    int h_i = e->hash_index;
    e->elem = (void *)elem;
    e->hash_index = hash_index;
    elem = h_e;
    hash_index = h_i;
    index = e_index;
  }
  elems[pos].elem = (void *)elem;
  elems[pos].hash_index = hash_index;
  IVP_IF(0){
      check();
  }
}

void *IVP_VHash::remove_elem(const void *elem, unsigned int hash_index){
  int index  = hash_index & size_mm;
  int pos = index;

  // 1. search elem
  IVP_VHash_Elem *e;
  for ( ; ; pos = (pos+1)&size_mm ){
    e = &elems[pos];
    if (e->elem == 0) CORE; //return 0;
    if ( (e->hash_index  | IVP_VHASH_TOUCH_BIT) != hash_index) continue;
    if ( compare((void *)e->elem, (void *)elem)== IVP_TRUE ){
	break;
    }
    
  }
  // now elem is found, remove and return it
  elem = e->elem; // remember elem
  nelems--;
  int last_pos = pos;

  int ideal_pos_of_last_element = size_mm;		// find index position of rightest element  
  if (elems[size_mm].elem){
      ideal_pos_of_last_element = elems[size_mm].hash_index & size_mm;
  }

  // shift next elements, try to preserve order
  for (pos = (pos+1) & size_mm; ; pos = (pos+1)&size_mm){
      IVP_VHash_Elem *en = &elems[pos];
      if ( !en->elem ) break;
      int ideal_pos = en->hash_index & size_mm;	// ideal position

      if (pos > last_pos){		// not wrapped situation
	  if (ideal_pos < ideal_pos_of_last_element ){	// shift wrapped elements also
	      if (ideal_pos > last_pos) continue;  // no way to shift elements left to their ideal position
	  }else if ( ideal_pos == ideal_pos_of_last_element){	// further checks needed: are we at the start or at the end of the hash ??
	      if (ideal_pos <= pos){	// element is not wrapped -> we are at the end of the hash table
		  if (ideal_pos > last_pos) continue;  // no way to shift elements left to their ideal position
	      }else{		// at the wrapped start of the hashtable, shifts are allowed
		  ;
	      }
	  }
      }else{
	  if (last_pos != size_mm) break;			// only wrap last elements
	  if (ideal_pos < ideal_pos_of_last_element) continue;	// jump over not wrapped elements
      }
      
      elems[last_pos] = *en;         // shift elem
      last_pos = pos;
  }
  elems[last_pos].elem = 0;// remove elem
  elems[last_pos].hash_index = 0;// untouch elem
  IVP_IF(0){
      check();
      IVP_ASSERT(!find_elem(elem,hash_index));
  }
  
  return (void *)elem;
}

void IVP_VHash::check(){
    int pos;
    int last_index = 0;
  int ideal_pos_of_last_element = size_mm;		// find index position of rightest element  
  if (elems[size_mm].elem){
      ideal_pos_of_last_element = elems[size_mm].hash_index & size_mm;
  }

  for (pos = 0;pos<=size_mm;pos++){	// search till first null or exact
      IVP_VHash_Elem *en = &elems[pos];
      if (!en->elem) continue;
      int index = en->hash_index & size_mm;
      if ( index >= ideal_pos_of_last_element){
	if ( pos <= (size_mm>>1) ){
	      continue;	// skip wrapped elements
	  }
      }
      IVP_ASSERT(index <= pos);
      if (index != pos){	// shifted element
	  IVP_ASSERT(index>=last_index);
      }
      last_index = index;
    }
}

void *IVP_VHash::find_elem(const void *elem, unsigned int hash_index)const {
  int pos = hash_index & size_mm;
  
  // 1. search elem
  for ( ; ; pos = (pos+1)&size_mm ){
    IVP_VHash_Elem *e = &elems[pos];
    if (!e->elem) break;
    if ( (e->hash_index  | IVP_VHASH_TOUCH_BIT )!= hash_index) continue;
    if ( compare((void *)e->elem, (void *)elem)== IVP_FALSE ){
	continue;
    }
    // now elem is found, return it
    return (void *)e->elem;
  }

  // search by hand
  IVP_IF(0){
      for (pos = size_mm; pos>=0;pos--){
	  IVP_VHash_Elem *e = &elems[pos];
	  if (!e->elem) break;
	  if ( (e->hash_index  | IVP_VHASH_TOUCH_BIT )== hash_index) CORE;
	  if ( compare((void *)e->elem,(void *) elem)== IVP_FALSE ) continue;
	  CORE;
      }
  }
  return 0; // not found
}

void *IVP_VHash::touch_element(const void *elem, unsigned int hash_index) {
  int pos = hash_index & size_mm;
  // 1. search elem
  for ( ; ; pos = (pos+1)&size_mm ){
    IVP_VHash_Elem *e = &elems[pos];
    if (!e->elem) break;
    if ( (e->hash_index  | IVP_VHASH_TOUCH_BIT )!= hash_index) continue;
    if ( compare((void *)e->elem,(void *) elem)== IVP_FALSE ) continue;
    // now elem is found, return it
    e->hash_index |= IVP_VHASH_TOUCH_BIT;
    return (void *)e->elem;
  }
  return 0; // not found
}


void IVP_VHash::print()const{
    int i;
    printf("%i:",len());
    for (i = 0; i<= size_mm;i++){
    //lwss - x64 fixes
	//printf (" %i:%X:%X  ", elems[i].hash_index & size_mm, (int)elems[i].elem, elems[i].hash_index);
	printf (" %i:%p:%X  ", elems[i].hash_index & size_mm, (void*)elems[i].elem, elems[i].hash_index);
	//lwss end
    }
    printf("\n");
}


/*******************************************************************************************************************
 * Store Hash  -  stores an additional associated element
 *                works only with pointers as key elements and pointers as additional associated elements
 *******************************************************************************************************************/

IVP_VHash_Store::IVP_VHash_Store(int size_i){
  IVP_IF(1){
    int x = size_i;
    while ( !(x&1) ) x = x>>1;
    IVP_ASSERT(x==1);// size must be 2**x
  }

  size = size_i;
  size_mm = size_i-1;
  nelems = 0;
  dont_free = 0;
  elems_store = (IVP_VHash_Store_Elem *)p_calloc(size_i, sizeof(IVP_VHash_Store_Elem));
}



IVP_VHash_Store::IVP_VHash_Store(IVP_VHash_Store_Elem *static_elems, int size_i){
  IVP_IF(1){
    int x = size_i;
    while ( !(x&1) ) x = x>>1;
    IVP_ASSERT(x==1);// size must be 2**x
  }

  size =  size_i;
  size_mm = size_i-1;
  nelems = 0;
  dont_free = static_elems;
  elems_store = static_elems;
}


IVP_VHash_Store::~IVP_VHash_Store(){
  if (elems_store != dont_free){
    P_FREE(elems_store);
  }
}

void IVP_VHash_Store::untouch_all(){
    int i;
    for (i=size-1;i>=0; i--){
	elems_store[i].hash_index &= (IVP_VHASH_TOUCH_BIT-1);
    }	
}

void IVP_VHash_Store::rehash(int new_size){
  IVP_VHash_Store_Elem *old_elems = elems_store;
  int old_size = size;

  // get new elements
  size = new_size;
  size_mm = new_size-1;
  elems_store = (IVP_VHash_Store_Elem *)p_calloc(new_size, sizeof(IVP_VHash_Store_Elem));

  IVP_VHash_Store_Elem *e = old_elems;
  int old_nelems = nelems;
  for (int i = old_size-1;i>=0;i--){
    if (e->elem){
	this->add_elem(e->key_elem, e->elem, e->hash_index);	// keeps touch bit,
    }
    e++;
  }
  nelems = old_nelems;
  
  if ( old_elems != dont_free){
    P_FREE(old_elems);
  }
};


void IVP_VHash_Store::add_elem(void *key_elem, void *elem){
    int hash_val = void_pointer_to_index( key_elem );
    add_elem( key_elem, elem, hash_val);
}

// keeps touch bit
void IVP_VHash_Store::add_elem(void *key_elem, void *elem, int hash_index){
  // check for size
  if ( nelems+nelems >= size){
    this->rehash(size+size);
  }
  IVP_IF(1){
      //check();
      IVP_ASSERT(!find_elem(key_elem));
 }

  int index = hash_index & size_mm;
  int pos = index;
  nelems++;
  // search a free place to put the elem
  for ( ; ; pos = (pos+1)&size_mm ){
    IVP_VHash_Store_Elem *e = &elems_store[pos];
    if (!e->key_elem) break;
    int e_index = e->hash_index & size_mm;
    if (index>=e_index) continue;

    void *h_ke = e->key_elem;
    void *h_e = e->elem;
    int h_i = e->hash_index;
    e->key_elem=key_elem;
    e->elem = elem;
    e->hash_index = hash_index;
    key_elem = h_ke;
    elem = h_e;
    hash_index = h_i;
    index = e_index;
  }
  elems_store[pos].key_elem = key_elem;
  elems_store[pos].elem = elem;
  elems_store[pos].hash_index = hash_index;
  IVP_IF(0){
      check();
  }
}

void *IVP_VHash_Store::remove_elem(void *key_elem) {
    int hash_val = void_pointer_to_index( key_elem );
    return remove_elem( key_elem, hash_val );
}

void *IVP_VHash_Store::remove_elem(void *key_elem, unsigned int hash_index){
  int index  = hash_index & size_mm;
  int pos = index;

  // 1. search elem
  IVP_VHash_Store_Elem *e;
  for ( ; ; pos = (pos+1)&size_mm ){
    e = &elems_store[pos];
    IVP_ASSERT( e->elem ); //was e->key_elem
    if ( (e->hash_index  | IVP_VHASH_TOUCH_BIT) != hash_index) continue;
    if ( compare_store_hash(e->key_elem, key_elem)== IVP_TRUE ){
	IVP_ASSERT(e->key_elem == key_elem);
	break;
    }
    
  }
  // now elem is found, remove and return it
  void *elem;
  elem = e->elem; // remember elem
  nelems--;
  int last_pos = pos;

  int ideal_pos_of_last_element = size_mm;		// find index position of rightest element  
  if (elems_store[size_mm].key_elem){
      ideal_pos_of_last_element = elems_store[size_mm].hash_index & size_mm;
  }

  // shift next elements, try to preserve order
  for (pos = (pos+1) & size_mm; ; pos = (pos+1)&size_mm){
      IVP_VHash_Store_Elem *en = &elems_store[pos];
      if ( !en->key_elem ) break;
      int ideal_pos = en->hash_index & size_mm;	// ideal position

      if (pos > last_pos){		// not wrapped situation
	  if (ideal_pos < ideal_pos_of_last_element ){	// shift wrapped elements also
	      if (ideal_pos > last_pos) continue;  // no way to shift elements left to their ideal position
	  }else if ( ideal_pos == ideal_pos_of_last_element){	// further checks needed: are we at the start or at the end of the hash ??
	      if (ideal_pos <= pos){	// element is not wrapped -> we are at the end of the hash table
		  if (ideal_pos > last_pos) continue;  // no way to shift elements left to their ideal position
	      }else{		// at the wrapped start of the hashtable, shifts are allowed
		  ;
	      }
	  }
      }else{
	  if (last_pos != size_mm) break;			// only wrap last elements
	  if (ideal_pos < ideal_pos_of_last_element) continue;	// jump over not wrapped elements
      }
      
      elems_store[last_pos] = *en;         // shift elem
      last_pos = pos;
  }
  elems_store[last_pos].key_elem = 0;// remove elem
  elems_store[last_pos].hash_index = 0;// untouch elem
  IVP_IF(0){
      check();
      IVP_ASSERT(!find_elem(key_elem,hash_index));
  }
  
  return elem;
}

void IVP_VHash_Store::check(){
    int pos;
    int last_index = 0;
  int ideal_pos_of_last_element = size_mm;		// find index position of rightest element  
  if (elems_store[size_mm].key_elem){
      ideal_pos_of_last_element = elems_store[size_mm].hash_index & size_mm;
  }

  for (pos = 0;pos<size;pos++){	// search till first null or exact
      IVP_VHash_Store_Elem *en = &elems_store[pos];
      if (!en->key_elem) continue;
      int index = en->hash_index & size_mm;
      if ( index >= ideal_pos_of_last_element){
	  if ( pos < size/2){
	      continue;	// skip wrapped elements
	  }
      }
      IVP_ASSERT(index <= pos);
      if (index != pos){	// shifted element
	  IVP_ASSERT(index>=last_index);
      }
      last_index = index;
    }
}

void *IVP_VHash_Store::find_elem(void *key_elem) {
    unsigned int hash_index = void_pointer_to_index( key_elem );
    return find_elem( key_elem, hash_index );
}
    
void *IVP_VHash_Store::find_elem(void *key_elem, unsigned int hash_index) {
  int pos = hash_index & size_mm;
  int searched_so_far=0;
  
  // 1. search elem
  for ( ; ; pos = (pos+1)&size_mm ){
    if(searched_so_far>=size) break;
    searched_so_far++;
    IVP_VHash_Store_Elem *e = &elems_store[pos];
    if (!e->elem) break; //was e->key_elem
    if ( (e->hash_index  | IVP_VHASH_TOUCH_BIT )!= hash_index) continue;
    if ( compare_store_hash(e->key_elem, key_elem)== IVP_FALSE ){
	continue;
    }
    // now elem is found, return it
    return e->elem;
  }

  // search by hand
  IVP_IF(0){
      for (pos = size-1; pos>=0;pos--){
	  IVP_VHash_Store_Elem *e = &elems_store[pos];
	  if (!e->key_elem) break;
	  if ( (e->hash_index  | IVP_VHASH_TOUCH_BIT )== hash_index) CORE;
	  if ( compare_store_hash(e->key_elem, key_elem)== IVP_FALSE ) continue;
	  CORE;
      }
  }
  return 0; // not found
}

void IVP_VHash_Store::change_elem(void *key_elem, void *new_value){
    unsigned int hash_index = void_pointer_to_index( key_elem );
    int pos = hash_index & size_mm;
    int searched_so_far=0;
  
  // 1. search elem
  for ( ; ; pos = (pos+1)&size_mm ){
    if(searched_so_far>=size) break;
    searched_so_far++;
    IVP_VHash_Store_Elem *e = &elems_store[pos];
    if (!e->elem) break; //was e->key_elem
    if ( (e->hash_index  | IVP_VHASH_TOUCH_BIT )!= hash_index) continue;
    if ( compare_store_hash(e->key_elem, key_elem)== IVP_FALSE ){
	continue;
    }
    // now elem is found, return it
    e->elem = new_value;
    return;
  }
  IVP_ASSERT(0==1);
  return; // not found
}

void *IVP_VHash_Store::touch_element(void *key_elem, unsigned int hash_index) {
  int pos = hash_index & size_mm;
  // 1. search elem
  for ( ; ; pos = (pos+1)&size_mm ){
    IVP_VHash_Store_Elem *e = &elems_store[pos];
    if (!e->key_elem) break;
    if ( (e->hash_index  | IVP_VHASH_TOUCH_BIT )!= hash_index) continue;
    if ( compare_store_hash(e->key_elem, key_elem)== IVP_FALSE ) continue;
    // now elem is found, return it
    e->hash_index |= IVP_VHASH_TOUCH_BIT;
    return e->elem; //maybe it is e->key_elem ??
  }
  return 0; // not found
}


void IVP_VHash_Store::print(){
    int i;
    printf("%i:",size);
    for (i = 0; i< size;i++){
    //lwss - x64 fixes
	//printf (" %i:%X:%X:%X  ", elems_store[i].hash_index & size_mm, (int)elems_store[i].key_elem, (int)elems_store[i].elem, elems_store[i].hash_index);
	printf (" %i:%p:%p:%X  ", elems_store[i].hash_index & size_mm, elems_store[i].key_elem, elems_store[i].elem, elems_store[i].hash_index);
	//lwss end
    }
    printf("\n");
}

