// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef _IVP_VHASH_INCLUDED
#define _IVP_VHASH_INCLUDED


#define IVP_VHASH_TOUCH_BIT 0x80000000


/********************************************************************************
 *	File:	       	ivu_vhash.hxx	
 *	Description:	special, superfast hashclass base,
 *                      key is calculated from the elems
 *                      hash is automatically expanded but does not automatically shrink#
 *
 *      Note:           highest bit of hash_index is used as a flag
 ********************************************************************************/

class IVP_VHash_Elem {
public:
  unsigned int hash_index;       // unmasked index : highest bit is used to touch elements
  const void *elem;
};

extern unsigned int IVP_Hash_crctab[];

class IVP_VHash {
private:
    void rehash(int new_size);
protected:
    int size_mm;  // size -1, used for masking index
    unsigned int nelems:24;
    IVP_BOOL dont_free:8;        // indicating external memory
  
    IVP_VHash_Elem *elems;

    IVP_VHash(int size);// assert(size = 2**x)
    IVP_VHash(IVP_VHash_Elem *static_elems, int size);// assert(size = 2**x)
    virtual IVP_BOOL compare(void *elem0, void *elem1)const = 0;  // return TRUE if equal
public:
    static inline int hash_index(const char *data, int size);            // useable index calculation, result is [0,0xfffffff]
    static inline int fast_hash_index(int key);       // useable index calculation when size == 4 , result is [0,0xfffffff]

    // touches element
    void add_elem(const void *elem, int hash_index);

    /* try to find and remove an element equals to elem, returns the element or null
     * elements left of elem are untouched
     */
    void *remove_elem(const void *elem, unsigned int hash_index);

/* try to find an element equals to elem, *********/
    void *find_elem(const void *elem, unsigned int hash_index) const;
    
    void *touch_element(const void *elem, unsigned int hash_index);	// finds and touches

    void garbage_collection( int estimated_hash_size );

    void deactivate(); // remove elems memory (assert no elems used)
    void activate(int preferred_size);   // allocate memory
 
    int len() const { return size_mm+1;};  // size of hash array
    int n_elems(){ return nelems; };  // elems in hash
    void *element_at(int i) const { return (void *)elems[i].elem;};
    IVP_BOOL is_element_touched(int i) const { return (IVP_BOOL)(elems[i].hash_index >= IVP_VHASH_TOUCH_BIT); }
    void untouch_all();
    void print()const;
    void check();	// check internal consistency
    virtual ~IVP_VHash();
};

// overflow of hash is handled by shifting elems to higher positions in hash,
// note: the order of entries ( comparing each (hash_index&size_mm) ) is preserved


// basic function for calculating the hash_index
inline int IVP_VHash::hash_index(const char *key, int key_size){
	unsigned int c;		
	unsigned int index = 0xffffffffL;
	int i;
	for (i=key_size-1;i>=0;i--){
	    c = *((unsigned char *)(key++));
	    index = IVP_Hash_crctab[((int) index ^ c) & 0xff] ^ (index >> 8);
	}
	return index | IVP_VHASH_TOUCH_BIT;	// set touch bit
    };

// basic function for calculating the hash_index of key is a long
inline int IVP_VHash::fast_hash_index(int key){
  int index =  ((key * 1001)>>16) + key * 75;
  return index | IVP_VHASH_TOUCH_BIT;	// set touch bit
}





class IVP_VHash_Store_Elem {
public:
  unsigned int hash_index;       // unmasked index : highest bit is used to touch elements
  void *key_elem;
  void *elem;
};


// special, superfast hashclass base,
// it can store a pointer to an associated key-pointer
// hash index is calculated from the key_pointer
// hash is automatically expanded but not automatically shrunken

// highest bit of hash_index is used as a flag

class IVP_VHash_Store {
private:
    void rehash(int new_size);
protected:
    int size;
    int size_mm;  // size -1, used for masking index
    int nelems;
    IVP_VHash_Store_Elem *elems_store;
    void *dont_free;        // indicating external memory

    static inline IVP_BOOL compare_store_hash(void *pointer0, void *pointer1);  // return TRUE if equal
    static inline int hash_index_store(const char *data, int size);            // useable index calculation, result is [0,0xfffffff]
    static inline int void_pointer_to_index(void *p);
public:
    // touches element
    void add_elem(void *key_elem, void *store_elem);
    void add_elem(void *key_elem, void *store_elem,int hash_index);    


    void change_elem(void *key_elem, void *store_elem);

    /* try to find and remove an element equals to key_elem, returns the element or null
     * elements left of elem are untouched
     */
    void *remove_elem(void *key_elem);
    void *remove_elem(void *key_elem, unsigned int hash_index);

/* try to find an element equals to elem,
 *********/
    void *find_elem(void *key_elem);
    void *find_elem(void *key_elem,unsigned int hash_index);
    
    void *touch_element(void *key_elem,unsigned int hash_index);	// finds and touches
 
    int len(){ return size;};
    int n_elems(){ return nelems; };
    void *element_at(int i){ return elems_store[i].elem;};
    IVP_BOOL is_element_touched(int i){ return (IVP_BOOL)(elems_store[i].hash_index >= IVP_VHASH_TOUCH_BIT); }
    void untouch_all();
    void print();
    void check();	// check internal consistency


    IVP_VHash_Store(int size);// assert(size = 2**x)
    IVP_VHash_Store(IVP_VHash_Store_Elem *static_elems, int size);// assert(size = 2**x)
    ~IVP_VHash_Store();
};

// overflow of hash is handled by shifting elems to higher positions in hash,
// note: the order of entries ( comparing each (hash_index&size_mm) ) is preserved


// basic function for calculating the hash_index
inline int IVP_VHash_Store::hash_index_store(const char *key, int key_size){
	unsigned int c;		
	unsigned int index = 0xffffffffL;
	int i;
	for (i=key_size-1;i>=0;i--){
	    c = *((unsigned char *)(key++));
	    index = IVP_Hash_crctab[((int) index ^ c) & 0xff] ^ (index >> 8);
	}
	return index | IVP_VHASH_TOUCH_BIT;	// set touch bit
};

inline int IVP_VHash_Store::void_pointer_to_index(void *p) {
    void *help_pointer=p;
    return hash_index_store( (char *)&help_pointer, sizeof(void *) );
};

inline IVP_BOOL IVP_VHash_Store::compare_store_hash(void *pointer0, void *pointer1) {
    return (IVP_BOOL)(pointer0 == pointer1);
};


// For threadsave usage
template<class T>
class IVP_VHash_Enumerator {
    int index;

public:
    IVP_VHash_Enumerator( IVP_VHash *vec){
	index = vec->len()-1;
    }

    T *get_next_element( IVP_VHash *vec){
	while (1){
	    if (index < 0) return NULL;
	    T *res = (T*)vec->element_at(index--);
	    if (res) return res;
	}
    }
};


#endif

