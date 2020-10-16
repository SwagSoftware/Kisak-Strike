#ifndef IVP_HASH_INCLUDED
#define IVP_HASH_INCLUDED

class IVP_Hash_Elem;

extern unsigned int IVP_Hash_crctab[];

class IVP_Hash {
    friend class IVP_U_String_Hash;
    int key_size;
    int	size;
    void *not_found_value;
    IVP_Hash_Elem **elems;
    
public:
    inline int hash_index(const char *key)const;
    
    IVP_Hash(int size, int key_size,void *not_found_value = 0);
    ~IVP_Hash();
    void add(const char *key,void *value);
    void remove(const char *key);
    void *find(const char *key)const;
};

/** class for looping through the Hash,
  if you want to delete current element, call next_element first,
  Note: not implemented */
class IVP_Hash_Enumerator {
    int index_pos;
    IVP_Hash *my_hash;
    IVP_Hash_Elem *el;
public:
    IVP_Hash_Enumerator(IVP_Hash *hash){ my_hash = hash; index_pos = -1; el = 0;};
    void *next_element();
};


/** typical usage:
 *	IVP_Hash_Enumerator en(hash);
 *	while  ( x = en.next_element()){
 *	}
*/

#endif
