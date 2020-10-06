// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

class IVP_Hash_Elem;

class IVP_U_String_Hash {
public:
    int	size;
    void *not_found_value;
    IVP_Hash_Elem **elems;
    
    inline int hash_index(const char *key)const;
    
    IVP_U_String_Hash(int size, void *not_found_value = 0);
    ~IVP_U_String_Hash();
    void add(const char *key,void *value);
    void remove(const char *key);
    void *find(const char *key) const;
};

