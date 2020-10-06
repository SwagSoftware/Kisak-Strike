
template<class KV>
void hk_Hash<KV>::rehash(int new_size)
{
  hk_Hash_Element *old_elems = m_elems;
  int old_size = m_size_mm+1;

  // get new elements
	m_size_mm = new_size-1;
    m_elems = hk_allocate( hk_Hash_Element, new_size, HK_MEMORY_CLASS_HASH);
	{
		hk_Hash_Element *e = m_elems;
		for ( int i = m_size_mm; i>=0; i--)
		{
			e->m_hash_index = 0;
			e++;
		}
	}


	hk_Hash_Element *e = old_elems;
	int old_nelems = m_nelems;
	for (int i = old_size-1;i>=0;i--){
		if (e->m_hash_index){
			this->add_element(e->m_kv);	// keeps touch bit,
		}
		e++;
	}
	m_nelems = old_nelems;
};


template<class KV>
void hk_Hash<KV>::add_element( KV &elem )
{
  // check for size
  if ( int(m_nelems + m_nelems) > m_size_mm){
    this->rehash(m_size_mm + m_size_mm + 2);
  }

  int hash_index = elem.get_hash_index();
  HK_ASSERT(hash_index != 0);

  int index = hash_index & m_size_mm;
  int pos = index;

  m_nelems++;

  // search a free place to put the elem
  for ( ; ; pos = (pos+1)&m_size_mm ){
    hk_Hash_Element *e = &m_elems[pos];
    if (!e->m_hash_index) break;
    int e_index = e->m_hash_index & m_size_mm;
    if (index >= e_index) continue;

    const KV h_e = e->m_kv;
    int h_i = e->m_hash_index;
    e->m_kv = elem;
    e->m_hash_index = hash_index;
    elem = h_e;
    hash_index = h_i;
    index = e_index;
  }

  m_elems[pos].m_kv = elem;
  m_elems[pos].m_hash_index = hash_index;
}

//inline void remove_element( KEY &key );

		//: search element
template<class KV>
KV* hk_Hash<KV>::search_element( KV &kv )
{
	hk_uint32 hash_index = kv.get_hash_index();
	int pos = hash_index & m_size_mm;
  
  // 1. search elem
	for ( ; ; pos = (pos+1) & m_size_mm ){
		hk_Hash_Element *e = &m_elems[pos];
		if (!e->m_hash_index) break;
		if ( e->m_hash_index  != hash_index) continue;
		if ( !e->m_kv.equals(kv) ){
			continue;
		}
		return &e->m_kv;
    }
	return HK_NULL;
}

template<class KV>
hk_Hash<KV>::hk_Hash(int size, hk_Memory *mem)
		//: assert(size = 2,4,8,16,32 ... 2**x)
{
 	m_size_mm = size - 1;
	m_nelems = 0;
    m_elems = hk_allocate( hk_Hash_Element, size, HK_MEMORY_CLASS_HASH);
	hk_Hash_Element *e = m_elems;
	for ( int i = m_size_mm; i>=0; i--)
	{
		e->m_hash_index = 0;
		e++;
	}
}

template<class KV>
hk_Hash<KV>::~hk_Hash()
{
	hk_deallocate( hk_Hash_Element, m_elems, m_size_mm+1, HK_MEMORY_CLASS_HASH );
}
