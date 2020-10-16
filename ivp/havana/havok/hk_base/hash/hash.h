#ifndef HK_BASE_HASH_H
#define HK_BASE_HASH_H


//: hash class, requires KV (== KEY VALUE pair) class to implement:
//: hk_uint32 KV::get_hash_index(): should return a number between and 1 and 0xffffffff
//: hk_bool KV::equals( KV & ) should return true if two KV are equal else HK_FALSE
template<class KV>
class hk_Hash
{
	public:

		HK_TEMPLATE_INLINE hk_Hash(int size, hk_Memory *mem = hk_Memory::get_instance());
			//: assert(size = 2,4,8,16,32 ... 2**x)
		inline ~hk_Hash();

		HK_TEMPLATE_INLINE void add_element( KV &key_value );

		//inline void remove_element( KEY &key );

		HK_TEMPLATE_INLINE KV* search_element( KV &in);
			//: search element


	protected:
		inline void rehash(int new_size);

		class hk_Hash_Element {
		public:
			KV	m_kv;
			hk_uint32 m_hash_index;
		};

		hk_uint32 m_size_mm;  // size -1, used for masking index
		hk_uint32 m_nelems;
		hk_Hash_Element *m_elems;
};

#include <hk_base/hash/hash.inl>

#endif /* HK_BASE_HASH_H */

