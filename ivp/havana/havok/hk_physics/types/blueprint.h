#ifndef HK_PHYSICS_BLUEPRINT_H
#define HK_PHYSICS_BLUEPRINT_H

#ifndef HK_PHYSICS_BLUEPRINT_TYPES_H
#	include <hk_physics/types/blueprint_types.h>
#endif 


//: A Generic Blueprint for an object
// A blueprint is simply a chunk of memory that contains enough data to instantiate an object.
// An appropriate factory will take a blueprint and construct an appropriate object
// By their nature Blueprints are disposable - a persistent blueprint is also provided.
class hk_Blueprint
{
	public:
		inline hk_id get_id()   const;  
			//: returns the blueprint id

		inline hk_type get_type() const;  
			//: returns the blueprint type

		inline unsigned int get_size() const;  
			//: returns the blueprint data chunk size

		inline hk_size_t size_of() const;  
			//: returns the size of the blueprint including the header

	protected:
		inline hk_Blueprint( hk_type type, int size );
			//: Blueprint constructor 
			// The id is filled in automatically from the
			// static 'next_id' counter

		hk_Blueprint* clone();
			//: Clones this blueprint 
			// Give the clone a new id

		hk_id			m_id;
			//: The unique ID for this blueprint
		const hk_type	m_type;
			//: The type of object that can be built from the blueprint
		int				m_length;
			//: The length of the blueprint in bytes
};

class hk_Persistent_Blueprint : public hk_Blueprint
{
protected:
	hk_Persistent_Blueprint( hk_type type, int size ) : hk_Blueprint( type, size ){};
};

class hk_Disposable_Blueprint : public hk_Blueprint
{
protected:
	hk_Disposable_Blueprint( hk_type type, int size ) : hk_Blueprint( type, size ){};
};

#include <hk_physics/types/blueprint.inl>

#endif  // HK_PHYSICS_BLUEPRINT_H
