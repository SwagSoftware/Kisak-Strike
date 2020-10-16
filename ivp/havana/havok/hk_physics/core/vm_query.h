#ifndef HK_PHYSICS_VM_QUERY_H
#define HK_PHYSICS_VM_QUERY_H


class hk_Cached_Force_Axis_Description
{
	hk_Vector4 dummy;
};

#define HK_MAX_SIZEOF_CACHED_FORCE_AXIS_DESCRIPTION (sizeof(hk_Cached_Force_Axis_Description))
	//: maximum size which is allowed to use for the force axis description


class hk_Impulse_Info
{
	public:
		hk_Vector3		HK_ALIGNED_VARIABLE(m_linear,16);
		//: is usually the direction (and optionally the magnitude) of the impulse

		hk_Vector3			m_angular; 
		/* is usually the 
		(direction	of impulse) cross (point_of_contact - pivot_point(==center_of_mass))
		*/

		hk_Entity_Core_ID	m_entity_core_id; // maybe body_core_index
		int					m_matrix_index;  // only used to  build big mass matrices		
		// maybe used for soft objects
		int client_data0;
		int client_data1;
		inline hk_Impulse_Info() {}
};

class hk_Virtual_Mass_Query: public hk_Impulse_Info
{
	// m_linear.length() must be 1.0f
	// m_angular.length() must be the distance between the center_of_mass
	// and the point of contact
	public:

		inline hk_Virtual_Mass_Query(){}
};

struct hk_Core_VMQ_Input
{
	int m_n_queries;
		//:				number of queries
	hk_Virtual_Mass_Query* m_vmq;
		//: pointing to an array of query structures

	hk_Cached_Force_Axis_Description* m_buffer;
		//:	pointing to a buffer which can be used to cache values
		//: the sice of the buffer must be at least HK_MAX_SIZEOF_CACHED_FORCE_AXIS_DESCRIPTION * m_n_queries
};


#endif /* HK_PHYSICS_VM_QUERY_H */

