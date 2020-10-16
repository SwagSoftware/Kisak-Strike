#ifndef HK_PHYSICS_VM_QUERY_BUILDER
#define HK_PHYSICS_VM_QUERY_BUILDER

#include <hk_math/densematrix.h>
#include <hk_math/dense_vector.h>
#include <hk_physics/core/vm_query_builder/mass_relative_vector3.h>
#include <hk_physics/core/vm_query.h>

// IVP_EXPORT_PUBLIC

#ifndef HK_TRANSFORM_TO_CORE_SPACE // for ipion compatibility
#	define HK_TRANSFORM_TO_CORE_SPACE(body,vector)
#endif

#define HK_MAX_IMPULSE 1e3

template <int N>
class hk_VMQ_Storage
{
	public:

		inline hk_VMQ_Storage() { }
		inline ~hk_VMQ_Storage(){ }

		inline void initialize()
		{
			m_dense_matrix.set_zero();
			clear_velocities();
		}

		inline void clear_velocities()
		{
			hk_Vector4* p = (hk_Vector4*) (&m_velocities[0]);
			for( int i= ((N-1)>>2); i>=0; p++, i--)
			{
				p->set_zero4();
			}
		}
		
		inline int length() { return N;		}

		inline hk_Dense_Matrix&	get_dense_matrix()
		{ return m_dense_matrix;	}

		inline hk_Fixed_Dense_Matrix<N>&	get_fixed_dense_matrix()
		{ return m_dense_matrix;	}

		inline hk_Cached_Force_Axis_Description* get_cfad_buffer_0()
		{ return (hk_Cached_Force_Axis_Description *)&m_buffer_0[0];		}

		inline hk_Cached_Force_Axis_Description* get_cfad_buffer_1()
		{ return (hk_Cached_Force_Axis_Description *)&m_buffer_1[0];		}

		inline hk_real *get_velocities() { return m_velocities;	}

		inline hk_Virtual_Mass_Query *get_vmq(int x)
		{ return (hk_Virtual_Mass_Query *)&m_impulse_info[x][0];	}

	protected:

		hk_real m_velocities[ HK_NEXT_MULTIPLE_OF(4,N) ];
		char m_buffer_0[ HK_MAX_SIZEOF_CACHED_FORCE_AXIS_DESCRIPTION * N];
		char m_buffer_1[ HK_MAX_SIZEOF_CACHED_FORCE_AXIS_DESCRIPTION * N];
		hk_real HK_ALIGNED_VARIABLE(m_impulse_info[2][sizeof( hk_Virtual_Mass_Query ) * N / sizeof( hk_real)],16);
		hk_Fixed_Dense_Matrix<N> m_dense_matrix;
};


// Note T is a storage class for hk_VM_Query_Builder
// see hk_VMQ_Storage_3 public elements for details
// 
enum hk_Body_Index
{
	HK_BODY_A = 0,
	HK_BODY_B = 1
};

enum hk_Axis_Index
{
	HK_X_DIRECTION = 0,
	HK_Y_DIRECTION = 1,
	HK_Z_DIRECTION = 2
};

template<class T>
class hk_VM_Query_Builder
{
	public:

		inline hk_VM_Query_Builder(){}

		void begin(int size)
		{
			m_input[0].m_buffer = m_vmq_storage.get_cfad_buffer_0();
			m_input[1].m_buffer = m_vmq_storage.get_cfad_buffer_1();
			m_input[0].m_vmq = m_vmq_storage.get_vmq(0);
			m_input[1].m_vmq = m_vmq_storage.get_vmq(1);

			m_vmq_offset = 0;
			m_dense_matrix_offset = 0;

			m_vmq_storage.initialize();
		}


		inline T& get_vmq_storage()
		{
			return m_vmq_storage;
		}


		inline void begin_entries(int size) {}
		

		inline void add_angular(
				int index_offset,
				hk_Body_Index body_index,
				hk_Rigid_Body *rb,
				const hk_Vector3 &axis_ws,
				hk_real sign )
		{
			HK_ASSERT(	int(index_offset + m_vmq_offset/sizeof(hk_Virtual_Mass_Query)) < m_vmq_storage.length() );

			hk_Virtual_Mass_Query &vmq = *offset_vmq (m_vmq_storage.get_vmq(body_index), index_offset);
			vmq.m_linear.set_zero();
			vmq.m_angular.set_mul( sign, axis_ws );

			HK_TRANSFORM_TO_CORE_SPACE( rb, vmq.m_angular );

			vmq.m_matrix_index = m_dense_matrix_offset + index_offset;
			vmq.m_entity_core_id = rb->get_entity_core_id();
		}


		inline void add_linear( int index_offset, hk_Body_Index body_index, hk_Rigid_Body *rb,
				const hk_Vector3 &position_ws, const hk_Vector3 &direction_ws, hk_real signum )
		{
			HK_ASSERT( int(index_offset + m_vmq_offset/sizeof(hk_Virtual_Mass_Query)) < m_vmq_storage.length() );

			hk_Virtual_Mass_Query &vmq = *offset_vmq (m_vmq_storage.get_vmq(body_index), index_offset);

			hk_Vector3 mass_center_relative; mass_center_relative.set_sub( position_ws, rb->get_center_of_mass() );

			vmq.m_linear.set_mul( signum, direction_ws);
			vmq.m_angular.set_cross( mass_center_relative, vmq.m_linear );
			HK_TRANSFORM_TO_CORE_SPACE( rb, vmq.m_angular );

			vmq.m_matrix_index = m_dense_matrix_offset + index_offset;
			vmq.m_entity_core_id = rb->get_entity_core_id();
		}

		inline void add_linear( int index_offset, hk_Body_Index body_index, hk_Rigid_Body *rb,
				const hk_Mass_Relative_Vector3 &pos_rel, const hk_Vector3 &direction_ws, hk_real signum )
		{
			HK_ASSERT( int(index_offset + m_vmq_offset/sizeof(hk_Virtual_Mass_Query)) < m_vmq_storage.length() );

			hk_Virtual_Mass_Query &vmq = *offset_vmq (m_vmq_storage.get_vmq(body_index), index_offset);
			const hk_Vector3 &mass_center_relative = pos_rel.m_vector;

			vmq.m_linear.set_mul( signum, direction_ws);
			vmq.m_angular.set_cross( mass_center_relative, vmq.m_linear );
			HK_TRANSFORM_TO_CORE_SPACE( rb, vmq.m_angular );

			vmq.m_matrix_index = m_dense_matrix_offset + index_offset;
			vmq.m_entity_core_id = rb->get_entity_core_id();
		}


		inline void add_linear_xyz( int index_offset, hk_Axis_Index axis,
				hk_Body_Index body_index, hk_Rigid_Body *rb,
				const hk_Mass_Relative_Vector3 &mass_center_relative, hk_real signum)
		{
			HK_ASSERT( int(index_offset + m_vmq_offset/sizeof(hk_Virtual_Mass_Query)) < m_vmq_storage.length() );

			hk_Virtual_Mass_Query &vmq = *offset_vmq (m_vmq_storage.get_vmq(body_index), index_offset);
			vmq.m_linear.set_zero();
			vmq.m_linear( axis ) = signum;

			const hk_Vector3 &mcr = mass_center_relative.m_vector;
#if 1
			if (axis == 0){			vmq.m_angular.set( 0.0f, signum * mcr.z, -mcr.y * signum  );
			}else if ( axis == 1){	vmq.m_angular.set( -mcr.z * signum, 0.0f, mcr.x *signum );
			}else{					vmq.m_angular.set( mcr.y * signum, -mcr.x * signum, 0.0f   );
			}
#else
			vmq.m_angular.set_cross( mcr, vmq.m_linear );
#endif
			HK_TRANSFORM_TO_CORE_SPACE( rb, vmq.m_angular );
			/*
			{ // ipion check hack
				IVP_U_Float_Point dir_ws( vmq.m_linear(0), vmq.m_linear(1),vmq.m_linear(2) );
				IVP_U_Float_Point dir_cs;
				rb->get_core()->get_m_world_f_core_PSI()->vimult3( &dir_ws, &dir_cs );

				IVP_U_Float_Point pos_ws( mass_center_relative(0), mass_center_relative(1), mass_center_relative(2) );
				IVP_U_Float_Point pos_cs;
				rb->get_core()->get_m_world_f_core_PSI()->vimult3( &pos_ws, &pos_cs );
	
				IVP_U_Float_Point cross_cs;
				cross_cs.inline_calc_cross_product( &pos_cs, &dir_cs );
				ivp_message("hav %1.3f %1.3f %1.3f   ipion %1.3f %1.3f %1.3f\n", vmq.m_angular.x, vmq.m_angular.y, vmq.m_angular.z, cross_cs.k[0], cross_cs.k[1], cross_cs.k[2]);
			}
			// */

			vmq.m_matrix_index = m_dense_matrix_offset + index_offset;
			vmq.m_entity_core_id = rb->get_entity_core_id();
		}


		inline void commit_entries(int size)
		{
			m_dense_matrix_offset += size;
			m_vmq_offset += size * sizeof( hk_Virtual_Mass_Query );
		}

		inline void commit(hk_Body_Index body_index, hk_Rigid_Body *rb)
		{
			m_input[0].m_n_queries = m_dense_matrix_offset;
			m_input[1].m_n_queries = m_dense_matrix_offset;
	
			hk_Rigid_Body_Core *core = rb->get_rigid_body_core();
			hk_Dense_Matrix &matrix = m_vmq_storage.get_dense_matrix();
			hk_real *velocities = m_vmq_storage.get_velocities();

			core->add_to_mass_matrix_inv( m_input[ body_index ],matrix, velocities);
		}

		inline void clear_velocities(hk_Body_Index body_index, hk_Rigid_Body *rb)
		{
			m_vmq_storage.clear_velocities();
		}

		inline void update_velocities(hk_Body_Index body_index, hk_Rigid_Body *rb)
		{
			rb->get_rigid_body_core()->add_to_velocities(	m_input[ body_index ],
															m_vmq_storage.get_velocities());
		}

		inline void apply_impulses( hk_Body_Index body_index, hk_Rigid_Body *rb,
									const hk_real impulses[] )
		{
#ifdef HK_CHECK // lwss: add ifdef so hk_checks aren't ran on release
			HK_IF_CHECK (HK_FALSE)
			{
				for (int i = 0; i < m_dense_matrix_offset;i++)
				{
					HK_CHECK( impulses[i] < HK_MAX_IMPULSE );
				}
			}
#endif
			rb->get_rigid_body_core()->apply_impulses(	m_input[ body_index ],	impulses);
		}
	protected:

		T	m_vmq_storage;
		hk_Core_VMQ_Input m_input[2];
		unsigned int m_vmq_offset;
		int m_dense_matrix_offset;

		inline hk_Virtual_Mass_Query *offset_vmq( hk_Virtual_Mass_Query *base, int index )
		{
			return (hk_Virtual_Mass_Query *) ( ((char *)&base[index]) + m_vmq_offset );
		}
};

/* Typical usage of this class :
 * 
 *	VMQB.begin();
 *	for ( xxx ){
 *		VMQB.begin_entry()
 *		VMQB.add_linear ( first_body.... )  or something else
 *		[VMQB.add_linear ( second_body ...  )  ] only if second body exists
 *		VMQB.commit_entry()
 *	}
 *  VMQB.commit(0, first_body, 1.0f)
 *  [VMQB.commit(1, second_body, -1.0f)] only if second body exists
 * 
  */

#endif /* HK_PHYSICS_VM_QUERY_BUILDER */

