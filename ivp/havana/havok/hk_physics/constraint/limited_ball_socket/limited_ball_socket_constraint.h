#ifndef HK_PHYSICS_LIMITED_BALL_SOCKET_CONTRAINT_H
#define HK_PHYSICS_LIMITED_BALL_SOCKET_CONTRAINT_H



class hk_Limited_Ball_Socket_BP;
class hk_Local_Constraint_System;

// IVP_EXPORT_PUBLIC

class hk_Limited_Ball_Socket_Constraint : public hk_Constraint
{
	public:

		hk_Limited_Ball_Socket_Constraint( hk_Environment *,             const hk_Limited_Ball_Socket_BP *, hk_Rigid_Body *a ,hk_Rigid_Body *b );	
		hk_Limited_Ball_Socket_Constraint( hk_Local_Constraint_System *, const hk_Limited_Ball_Socket_BP *, hk_Rigid_Body *a ,hk_Rigid_Body *b );	


		void apply_effector_PSI( hk_PSI_Info&, hk_Array<hk_Entity*>* );

		virtual int get_vmq_storage_size();

		inline virtual int	setup_and_step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );
			//: uses the mem as a vmq storage, returns the bytes needed to store its vmq_storage

		virtual void step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );
			//: use the mem as a vmq storage setup before

		void set_angular_limits( int axis, hk_real min, hk_real max );

		void init_constraint( const void* bp );

	protected:

		void init_ball_socket_constraint( const hk_Limited_Ball_Socket_BP *bp );

		hk_Transform m_transform_os_ks[2];
		//hk_Vector3 m_position_os_ks[2];
		//hk_Matrix3* m_joint_axes[2];
		hk_real m_strength;
		hk_real m_tau;
		//hk_real m_joint_limits[3][2];
		hk_Interval<hk_real> m_angular_limits[3];
		bool m_constrainTranslation;
};

#endif /* HK_PHYSICS_LIMITED_BALL_SOCKET_CONTRAINT_H */
