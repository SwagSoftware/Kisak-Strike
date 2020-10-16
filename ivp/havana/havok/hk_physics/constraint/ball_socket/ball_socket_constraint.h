#ifndef HK_LOCAL_BALL_SOCKET_CONTRAINT_H
#define HK_LOCAL_BALL_SOCKET_CONTRAINT_H

class hk_Ball_Socket_BP;
class hk_Local_Constraint_System;

// IVP_EXPORT_PUBLIC

class hk_Ball_Socket_Constraint : public hk_Constraint
{
	public:

		hk_Ball_Socket_Constraint( hk_Environment *,             const hk_Ball_Socket_BP *, hk_Rigid_Body *a ,hk_Rigid_Body *b );	
		hk_Ball_Socket_Constraint( hk_Local_Constraint_System *, const hk_Ball_Socket_BP *, hk_Rigid_Body *a ,hk_Rigid_Body *b );	

		void apply_effector_PSI( hk_PSI_Info&, hk_Array<hk_Entity*>* );

		virtual int get_vmq_storage_size();

		inline virtual int	setup_and_step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );
			//: uses the mem as a vmq storage, returns the bytes needed to store its vmq_storage

		virtual void step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );
			//: use the mem as a vmq storage setup before

		virtual void init_constraint(const void /*blueprint*/ *);

		void write_to_blueprint( hk_Ball_Socket_BP * );

		//lwss add
        const hk_Vector3 get_transform(int index) { return m_translation_os_ks[index]; }
        //lwss end
	protected:

		void init_ball_socket_constraint(const hk_Ball_Socket_BP *);

		hk_Vector3 m_translation_os_ks[2];  // 32 

		hk_real m_tau;  // 4
		hk_real m_strength;       // 4
};

#endif //HK_LOCAL_BALL_SOCKET_CONTRAINT_H
