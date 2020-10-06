#ifndef HK_STIFF_SPRING_CONSTRAINT_H
#define HK_STIFF_SPRING_CONSTRAINT_H

class hk_Stiff_Spring_BP;

class hk_Stiff_Spring_Constraint : public hk_Constraint
{
	public:

		void apply_effector_PSI( hk_PSI_Info&, hk_Array<hk_Entity*>* );

		hk_Stiff_Spring_Constraint( hk_Environment *,             const hk_Stiff_Spring_BP *, hk_Rigid_Body *a ,hk_Rigid_Body *b );	
		hk_Stiff_Spring_Constraint( hk_Local_Constraint_System *, const hk_Stiff_Spring_BP *, hk_Rigid_Body *a ,hk_Rigid_Body *b );	

		inline ~hk_Stiff_Spring_Constraint(){;};

		void set_length( hk_real );

		virtual void init_constraint( const void* );
		void write_to_blueprint( hk_Stiff_Spring_BP * );

	protected:

		void init_stiff_spring_constraint( const hk_Stiff_Spring_BP * );

		virtual void step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );

		virtual int get_vmq_storage_size();

		hk_Vector3 m_translation_os_ks[2];		// 32 

		hk_real m_tau;       // 4
		hk_real m_strength;  // 4
		hk_real m_stiff_spring_length;       // 4
		hk_bool m_is_rigid;

	private:

		inline virtual int	setup_and_step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );
};

#endif //HK_STIFF_SPRING_CONSTRAINT_H
