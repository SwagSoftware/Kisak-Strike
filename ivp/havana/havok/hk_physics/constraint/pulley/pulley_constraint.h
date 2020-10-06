#ifndef HK_PULLEY_CONSTRAINT_H
#define HK_PULLEY_CONSTRAINT_H

class hk_Pulley_BP;

class hk_Pulley_Constraint : public hk_Constraint
{
	public:

		void apply_effector_PSI( hk_PSI_Info&, hk_Array<hk_Entity*>* );

		hk_Pulley_Constraint( hk_Environment*, const hk_Pulley_BP*, hk_Rigid_Body* a ,hk_Rigid_Body* b );	
		hk_Pulley_Constraint( hk_Local_Constraint_System*, const hk_Pulley_BP*, hk_Rigid_Body* a ,hk_Rigid_Body* b );	

		inline ~hk_Pulley_Constraint() {;};

		void set_length( hk_real length );
		
		void set_worldspace_point( int index, const hk_Vector3& ws_point );
		
		void set_gearing( hk_real gearing );

		virtual void init_constraint( const void* );
		void write_to_blueprint( hk_Pulley_BP * );

	protected:

		void init_pulley_constraint( const hk_Pulley_BP * );

		virtual void step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );

		virtual int get_vmq_storage_size();

		hk_Vector3 m_translation_os_ks[2];	// 32 
		hk_Vector3 m_worldspace_point[2];	// 32

		hk_real m_tau;       			// 4
		hk_real m_strength;  			// 4
		hk_real m_length;       		// 4
		hk_real m_gearing;			// 4
		hk_bool m_is_rigid;
		
	private:

		inline virtual int	setup_and_step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );
};

#endif //HK_PULLEY_CONSTRAINT_H
