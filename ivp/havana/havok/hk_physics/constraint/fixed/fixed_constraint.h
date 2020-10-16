// TKBMS v1.0 -----------------------------------------------------
//
// PLATFORM		: ALL
// PRODUCT		: VPHYSICS
// VISIBILITY	: INTERNAL
//
// ------------------------------------------------------TKBMS v1.0

#ifndef HK_FIXED_CONSTRAINT_H
#define HK_FIXED_CONSTRAINT_H

class hk_Fixed_BP;
class hk_Local_Constraint_System;

// IVP_EXPORT_PUBLIC

class hk_Fixed_Constraint : public hk_Constraint
{
	public:

		hk_Fixed_Constraint( hk_Environment *,             const hk_Fixed_BP *, hk_Rigid_Body *a ,hk_Rigid_Body *b );	
		hk_Fixed_Constraint( hk_Local_Constraint_System *, const hk_Fixed_BP *, hk_Rigid_Body *a ,hk_Rigid_Body *b );	

		void apply_effector_PSI( hk_PSI_Info&, hk_Array<hk_Entity*>* );

		virtual int get_vmq_storage_size();

		inline virtual int setup_and_step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );
			//: uses the mem as a vmq storage, returns the bytes needed to store its vmq_storage

		virtual void step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );
			//: use the mem as a vmq storage setup before

		void init_constraint( const void* bp );

		void write_to_blueprint( hk_Fixed_BP * );

        //lwss add
        //lwss hack - is this correct??
        const hk_Transform get_transform(int index) { return m_transform_os_ks; }
        //lwss end

	protected:

		void init_fixed_constraint( const hk_Fixed_BP *bp );

		hk_Transform m_transform_os_ks;
		hk_real m_strength;
		hk_real m_tau;
};

#endif /* HK_FIXED_CONSTRAINT_H */
