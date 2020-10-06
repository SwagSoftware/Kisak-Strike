#ifndef HK_LOCAL_BREAKABLE_CONTRAINT_H
#define HK_LOCAL_BREAKABLE_CONTRAINT_H

class hk_Breakable_BP;
class hk_Local_Constraint_System;

// IVP_EXPORT_PUBLIC

class hk_Breakable_Constraint : public hk_Constraint
{
	public:

		hk_Breakable_Constraint( hk_Environment*,             const hk_Breakable_Constraint_BP* );	
		hk_Breakable_Constraint( hk_Local_Constraint_System*, const hk_Breakable_Constraint_BP* );	
		~hk_Breakable_Constraint();

		void apply_effector_PSI( hk_PSI_Info&, hk_Array<hk_Entity*>* );

		virtual int get_vmq_storage_size();

		inline virtual int setup_and_step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );
			//: uses the mem as a vmq storage, returns the bytes needed to store its vmq_storage

		virtual void step_constraint( hk_PSI_Info& pi, void* mem, hk_real tau_factor, hk_real damp_factor );
			//: use the mem as a vmq storage setup before

		virtual void init_constraint(const void /*blueprint*/ *);

		void write_to_blueprint( hk_Breakable_Constraint_BP * );
		void FireEventIfBroken();

	protected:

		void init_breakable_constraint(const hk_Breakable_Constraint_BP *);

		hk_Constraint* m_real_constraint;

		hk_real m_linear_strength;
		hk_real m_angular_strength;
		bool	m_is_broken;
};

#endif //HK_LOCAL_BREAKABLE_CONTRAINT_H
