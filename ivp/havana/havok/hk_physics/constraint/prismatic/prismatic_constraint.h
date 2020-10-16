#ifndef HK_PHYSICS_PRISMATIC_CONTRAINT_H
#define HK_PHYSICS_PRISMATIC_CONTRAINT_H

class hk_Prismatic_BP;
class hk_Local_Constraint_System;

class hk_Prismatic_Constraint : public hk_Constraint
{
	public:

		hk_Prismatic_Constraint( hk_Environment *,             const hk_Prismatic_BP *, hk_Rigid_Body *Ref ,hk_Rigid_Body *Att );	
		hk_Prismatic_Constraint( hk_Local_Constraint_System *, const hk_Prismatic_BP *, hk_Rigid_Body *Ref ,hk_Rigid_Body *Att );	


		void apply_effector_PSI( hk_PSI_Info&, hk_Array<hk_Entity*>* );

		virtual int get_vmq_storage_size();

		inline virtual int	setup_and_step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );
			//: uses the mem as a vmq storage, returns the bytes needed to store its vmq_storage

		virtual void step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor );
			//: use the mem as a vmq storage setup before

		void set_limits( hk_real min, hk_real max );
		void set_friction( hk_real friction );
		void set_motor( hk_real desired_vel, hk_real max_force );

		void write_to_blueprint( hk_Prismatic_BP * );
		void init_constraint(const void*);

	protected:

		void init_prismatic_constraint(const hk_Prismatic_BP*, hk_Local_Constraint_System *sys);
		
		hk_real m_strength;
		hk_real m_tau;
		hk_real	m_pos_interpolation_value; // if 0.0f than t0.get_position() is the reference pos, 1.0f 

		hk_Constraint_Limit m_limits[1];

		hk_Vector3			m_axis_Ros;
		hk_Vector3			m_perp_axis_Ros;

		hk_QTransform m_transform_Ros_Aos;
};

#endif /* HK_PHYSICS_PRISMATIC_CONTRAINT_H */
