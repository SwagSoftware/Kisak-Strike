#ifndef HK_PHYSICS_CONSTRAINT_BP_H
#define HK_PHYSICS_CONSTRAINT_BP_H

// IVP_EXPORT_PUBLIC

enum hk_Constraint_Blueprint_Types
{
	HK_UNDEFINED_CONSTRAINT		= 0,
	HK_FEATHERSTONE				= 1,

	HK_NUM_CONSTRAINTS

};

typedef enum {
	HK_JOINT_REVOLUTE,
	HK_JOINT_PRISMATIC
} hk_joint_type;


class hk_Constraint_BP : public hk_Blueprint
{
public:
	hk_Vector3 m_axis_bs;
	hk_Vector3 m_joint_offset0_bs0;
	hk_Vector3 m_joint_offset1_bs1;
	hk_joint_type m_joint_type;

	hk_Constraint_Blueprint_Types m_cbp_type;

	hk_Constraint_BP()
		: hk_Blueprint( (hk_type)HK_CONSTRAINT_TYPE,
				sizeof(hk_Constraint_BP) )
	{
	}
};

#endif // HK_PHYSICS_CONSTRAINT_BP_H
