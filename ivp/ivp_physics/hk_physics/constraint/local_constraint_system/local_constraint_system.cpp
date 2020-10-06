#include <ivp_physics.hxx>
#include <ivp_template_constraint.hxx>

#include <hk_physics/physics.h>

#include <hk_physics/constraint/constraint.h>
#include <hk_physics/constraint/local_constraint_system/local_constraint_system.h>
#include <hk_physics/constraint/local_constraint_system/local_constraint_system_bp.h>

#include <hk_physics/constraint/limited_ball_socket/limited_ball_socket_bp.h>
#include <hk_physics/constraint/limited_ball_socket/limited_ball_socket_constraint.h>
#include <hk_physics/constraint/ragdoll/ragdoll_constraint_bp.h>
#include <hk_physics/constraint/ragdoll/ragdoll_constraint.h>
#include <hk_physics/constraint/ragdoll/ragdoll_constraint_bp_builder.h>



hk_Local_Constraint_System::hk_Local_Constraint_System( hk_Environment *env, hk_Local_Constraint_System_BP *bp )
	: hk_Link_EF(env)
{
	m_size_of_all_vmq_storages = 0;
	m_is_active = false;
}

hk_Local_Constraint_System::~hk_Local_Constraint_System()
{
	for ( hk_Array<hk_Constraint *>::iterator i = m_constraints.start();
			m_constraints.is_valid(i);
			i = m_constraints.next( i ) )
	{
		hk_Constraint *constraint = m_constraints.get_element(i);
		constraint->constraint_system_deleted_event( this );
	}

	if ( m_is_active )
	{
		m_environment->get_controller_manager()->remove_controller_from_environment(this,IVP_TRUE); //silently    
	}
}


//@@CB
void hk_Local_Constraint_System::entity_deletion_event(hk_Entity *entity)
{
	hk_Constraint* constraint;

	for ( hk_Array<hk_Constraint *>::iterator i = m_constraints.start();
			m_constraints.is_valid(i);
			i = m_constraints.next( i ) )
	{
		constraint = m_constraints.get_element(i);

		if(constraint->get_rigid_body(0) == entity || constraint->get_rigid_body(1) == entity)
		{
			delete constraint;
		}
	}

	m_bodies.search_and_remove_element(entity);
	actuator_controlled_cores.remove(entity->get_core());

//	HK_BREAK;
}


//@@CB
void hk_Local_Constraint_System::core_is_going_to_be_deleted_event(IVP_Core *my_core)
{
	hk_Rigid_Body* rigid_body;

	if( m_bodies.length() )
	{
		for ( hk_Array<hk_Rigid_Body *>::iterator i = m_bodies.start();
				m_bodies.is_valid(i);
				i = m_bodies.next( i ) )
		{
			rigid_body = m_bodies.get_element(i);

			if(rigid_body->get_core() == my_core)
			{
				this->entity_deletion_event(rigid_body);
			}
		}
	}
}


void hk_Local_Constraint_System::constraint_deletion_event( hk_Constraint *constraint )
{
	m_constraints.search_and_remove_element_sorted(constraint);
	if ( m_constraints.length() != 0)
	{
		recalc_storage_size();
	}
}


void hk_Local_Constraint_System::recalc_storage_size()
{
	m_size_of_all_vmq_storages = 0;
	for ( hk_Array<hk_Constraint *>::iterator i = m_constraints.start();
			m_constraints.is_valid(i);
			i = m_constraints.next( i ) )
	{
		m_size_of_all_vmq_storages += m_constraints.get_element(i)->get_vmq_storage_size();
	}
}


void hk_Local_Constraint_System::add_constraint( hk_Constraint * constraint, int storage_size)
{
	bool isActive = m_is_active;
	if ( isActive )
	{
		deactivate();
	}
	m_constraints.add_element( constraint );
	
	int i = 1;
	do 
	{
		hk_Rigid_Body *b = constraint->get_rigid_body(i);
		if ( m_bodies.index_of( b ) <0)
		{
			m_bodies.add_element(b);
			if ( !b->get_core()->physical_unmoveable )
			{
				actuator_controlled_cores.add( b->get_core());
			}
		}
	} while (--i>=0);

	if ( isActive )
	{
		activate();
	}
	m_size_of_all_vmq_storages += storage_size;	
}

void hk_Local_Constraint_System::activate()
{
	if ( !m_is_active && m_bodies.length() )
	{
	    m_environment->get_controller_manager()->announce_controller_to_environment(this);
		m_is_active = true;
	}
}

//@@CB
void hk_Local_Constraint_System::deactivate()
{
	if ( m_is_active && actuator_controlled_cores.len() )
	{
		m_environment->get_controller_manager()->remove_controller_from_environment(this, IVP_FALSE);
		m_is_active = false;
	}
}

//@@CB
void hk_Local_Constraint_System::deactivate_silently()
{
	if ( m_is_active && actuator_controlled_cores.len() )
	{
		m_is_active = false;
		m_environment->get_controller_manager()->remove_controller_from_environment(this, IVP_TRUE);
	}
}

void hk_Local_Constraint_System::get_effected_entities(hk_Array<hk_Entity*> &ent_out)
{
	for ( hk_Array<hk_Entity*>::iterator i = m_bodies.start();
			m_bodies.is_valid(i);
			i = m_bodies.next(i))
	{
		ent_out.add_element( m_bodies.get_element(i));
	}
}
		
//virtual hk_real get_minimum_simulation_frequency(hk_Array<hk_Entity> *);

void hk_Local_Constraint_System::apply_effector_PSI( hk_PSI_Info& pi, hk_Array<hk_Entity*>* )
{
	const int buffer_size = 150000;
	const int max_constraints = 1000;
	void *vmq_buffers[ max_constraints ];
	char buffer[buffer_size];
	HK_ASSERT( m_size_of_all_vmq_storages < buffer_size );

	hk_real taus[] =  { 1.0f, 1.0f, 0.8f, 0.6f, 0.4f, 0.4f, 0.4f, 0.4f, 0.4f, 0.0f };
	hk_real damps[] = { 1.0f, 1.0f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.0f };
	//first do the setup
	{
		char *p_buffer = &buffer[0];
		for ( int i = 0; i < m_constraints.length(); i++ ){
			vmq_buffers[i] = (void *)p_buffer;
			int b_size = m_constraints.element_at(i)->setup_and_step_constraint( pi, (void *) p_buffer,1.0f, 1.0f);
			p_buffer += b_size;
		}
	}

	// do the steps
	for (int x = 0; x<2 && taus[x]!=0; x++)	
	{ 
		for ( int i = m_constraints.length()-1; i >=0 ; i-- ){
			m_constraints.element_at(i)->step_constraint( pi, (void *)vmq_buffers[i],taus[x], damps[x]);
		}
		for ( int j = 0; j < m_constraints.length(); j++ ){
			m_constraints.element_at(j)->step_constraint( pi, (void *)vmq_buffers[j],taus[x], damps[x]);
		}
	}
}

//lwss add
void hk_Local_Constraint_System::write_to_blueprint(hk_Local_Constraint_System_BP *pOutBP)
{

}
//lwss end

hk_real hk_Local_Constraint_System::get_epsilon()
{
	return 0.2f;
}

hk_Limited_Ball_Socket_BP *ivp_create_limited_ball_socket_bp(IVP_Template_Constraint *tmpl )
{
	
    IVP_Real_Object *objectR = tmpl->objectR;
    IVP_Real_Object *objectA = tmpl->objectA;

    IVP_U_Matrix m_ws_f_Ros; if (objectR) objectR->get_m_world_f_object_AT(&m_ws_f_Ros); else m_ws_f_Ros.init();
    IVP_U_Matrix m_ws_f_Aos; if (objectA) objectA->get_m_world_f_object_AT(&m_ws_f_Aos); else m_ws_f_Aos.init();
    IVP_U_Matrix m_Ros_f_Aos; m_ws_f_Ros.mimult4(&m_ws_f_Aos, &m_Ros_f_Aos);

    // do constraints in object space, skip core xform
	IVP_U_Matrix m_Rcs_f_Ros; m_Rcs_f_Ros.init();
    IVP_U_Matrix m_Acs_f_Aos; m_Acs_f_Aos.init();

    IVP_U_Matrix m_Rcs_f_Aos; m_Rcs_f_Ros.mmult4(&m_Ros_f_Aos, &m_Rcs_f_Aos);
    IVP_U_Matrix m_Rcs_f_Acs; m_Rcs_f_Aos.mi2mult4(&m_Acs_f_Aos, &m_Rcs_f_Acs);
    
    IVP_U_Matrix m_Rcs_f_Rfs;

    IVP_U_Matrix m_Acs_f_Afs;
    {
		if (tmpl->m_Ros_f_Rfs) 
		{
			m_Rcs_f_Ros.mmult4(tmpl->m_Ros_f_Rfs, &m_Rcs_f_Rfs); 
		} 
		else 
		{
			m_Rcs_f_Rfs.set_matrix(&m_Rcs_f_Ros);
		}


		IVP_U_Matrix m_Rfs_f_Rcs;
		m_Rfs_f_Rcs.real_invert( &m_Rcs_f_Rfs );

		if (tmpl->m_Aos_f_Afs) 
		{
			m_Acs_f_Aos.mmult4(tmpl->m_Aos_f_Afs, &m_Acs_f_Afs);
		} 
		else 
		{ // Afs is Rfs
			m_Rfs_f_Rcs.mmult4(&m_Rcs_f_Acs, &m_Acs_f_Afs); m_Acs_f_Afs.real_invert();
		}
	}


	hk_Limited_Ball_Socket_BP *bp = new hk_Limited_Ball_Socket_BP();
    hk_Limited_Ball_Socket_BP &hbp = *bp;;

    m_Rcs_f_Rfs.get_4x4_column_major( hbp.m_transform_os_ks[0].get_elem_address(0,0) );
    m_Acs_f_Afs.get_4x4_column_major( hbp.m_transform_os_ks[1].get_elem_address(0,0) );

    for (int i = 0; i< 3; i++)
	{
		if ( tmpl->axis_type[i+3] == IVP_CONSTRAINT_AXIS_LIMITED )
		{
			hbp.m_angular_limits[i].set( tmpl->borderleft_Rfs[i+3], tmpl->borderright_Rfs[i+3]);
		}
		else
		{
			hbp.m_angular_limits[i].set( 0.0f, 0.0f );
		}
    }
	return bp;
}


hk_Constraint *ivp_create_ragdoll_constraint_from_local_constraint_system(hk_Local_Constraint_System *lcs, IVP_Template_Constraint *tmpl )
{
    IVP_Real_Object *objectR = tmpl->objectR;
    IVP_Real_Object *objectA = tmpl->objectA;

    IVP_U_Matrix m_ws_f_Ros; if (objectR) objectR->get_m_world_f_object_AT(&m_ws_f_Ros); else m_ws_f_Ros.init();
    IVP_U_Matrix m_ws_f_Aos; if (objectA) objectA->get_m_world_f_object_AT(&m_ws_f_Aos); else m_ws_f_Aos.init();
    IVP_U_Matrix m_Ros_f_Aos; m_ws_f_Ros.mimult4(&m_ws_f_Aos, &m_Ros_f_Aos);
    
	// NOTE: disabled core xform.
	// havana constraints are now solved in object space
	IVP_U_Matrix m_Rcs_f_Ros; m_Rcs_f_Ros.init();
    IVP_U_Matrix m_Acs_f_Aos; m_Acs_f_Aos.init();

    IVP_U_Matrix m_Rcs_f_Aos; m_Rcs_f_Ros.mmult4(&m_Ros_f_Aos, &m_Rcs_f_Aos);
    IVP_U_Matrix m_Rcs_f_Acs; m_Rcs_f_Aos.mi2mult4(&m_Acs_f_Aos, &m_Rcs_f_Acs);
    
    IVP_U_Matrix m_Rcs_f_Rfs;

    IVP_U_Matrix m_Acs_f_Afs;
    {
		if (tmpl->m_Ros_f_Rfs) 
		{
			m_Rcs_f_Ros.mmult4(tmpl->m_Ros_f_Rfs, &m_Rcs_f_Rfs); 
		} 
		else 
		{
			m_Rcs_f_Rfs.set_matrix(&m_Rcs_f_Ros);
		}


		IVP_U_Matrix m_Rfs_f_Rcs;
		m_Rfs_f_Rcs.real_invert( &m_Rcs_f_Rfs );

		if (tmpl->m_Aos_f_Afs) 
		{
			m_Acs_f_Aos.mmult4(tmpl->m_Aos_f_Afs, &m_Acs_f_Afs);
		} 
		else 
		{ // Afs is Rfs
			m_Rfs_f_Rcs.mmult4(&m_Rcs_f_Acs, &m_Acs_f_Afs); m_Acs_f_Afs.real_invert();
		}
    }


	hk_Limited_Ball_Socket_BP hbp;

    m_Rcs_f_Rfs.get_4x4_column_major( hbp.m_transform_os_ks[0].get_elem_address(0,0) );
    m_Acs_f_Afs.get_4x4_column_major( hbp.m_transform_os_ks[1].get_elem_address(0,0) );

    for (int i = 0; i< 3; i++)
	{
		if ( tmpl->axis_type[i+3] == IVP_CONSTRAINT_AXIS_LIMITED )
		{
			hbp.m_angular_limits[i].set( tmpl->borderleft_Rfs[i+3], tmpl->borderright_Rfs[i+3]);
		}
		else if (tmpl->axis_type[i+3] == IVP_CONSTRAINT_AXIS_FIXED)
		{
			hbp.m_angular_limits[i].set( 0.0f, 0.0f );
		}
		else 
		{
			hbp.m_angular_limits[i].set( -10.0f, 10.0f );		// no limits
		}
    }

	hk_Ragdoll_Constraint_BP_Builder rb;
	hk_result res = rb.initialize_from_limited_ball_socket_bp( &hbp, (hk_Rigid_Body *)objectR, (hk_Rigid_Body *)objectA );

	if (res == HK_FAULT)
	{
	    return new hk_Limited_Ball_Socket_Constraint( lcs,	&hbp, (hk_Rigid_Body *)objectR, (hk_Rigid_Body *)objectA );
	}

	const hk_Ragdoll_Constraint_BP *rbp = rb.get_blueprint();

	return new hk_Ragdoll_Constraint( lcs,	rbp, (hk_Rigid_Body *)objectR, (hk_Rigid_Body *)objectA );
}


