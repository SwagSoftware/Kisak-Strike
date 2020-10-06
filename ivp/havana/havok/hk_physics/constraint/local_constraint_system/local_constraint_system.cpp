#include <hk_physics/physics.h>
#include <hk_physics/simunit/sim_manager.h>
#include <hk_physics/simunit/psi_info.h>
#include <hk_physics/constraint/constraint.h>
#include <hk_physics/constraint/local_constraint_system/local_constraint_system.h>

hk_Local_Constraint_System::hk_Local_Constraint_System( hk_Environment *env, hk_Local_Constraint_System_BP* bp )
	: hk_Link_EF(env)
{
	m_environment = env;
	m_size_of_all_vmq_storages = 0;
}

hk_Local_Constraint_System::~hk_Local_Constraint_System()
{
	while ( m_constraints.length() )
	{
		delete m_constraints.get_element( m_constraints.start());
	}

	//XXX hack for havok
	if(m_environment)
	{
		m_environment->get_sim_mgr()->remove_link_effector( this );
	}
}


void hk_Local_Constraint_System::entity_deletion_event(hk_Entity *entity)
{
	HK_BREAK;  // XXX fix me
}


void hk_Local_Constraint_System::constraint_deletion_event( hk_Constraint *constraint )
{
	m_constraints.search_and_remove_element_sorted(constraint);
	if ( m_constraints.length() == 0){
		delete this;
		return;
	}else{
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
	m_constraints.add_element( constraint );
	
	int i = 1;
	do {
		hk_Rigid_Body *b = constraint->get_rigid_body(i);
		if ( m_bodies.index_of( b ) <0){
			m_bodies.add_element(b );
		}
	} while (--i>=0);

	m_size_of_all_vmq_storages += storage_size;	
}

void hk_Local_Constraint_System::activate()
{
	if ( m_bodies.length() ){
		m_environment->get_sim_mgr()->add_link_effector( this, HK_PRIORITY_LOCAL_CONSTRAINT_SYSTEM );
	}
}

void hk_Local_Constraint_System::get_effected_entities(hk_Array<hk_Entity*> &ent_out)
{
	for ( hk_Array<hk_Entity*>::iterator i = m_bodies.start();
			m_bodies.is_valid(i);
			i = m_bodies.next(i))
	{
		ent_out.add_element( (hk_Entity*)m_bodies.get_element(i));
	}
}
		
	//virtual hk_real get_minimum_simulation_frequency(hk_Array<hk_Entity> *);

hk_real hk_Local_Constraint_System::get_epsilon()
{
	return 0.2f;
}

void hk_Local_Constraint_System::apply_effector_PSI(	hk_PSI_Info& pi, hk_Array<hk_Entity*>* )
{
	const int buffer_size = 150000;
	const int max_constraints = 1000;
	void *vmq_buffers[ max_constraints ];
	char buffer[buffer_size];
	HK_CHECK( m_size_of_all_vmq_storages < buffer_size );

	//hk_real taus[] = { 0.2f, 0.2f, 0.0f, 0.6f, 0.4f, 0.0f };
	hk_real taus[] = { 1.0f, 1.0f, 0.0f, 0.6f, 0.4f, 0.0f };
	hk_real damps[] = { 1.0f, 1.0f, 0.8f, 0.8f, 0.0f };
	//first do the setup
	{
		char *p_buffer = &buffer[0];
		for ( int i = 0; i < m_constraints.length(); i++ ){
//		for ( int i = m_constraints.length()-1; i>=0; i-- ){
			vmq_buffers[i] = (void *)p_buffer;
			int b_size = m_constraints.element_at(i)->setup_and_step_constraint( pi, (void *) p_buffer,1.0f, 1.0f);
			p_buffer += b_size;
		}
	}
	
//	return;
	// do the steps
	for (int x = 0; x< 10; x++)	{ 
		if ( taus[x]==0.0f ) break;
		for ( int i = m_constraints.length()-2; i >=0 ; i-- ){
			m_constraints.element_at(i)->step_constraint( pi, (void *)vmq_buffers[i],taus[x], damps[x]);
		}
		for ( int j = 1; j < m_constraints.length(); j++ ){
			m_constraints.element_at(j)->step_constraint( pi, (void *)vmq_buffers[j],taus[x], damps[x]);
		}
	}
}

