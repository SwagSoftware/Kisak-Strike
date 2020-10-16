// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_U_Active_Float ist ein modular kombinierbarer Zahlengeber (Synthesizer)
// und Event-Ausloeser zur Implementierung der Spiel-Logik ausserhalb
// der eigentlichen Grundphysik.
// z.B. fuer IVP_Actuator_Forces (oder Klaenge :-)

// IVP_Active_Float is a modular combinable number generator to implement
// some logics stuff. It is no internal part of the physics but some
// add on for convenience. It is based on event-callbacks to update the
// values when it is necessary. It is used e.g. for IVP_Actuator_Forces.

#include <ivp_physics.hxx>
#include <string.h>
#ifndef WIN32
#	pragma implementation "ivu_active_value.hxx"
#	pragma implementation "ivu_active_value_hash.hxx"
#endif

#include <ivu_active_value.hxx>
#include <ivu_active_value_hash.hxx>

IVP_Active_Value_Hash::~IVP_Active_Value_Hash() 
{
    for (int i=this->len()-1; i>=0; i--) {
	IVP_U_Active_Value *av;
	av = (IVP_U_Active_Value *)this->element_at(i);
	if (av){
		av->remove_reference();
	}
    }
}

int IVP_Active_Value_Hash::object_to_index(IVP_U_Active_Value *av)
{
    const char *name = av->get_name();
    return hash_index( name, strlen( name) );
}

IVP_BOOL IVP_Active_Value_Hash::compare(void *elem0, void *elem1) const
{
    IVP_U_Active_Value *av0 = (IVP_U_Active_Value *)elem0;
    IVP_U_Active_Value *av1 = (IVP_U_Active_Value *)elem1;

    const char *name0 = av0->get_name();
    const char *name1 = av1->get_name();
    if (p_strcmp(name0, name1) == 0) return IVP_TRUE;
    return IVP_FALSE;
}



int IVP_U_Active_Float::change_meter = -1;

IVP_U_Active_Value_Manager::IVP_U_Active_Value_Manager(IVP_BOOL delete_on_env_delete_in)
{
    delete_on_env_delete = delete_on_env_delete_in;
    // init name hash
    this->floats_name_hash = new IVP_Active_Value_Hash(IVP_U_MOD_NAME_HASH_SIZE);
    this->ints_name_hash = new IVP_Active_Value_Hash(IVP_U_MOD_NAME_HASH_SIZE);
    this->search_active_value = new IVP_U_Active_Value(NULL);
    this->mod_current_time = NULL;
    this->init_active_values_generic();
}

IVP_U_Active_Value_Manager::~IVP_U_Active_Value_Manager()
{
    P_DELETE(this->floats_name_hash);   // removes references two
    P_DELETE(this->ints_name_hash);
    P_DELETE(this->search_active_value);
}

void IVP_U_Active_Value_Manager::init_active_values_generic()
{
    IVP_U_Active_Terminal_Double *mod_double_null = new IVP_U_Active_Terminal_Double("double_null", 0.0f);
    this->mod_current_time = new IVP_U_Active_Terminal_Double(IVP_ACTIVE_FLOAT_CURRENT_TIME_NAME, 0.0f);    
    this->insert_active_float(mod_double_null);
    this->insert_active_float(mod_current_time);
}


void IVP_U_Active_Value_Manager::insert_active_float(IVP_U_Active_Float *mod)
{
    const char *name = mod->get_name();
    if(p_strlen(name)==0){
	printf("insert_active_float: tried to insert active_IVP_FLOAT without name!\n");
	return;
    }
    
    IVP_U_Active_Float *found = (IVP_U_Active_Float *)floats_name_hash->find_active_value(mod);
    if(found){
	printf("insert_active_float: name '%s' already exists in name_hash!\n", name);
	return;
    }

    floats_name_hash->add_active_value(mod);
    mod->l_mod_manager = this; // provide backlink
}

void IVP_U_Active_Value_Manager::insert_active_int(IVP_U_Active_Int *mod)
{
    const char *name = mod->get_name();
    if(p_strlen(name)==0){
	printf("insert_active_int: tried to insert active_int without name!\n");
	return;
    }
    
    IVP_U_Active_Int *found = (IVP_U_Active_Int *)ints_name_hash->find_active_value(mod);
    if(found){
	printf("insert_active_int: name '%s' already exists in name_hash!\n", name);
	return;
    }

    ints_name_hash->add_active_value(mod);
    mod->add_reference();

    mod->l_mod_manager = this; // provide backlink
}



void IVP_U_Active_Value_Manager::remove_active_float(IVP_U_Active_Float *mod)
{
    floats_name_hash->remove_active_value(mod);
}

void IVP_U_Active_Value_Manager::remove_active_int(IVP_U_Active_Int *mod)
{
    ints_name_hash->remove_active_value(mod);
}

void IVP_U_Active_Value_Manager::delay_active_float(IVP_U_Active_Float_Delayed *mod)
{
    if (delayed_active_floats.index_of(mod) == -1){
	delayed_active_floats.add(mod);
    }
}

void IVP_U_Active_Value_Manager::delay_active_int(IVP_U_Active_Int_Delayed *mod)
{
    if (delayed_active_ints.index_of(mod) == -1){
	delayed_active_ints.add(mod);
    }
}

void IVP_U_Active_Value_Manager::update_delayed_active_values()
{
    // update all active_floats contained in vector delayed_active_floats
    {
	IVP_U_Active_Float_Delayed *mod;
	int i;
	for (i = 0; i< delayed_active_floats.len();i++){
	    mod = delayed_active_floats.element_at(i);
	    mod->update_float();
	}
	delayed_active_floats.clear();
    }

    {
	IVP_U_Active_Int_Delayed *mod;
	int i;
	for (i = 0; i< delayed_active_ints.len();i++){
	    mod = delayed_active_ints.element_at(i);
	    mod->update_int();
	}
	delayed_active_ints.clear();
    }

}

void IVP_U_Active_Value_Manager::refresh_psi_active_values(IVP_Environment *env)
{
    if (mod_current_time){
	mod_current_time->set_double(env->get_current_time().get_time());
    }
    update_delayed_active_values();
}

IVP_U_Active_Float *IVP_U_Active_Value_Manager::get_active_float_by_name(const char *i_name)
{
    if(!i_name) return 0;
    // check module hash first
    search_active_value->name = (char *)i_name;// bad hack, thats ok
    
    IVP_U_Active_Float *mod = (IVP_U_Active_Float *)floats_name_hash->find_active_value(search_active_value);

    search_active_value->name = NULL;
    
    if (mod) return mod;

    // if module name is a number, generate terminal active_floats with
    // that (IVP_DOUBLE) value on the fly.    
    int pos = 0;
    if(i_name[pos] == '-') pos++; // for negative sign
    if(i_name[pos] == '.') pos++; // for omitted leading zero
    if(i_name[pos] >= '0' && i_name[pos] <= '9'){
	mod = new IVP_U_Active_Terminal_Double(i_name, p_atof(i_name));
	this->insert_active_float(mod);
	return mod;
    }
    return NULL;
}

IVP_U_Active_Int *IVP_U_Active_Value_Manager::get_active_int_by_name(const char *i_name)
{
    if(!i_name) return 0;
    // check module hash first
    search_active_value->name = (char *)i_name;// bad hack, thats ok

    IVP_U_Active_Int *mod = (IVP_U_Active_Int *)ints_name_hash->find_active_value(search_active_value);
    search_active_value->name = NULL;
    
    if (mod) return mod;

    // if module name is a number, generate terminal active_floats with
    // that (IVP_DOUBLE) value on the fly.    
    int pos = 0;
    if(i_name[pos] == '-') pos++; // for negative sign
    if(i_name[pos] == '.') pos++; // for omitted leading zero
    if(i_name[pos] >= '0' && i_name[pos] <= '9'){
	mod = new IVP_U_Active_Terminal_Int(i_name, p_atoi(i_name));
	this->insert_active_int(mod);
	return mod;
    }
    return NULL;
}

IVP_U_Active_Float *IVP_U_Active_Value_Manager::install_active_float(const char *i_name, IVP_DOUBLE value)
{
    // check module hash first
    search_active_value->name = (char *)i_name;// bad hack, thats ok
    IVP_U_Active_Float *mod = (IVP_U_Active_Float *)floats_name_hash->find_active_value(search_active_value);
    search_active_value->name = NULL;

    if (mod) return mod;

    IVP_U_Active_Terminal_Double *atd = new IVP_U_Active_Terminal_Double(i_name, value);
    this->insert_active_float(atd);
    return atd;
}

IVP_U_Active_Terminal_Double *IVP_U_Active_Value_Manager::create_active_float(const char *i_name, IVP_DOUBLE value)
{
    // check module hash first
    search_active_value->name = (char *)i_name;// bad hack, thats ok
    IVP_U_Active_Float *mod = (IVP_U_Active_Float *)floats_name_hash->find_active_value(search_active_value);
    search_active_value->name = NULL;

    if (mod) return NULL;

    IVP_U_Active_Terminal_Double *atd = new IVP_U_Active_Terminal_Double(i_name, value);
    this->insert_active_float(atd);
    return atd;
}

IVP_U_Active_Terminal_Int *IVP_U_Active_Value_Manager::create_active_int(const char *i_name, int value)
{
    // check module hash first
    search_active_value->name = (char *)i_name;// bad hack, thats ok
    IVP_U_Active_Int *mod = (IVP_U_Active_Int *)ints_name_hash->find_active_value(search_active_value);
    search_active_value->name = NULL;
    if (mod) return NULL;

    IVP_U_Active_Terminal_Int *atd = new IVP_U_Active_Terminal_Int(i_name, value);
    this->insert_active_int(atd);
    return atd;
}



IVP_U_Active_Int *IVP_U_Active_Value_Manager::install_active_int(const char *i_name, int value)
{
    // check module hash first
    search_active_value->name = (char *)i_name;// bad hack, thats ok
    IVP_U_Active_Int *mod = (IVP_U_Active_Int *)ints_name_hash->find_active_value(search_active_value);
    search_active_value->name = NULL;
    if (mod) return mod;

    IVP_U_Active_Terminal_Int *atd = new IVP_U_Active_Terminal_Int(i_name, value);
    this->insert_active_int(atd);
    return atd;
}


//////////////
IVP_U_Active_Value::IVP_U_Active_Value(const char *name_){
    name = p_strdup(name_);
    reference_count = 0;
}

IVP_U_Active_Value::~IVP_U_Active_Value(){
    P_FREE(name);
    IVP_ASSERT(reference_count == 0);
}

IVP_U_Active_Float::IVP_U_Active_Float(const char *i_name) : IVP_U_Active_Value(i_name)
{
    // memclear not allowed because of virt. functions
    this->last_update = 0; // should be updated
    this->l_mod_manager = NULL; // link is done by manager
    this->double_value = 0.0f;
}

IVP_U_Active_Int::IVP_U_Active_Int(const char *i_name): IVP_U_Active_Value(i_name)
{
    // memclear not allowed because of virt. functions
    this->last_update = 0; // should be updated
    this->l_mod_manager = NULL; // link is done by manager
    this->int_value = 0;
}


IVP_U_Active_Float::~IVP_U_Active_Float()
{ 
    // if(name_hash){
    //     name_hash->remove(this->name);
    // }
    // must be done by hand (or by manager), no arg supported by destructor !
    return;    
}

IVP_U_Active_Int::~IVP_U_Active_Int()
{ 
    // if(name_hash){
    //     name_hash->remove(this->name);
    // }
    // must be done by hand (or by manager), no arg supported by destructor !
    
}

void IVP_U_Active_Float::update_derived()
{
    // update all active_float_listener which depend on my value

    for (int i = derived_mods.len()-1; i>=0;i--){
	IVP_U_Active_Float_Listener *mod = derived_mods.element_at(i);
	mod->active_float_changed(this);
    }
}

void IVP_U_Active_Int::update_derived()
{
    // update all active_float_listener which depend on my value

    for( int i = derived_mods.len()-1; i>=0; i--){
	IVP_U_Active_Int_Listener *mod= derived_mods.element_at(i);
	mod->active_int_changed(this);
    }
}

void IVP_U_Active_Float::add_dependency(IVP_U_Active_Float_Listener *derived_active_IVP_FLOAT)
{
    // add update callback
    this->derived_mods.add(derived_active_IVP_FLOAT);
    this->add_reference();
}

void IVP_U_Active_Float::remove_dependency(IVP_U_Active_Float_Listener *derived_active_IVP_FLOAT)
{
    // add update callback
    this->derived_mods.remove(derived_active_IVP_FLOAT);
    this->remove_reference();
}

void IVP_U_Active_Int::add_dependency(IVP_U_Active_Int_Listener *derived_active_int)
{
    // add update callback
    this->derived_mods.add(derived_active_int);
    this->add_reference();
}

void IVP_U_Active_Int::remove_dependency(IVP_U_Active_Int_Listener *derived_active_int)
{
    // add update callback
    this->derived_mods.remove(derived_active_int);
    this->remove_reference();
}



/**** BASIC VALUES ********************/
/**** BASIC VALUES ********************/
/**** BASIC VALUES ********************/


/****** 'terminal' IVP_DOUBLE value ******/

IVP_U_Active_Terminal_Double::IVP_U_Active_Terminal_Double(const char *i_name,
					     IVP_DOUBLE new_value) : IVP_U_Active_Float(i_name)
{
    this->double_value = new_value;
    this->old_value = new_value;
}

IVP_U_Active_Terminal_Int::IVP_U_Active_Terminal_Int(const char *i_name,int new_value) : IVP_U_Active_Int(i_name)
{
    this->int_value = new_value;
    this->old_value = new_value;
}

void IVP_U_Active_Terminal_Double::update_float()
{
    if(this->double_value != old_value){
	this->old_value = double_value;
	this->update_derived();
    }
}

void IVP_U_Active_Terminal_Int::update_int()
{
    if(this->int_value != old_value){
	this->old_value = int_value;
	this->update_derived();
    }
}

void IVP_U_Active_Terminal_Double::set_double(IVP_DOUBLE new_value, IVP_BOOL delayed_update)
{
    this->double_value = new_value;
    this->change_meter++;
    if(delayed_update && l_mod_manager){
	l_mod_manager->delay_active_float(this);
    }else{
	this->update_float();
    }
}

int IVP_U_Active_Terminal_Double::print()
{
    printf("DoubleVal");
    return 0; // for debugger
}

////////////////////



void IVP_U_Active_Terminal_Int::set_int(int new_value, IVP_BOOL delayed_update)
{
    this->int_value = new_value;
    IVP_U_Active_Float::change_meter++;

    if(delayed_update && l_mod_manager){
	l_mod_manager->delay_active_int(this);
    }else{
	this->update_int();
    }
}

int IVP_U_Active_Terminal_Int::print()
{
    printf("IntVal");
    return 0; // for debugger
}

/*** OSCILLATORS (time dependant) **************/
/*** OSCILLATORS (time dependant) **************/
/*** OSCILLATORS (time dependant) **************/

IVP_U_Active_Sine::IVP_U_Active_Sine(const char *i_name,
		       IVP_U_Active_Float *time_mod_,
		       IVP_DOUBLE freq, IVP_DOUBLE amp,
		       IVP_DOUBLE i_null_level,
		       IVP_DOUBLE i_time_shift
                      ) : IVP_U_Active_Float(i_name)
{
    this->time_mod = time_mod_;

    
    this->time_mod->add_dependency(this);

    this->frequence = freq;
    this->amplitude = amp;
    this->null_level = i_null_level;
    this->time_shift = i_time_shift;

    this->active_float_changed(this); // no derived existing yet!
}

IVP_U_Active_Sine::~IVP_U_Active_Sine()
{
    this->time_mod->remove_dependency(this);
	return;
}

void IVP_U_Active_Sine::active_float_changed(IVP_U_Active_Float *)
{    
    if(this->last_update == this->change_meter) return; // already valid
    this->last_update = this->change_meter;

    IVP_DOUBLE time = time_mod->give_double_value();
    IVP_DOUBLE new_val = IVP_Inline_Math::sind(time * frequence + time_shift) * amplitude + null_level;

    if(new_val != this->double_value){
	this->double_value = new_val;
	this->update_derived();
    }
}

int IVP_U_Active_Sine::print()
{
    printf("Sine[F %g, A %g, N %g, ts %g](", frequence, amplitude, null_level, time_shift);
    time_mod->print();
    printf(")");
    return 0; // for debugger
}

///////////////////

IVP_U_Active_Square::IVP_U_Active_Square(const char *i_name,
			   IVP_U_Active_Float *i_time_mod,
			   IVP_DOUBLE i_freq,
			   IVP_DOUBLE i_low_val,
			   IVP_DOUBLE i_high_val
                          ) : IVP_U_Active_Float(i_name)
{
    this->time_mod = i_time_mod;

    time_mod->add_dependency(this);

    this->frequence = i_freq;
    this->low_val = i_low_val;
    this->high_val = i_high_val;
}

IVP_U_Active_Square::~IVP_U_Active_Square()
{
    time_mod->remove_dependency(this);
	return;
}

void IVP_U_Active_Square::active_float_changed(IVP_U_Active_Float *)
{
    if(last_update == change_meter) return; // already valid
    last_update = change_meter;
    
    IVP_DOUBLE time = time_mod->give_double_value();
    int h = (int)(time * frequence) & 1;
    IVP_DOUBLE new_val = (h) ? high_val : low_val;
    
    if(new_val != this->double_value){
	this->double_value = new_val;
	this->update_derived();
    }
}

int IVP_U_Active_Square::print()
{
    printf("Square[F %g, L %g, H %g](", frequence, low_val, high_val);
    time_mod->print();
    printf(")");
    return 0; // f db
}

/////////////////

IVP_U_Active_Pulse::IVP_U_Active_Pulse(const char *i_name,
				   IVP_U_Active_Float *i_time_mod,
				   IVP_DOUBLE freq,
				   int i_m1,
				   int i_m2,
				   IVP_DOUBLE i_low_val,
				   IVP_DOUBLE i_high_val
                                   ) : IVP_U_Active_Float(i_name)
{
    this->time_mod = i_time_mod;

    time_mod->add_dependency(this);

    this->frequence = freq;
    this->low_val = i_low_val;
    this->high_val = i_high_val;
    this->m1 = i_m1;
    this->m2 = i_m2;

    this->active_float_changed(this);
}

IVP_U_Active_Pulse::~IVP_U_Active_Pulse()
{
    time_mod->remove_dependency(this);
	return;
}

void IVP_U_Active_Pulse::active_float_changed(IVP_U_Active_Float *)
{
    if(last_update == change_meter) return; // already valid
    last_update = change_meter;
    
    IVP_DOUBLE time = time_mod->give_double_value();
    int val = (int)(time * frequence * m2) % m2;
    IVP_DOUBLE new_val = (val < m1) ? high_val : low_val;
    
    if(new_val != this->double_value){
	this->double_value = new_val;
	this->update_derived();
    }
}

int IVP_U_Active_Pulse::print()
{
    printf("Pulse[F %g, L %g, H %g, %d:%d](", frequence, low_val, high_val, m1, m2);
    time_mod->print();
    printf(")");
    return 0; // f db
}

/***** MIXER ***********************************/
/***** MIXER ***********************************/
/***** MIXER ***********************************/

IVP_U_Active_Add::IVP_U_Active_Add(const char *i_name,
		     IVP_U_Active_Float *i_mod0,
		     IVP_U_Active_Float *i_mod1) : IVP_U_Active_Float(i_name)
{
    this->mod0 = i_mod0;
    this->mod1 = i_mod1;

    mod0->add_dependency(this);
    mod1->add_dependency(this);
    
    this->active_float_changed(this);
}

IVP_U_Active_Add::~IVP_U_Active_Add()
{
    mod0->remove_dependency(this);
    mod1->remove_dependency(this);
	return;
}

void IVP_U_Active_Add::active_float_changed(IVP_U_Active_Float *)
{
    if(last_update == change_meter) return; // already valid
    last_update = change_meter;
    
    IVP_DOUBLE new_val = mod0->give_double_value() + mod1->give_double_value();
    
    if(new_val != this->give_double_value()){
	this->double_value = new_val;
	this->update_derived();
    }
}

int IVP_U_Active_Add::print()
{
    mod0->print();
    printf(" + ");
    mod1->print();
    
    return 0; // for debugger
}

///////////////////////

IVP_U_Active_Sub::IVP_U_Active_Sub(const char *i_name,
		     IVP_U_Active_Float *i_mod0,
		     IVP_U_Active_Float *i_mod1) : IVP_U_Active_Float(i_name)
{

    this->mod0 = i_mod0;
    this->mod1 = i_mod1;

    mod0->add_dependency(this);
    mod1->add_dependency(this);
    
    this->active_float_changed(this);
}

IVP_U_Active_Sub::~IVP_U_Active_Sub()
{
    mod0->remove_dependency(this);
    mod1->remove_dependency(this);
	return;
}

void IVP_U_Active_Sub::active_float_changed(IVP_U_Active_Float *)
{
    if(last_update == change_meter) return; // already valid
    last_update = change_meter;
    
    IVP_DOUBLE new_val = mod0->give_double_value() - mod1->give_double_value();
    
    if(new_val != this->double_value){
	this->double_value = new_val;
	this->update_derived();
    }
}

int IVP_U_Active_Sub::print()
{
    mod0->print();
    printf(" - ");
    mod1->print();
    
    return 0; // for debugger
}

///////////////////////


IVP_U_Active_Add_Multiple::IVP_U_Active_Add_Multiple(const char *i_name,
				       IVP_U_Active_Float *i_mod0,
				       IVP_U_Active_Float *i_mod1,
				       IVP_DOUBLE i_factor) : IVP_U_Active_Float(i_name)
{
    this->mod0 = i_mod0;
    this->mod1 = i_mod1;

    this->factor = i_factor;

    mod0->add_dependency(this);
    mod1->add_dependency(this);

    this->active_float_changed((IVP_U_Active_Float *)this);
}

IVP_U_Active_Add_Multiple::~IVP_U_Active_Add_Multiple()
{
    mod0->remove_dependency(this);
    mod1->remove_dependency(this);
	return;
}

void IVP_U_Active_Add_Multiple::active_float_changed(IVP_U_Active_Float *)
{
    if(last_update == change_meter) return; // already valid
    last_update = change_meter;
    
    IVP_DOUBLE new_val = mod0->give_double_value() + factor * mod1->give_double_value();
    
    if(new_val != this->double_value){
	this->double_value = new_val;
	this->update_derived();
    }
}

int IVP_U_Active_Add_Multiple::print()
{
    printf("(");
    mod0->print();
    printf(")");

    printf(" + %g * (", factor);
    mod1->print();
    printf(")");
    
    return 0; // f db
}

////////////////////

IVP_U_Active_Mult::IVP_U_Active_Mult(const char *i_name,
		       IVP_U_Active_Float *i_mod0,
		       IVP_U_Active_Float *i_mod1) : IVP_U_Active_Float(i_name)
{
    this->mod0 = i_mod0;
    this->mod1 = i_mod1;

    mod0->add_dependency(this);
    mod1->add_dependency(this);

    this->active_float_changed((IVP_U_Active_Float *)this);
}

IVP_U_Active_Mult::~IVP_U_Active_Mult()
{
    mod0->remove_dependency(this);
    mod1->remove_dependency(this);
	return;
}

void IVP_U_Active_Mult::active_float_changed(IVP_U_Active_Float *)
{
    if(last_update == change_meter) return; // already valid
    last_update = change_meter;
    
    IVP_DOUBLE new_val = mod0->give_double_value() * mod1->give_double_value();
    
    if(new_val != this->double_value){
	this->double_value = new_val;
	this->update_derived();
    }
}

int IVP_U_Active_Mult::print()
{
    printf("(");
    mod0->print();
    printf(")");

    printf(" * ");

    printf("(");
    mod1->print();
    printf(")");
    
    return 0; // f db
}

/**** FILTER **************************/
/**** FILTER **************************/
/**** FILTER **************************/

IVP_U_Active_Limit::IVP_U_Active_Limit(const char *i_name,
			 IVP_U_Active_Float *i_mod,
			 IVP_DOUBLE i_low_val,
			 IVP_DOUBLE i_high_val) : IVP_U_Active_Float(i_name)
{
    this->mod = i_mod;

    
    this->low_val = i_low_val;
    this->high_val = i_high_val;

    mod->add_dependency(this);

    this->active_float_changed((IVP_U_Active_Float *)this);
}

IVP_U_Active_Limit::~IVP_U_Active_Limit()
{
    mod->remove_dependency(this);
	return;
}

void IVP_U_Active_Limit::active_float_changed(IVP_U_Active_Float *)
{
    if(last_update == change_meter) return; // already valid
    last_update = change_meter;
    
    IVP_DOUBLE new_val = mod->give_double_value();
    if (new_val < low_val) new_val = low_val;
    if (new_val > high_val) new_val = high_val;
    
    if(new_val != this->double_value){
	this->double_value = new_val;
	this->update_derived();
    }
}

int IVP_U_Active_Limit::print()
{
    printf("Limit[%g, %g](", low_val, high_val);
    mod->print();
    printf(")");
    return 0; // for debugger
}

/********** LOGICS ****************/
/********** LOGICS ****************/
/********** LOGICS ****************/


IVP_U_Active_Test_Range::IVP_U_Active_Test_Range(const char *i_name,
			 IVP_U_Active_Float *i_mod_test,
			 IVP_U_Active_Float *i_mod_low_val,
			 IVP_U_Active_Float *i_mod_high_val ) : IVP_U_Active_Int(i_name)
{

    this->mod_test = i_mod_test;
    
    this->mod_low_val = i_mod_low_val;
    
    this->mod_high_val = i_mod_high_val;
    
    mod_test->add_dependency(this);
    mod_low_val->add_dependency(this);
    mod_high_val->add_dependency(this);

    this->active_float_changed((IVP_U_Active_Float *)this);
}

IVP_U_Active_Test_Range::~IVP_U_Active_Test_Range()
{
    mod_test->remove_dependency(this);
    mod_low_val->remove_dependency(this);
    mod_high_val->remove_dependency(this);
	return;
}

void IVP_U_Active_Test_Range::active_float_changed(IVP_U_Active_Float *)
{
    if(last_update == IVP_U_Active_Float::change_meter) return; // already valid
    last_update = IVP_U_Active_Float::change_meter;

    IVP_DOUBLE cmp_val = mod_test->give_double_value();
    IVP_DOUBLE low_val = mod_low_val->give_double_value();
    IVP_DOUBLE high_val = mod_high_val->give_double_value();
    int new_val = ((cmp_val>=low_val)&&(cmp_val<=high_val)) ? 1 : 0;
    if(new_val != this->int_value){
	this->int_value = new_val;
	this->update_derived();
    }
}

int IVP_U_Active_Test_Range::print()
{
    printf("TestRange[");
    mod_low_val->print();
    printf(", ");
    mod_high_val->print();
    printf("](");
    mod_test->print();
    printf(")");
    return 0; // fdb
}

////////////////
    
IVP_U_Active_Switch::IVP_U_Active_Switch(const char *i_name,
		       IVP_U_Active_Int *i_mod_cond,
		       IVP_U_Active_Float *i_mod_true,
		       IVP_U_Active_Float *i_mod_false ) : IVP_U_Active_Float(i_name)
{
    this->mod_cond = i_mod_cond;
    
    this->mod_true = i_mod_true;    
    this->mod_false = i_mod_false;
    
    mod_cond->add_dependency(this);
    mod_true->add_dependency(this);
    mod_false->add_dependency(this);

    this->active_float_changed(this);
}

IVP_U_Active_Switch::~IVP_U_Active_Switch()
{
    mod_cond->remove_dependency(this);
    mod_true->remove_dependency(this);
    mod_false->remove_dependency(this);
	return;
}

void IVP_U_Active_Switch::active_float_changed(IVP_U_Active_Float *)
{
    if(last_update == IVP_U_Active_Float::change_meter) return; // already valid
    last_update = IVP_U_Active_Float::change_meter;

    IVP_DOUBLE new_val;
    IVP_DOUBLE cond_val = mod_cond->give_int_value();
    if(cond_val){
	new_val = mod_true->give_double_value();
    }else{
	new_val = mod_false->give_double_value();
    }

    if(new_val != this->double_value){
	this->double_value = new_val;
	this->update_derived();
    }
}

void IVP_U_Active_Switch::active_int_changed(IVP_U_Active_Int *){
    active_float_changed(NULL);

}

int IVP_U_Active_Switch::print()
{
    printf("Cond[");
    mod_true->print();
    printf(", ");
    mod_false->print();
    printf("](");
    mod_cond->print();
    printf(")");
    return 0; // fdb
}



