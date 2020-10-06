// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

// IVP_U_Active_Float ist ein modular kombinierbarer Zahlengeber (Synthesizer)
// und Event-Ausloeser zur Implementierung der Spiel-Logik ausserhalb
// der eigentlichen Grundphysik.
// z.B. fuer IVP_Actuator_Forces (oder Klaenge :-)

#ifndef _IVP_ACTIVE_VALUES_INCLUDED
#define _IVP_ACTIVE_VALUES_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

class IVP_U_Active_Value;
class IVP_U_String_Hash;
class IVP_U_Active_Float;
class IVP_U_Active_Int;
class IVP_U_Active_Float_Delayed;
class IVP_U_Active_Int_Delayed;
class IVP_U_Active_Terminal_Double;
class IVP_U_Active_Terminal_Int;
class IVP_Active_Value_Hash;
class IVP_Environment;

#define IVP_U_MOD_NAME_HASH_SIZE 16


#define IVP_ACTIVE_FLOAT_CURRENT_TIME_NAME "current_time"

/********************************************************************************
 *	Name:	       	IVP_U_Active_Value_Manager
 *	Description:	a simple name to active_value hash
 *	Attention:	will not be freed when env is deleted (to do so use IVP_U_Active_Value_Manager_EnvironmentLocal)
 ********************************************************************************/
class IVP_U_Active_Value_Manager
{
private:
    IVP_BOOL delete_on_env_delete; // see deconstructor

    IVP_Active_Value_Hash *floats_name_hash;
    IVP_Active_Value_Hash *ints_name_hash;
    IVP_U_Vector<IVP_U_Active_Float_Delayed> delayed_active_floats;
    IVP_U_Vector<IVP_U_Active_Int_Delayed> delayed_active_ints;

    IVP_U_Active_Terminal_Double  *mod_current_time;
    IVP_U_Active_Value *search_active_value;
public:
    IVP_U_Active_Value_Manager(IVP_BOOL delete_on_env_delete);
    virtual ~IVP_U_Active_Value_Manager();

    virtual void environment_will_be_deleted(IVP_Environment *){
	if (delete_on_env_delete){
	    P_DELETE_THIS(this);
	}
    };
    
    virtual void insert_active_float(IVP_U_Active_Float *mod); // add to name_hash
    virtual void remove_active_float(IVP_U_Active_Float *mod); // remove from name_hash

    virtual void insert_active_int(IVP_U_Active_Int *mod); // add to name_hash
    virtual void remove_active_int(IVP_U_Active_Int *mod); // remove from name_hash
    
    virtual void delay_active_float(IVP_U_Active_Float_Delayed *mod);  // put into delay_active_IVP_FLOAT queue, so update can be executed later (next PSI)
    virtual void delay_active_int(IVP_U_Active_Int_Delayed *mod);  // put into delay_active_IVP_FLOAT queue, so update can be executed later (next PSI)
    
    virtual void update_delayed_active_values();
    
    virtual void init_active_values_generic(); // current_time, double_null, ...
    
    virtual void refresh_psi_active_values(IVP_Environment *env);

    /********************************************************************************
     *	Name:	       	install_active_float
     *	Description:	searches for active float, returns active IVP_FLOAT if already
     *                  present, otherwise creates it
     ********************************************************************************/
    virtual IVP_U_Active_Float *install_active_float(const char *i_name, IVP_DOUBLE value);
    virtual IVP_U_Active_Terminal_Double *create_active_float(const char *i_name, IVP_DOUBLE value);
    IVP_U_Active_Float *get_active_float_by_name(const char *i_name);

    virtual IVP_U_Active_Int *install_active_int(const char *i_name, int value);
    virtual IVP_U_Active_Terminal_Int *create_active_int(const char *i_name, int value);
    IVP_U_Active_Int *get_active_int_by_name(const char *i_name);
};


///////////////

class IVP_U_Active_Float_Listener
{
    // Classes that want to be called at a active_IVP_FLOAT update
    // must be derived from IVP_U_Active_Float_Listener and must
    // implement the update method.

    // If more than one event could be possible, you can find out
    // which active_IVP_FLOAT triggered the current event by comparing with the
    // 'calling_active_float' parameter.

public:
    virtual void active_float_changed(IVP_U_Active_Float *calling_active_IVP_FLOAT) = 0;
};

///////////////

class IVP_U_Active_Int_Listener
{
    // Classes that want to be called at a active_int update
    // must be derived from IVP_U_Active_Float_Listener and must
    // implement the update method.

    // If more than one event could be possible, you can find out
    // which active_IVP_FLOAT triggered the current event by comparing with the
    // 'calling_active_float' parameter.

public:
    virtual void active_int_changed(IVP_U_Active_Int *calling_active_int) = 0;
};


class IVP_U_Active_Int_Delayed {
public:
    virtual void update_int() = 0;
};


class IVP_U_Active_Float_Delayed {
public:
    virtual void update_float() = 0;
};


///////////////
class IVP_U_Active_Value {
private:
    friend class IVP_U_Active_Value_Manager;
    char *name;
protected:
    int reference_count;
public:
    void add_reference(){ reference_count++;};
    void remove_reference(){ reference_count--;if (!reference_count) delete this; };
    virtual ~IVP_U_Active_Value();
    IVP_U_Active_Value(const char *name_);
    
    const char *get_name() { return name; };
};

class IVP_U_Active_Float: public  IVP_U_Active_Value //
{
    friend class IVP_U_Active_Value_Manager;    
private:
    IVP_U_Vector<IVP_U_Active_Float_Listener> derived_mods; // compiled dependant active_floats
    
    // dep could be removed when update_derived() doesn't find the entry in name_hash
protected:
    IVP_U_Active_Value_Manager *l_mod_manager; // backlink
public:
    int last_update;	// contains value of change_meter at last update    
    IVP_DOUBLE double_value;// for IVP_U_MOD_TYPE_DOUBLE active_floats
    void update_derived();
public:
    IVP_U_Active_Float(const char *name);
    virtual ~IVP_U_Active_Float();

    static int change_meter;
    
    IVP_DOUBLE give_double_value(){ return double_value; };
    IVP_FLOAT get_float_value(){ return (IVP_FLOAT)double_value; };

    void add_dependency(IVP_U_Active_Float_Listener *derived_active_IVP_FLOAT);
    void remove_dependency(IVP_U_Active_Float_Listener *derived_active_IVP_FLOAT);

    virtual int print() = 0;
};

///////////////

class IVP_U_Active_Int: public  IVP_U_Active_Value
{
    friend class IVP_U_Active_Value_Manager;    
private:     
    IVP_U_Vector<IVP_U_Active_Int_Listener> derived_mods; // compiled dependant active_ints
    
    // dep could be removed when update_derived() doesn't find the entry in name_hash
protected:
    IVP_U_Active_Value_Manager *l_mod_manager; // backlink
public:
    int last_update;	// contains value of change_meter at last update
    int int_value; 	// for IVP_U_MOD_TYPE_INT active_ints
    
    void update_derived();
public:
    int give_int_value(){ return int_value; };

    void add_dependency(IVP_U_Active_Int_Listener *derived_active_int);
    void remove_dependency(IVP_U_Active_Int_Listener *derived_active_int);

    virtual int print() = 0;

    IVP_U_Active_Int(const char *name);
    virtual ~IVP_U_Active_Int();
};

/**** BASIC VALUES ********************/
/**** BASIC VALUES ********************/
/**** BASIC VALUES ********************/

// basic values are the ONLY values which can be changed directly.
// whenever they are changed, the change_meter is increased.

class IVP_U_Active_Terminal_Double : public IVP_U_Active_Float, public IVP_U_Active_Float_Delayed
{
private:
    IVP_DOUBLE old_value;
public:
    IVP_U_Active_Terminal_Double(const char *name,  IVP_DOUBLE value);
    
    virtual void update_float();     // tell dependent classes that something has changed
    virtual void set_double(IVP_DOUBLE new_value, IVP_BOOL delayed=IVP_FALSE);
    int print();
};

class IVP_U_Active_Terminal_Int : public IVP_U_Active_Int, public IVP_U_Active_Int_Delayed
{
private:
    int old_value;
public:
    IVP_U_Active_Terminal_Int(const char *name,int value);
    
    virtual void update_int();	// Int::set_int
    
    virtual void set_int(int new_value, IVP_BOOL delayed=IVP_FALSE);
    int print();
};



/*** OSCILLATORS  **************/
/*** OSCILLATORS  **************/
/*** OSCILLATORS  **************/

class IVP_U_Active_Sine : public IVP_U_Active_Float, public IVP_U_Active_Float_Listener
{
private:
    IVP_U_Active_Float *time_mod; // depends on this
    IVP_DOUBLE frequence;	// Hz
    IVP_DOUBLE amplitude;
    IVP_DOUBLE null_level; // um diesen wert schwankt der sinus
    IVP_DOUBLE time_shift;
public:
    IVP_U_Active_Sine(const char *name,
	       IVP_U_Active_Float *time_mode,
	       IVP_DOUBLE freq, IVP_DOUBLE amp,
	       IVP_DOUBLE null_level,
	       IVP_DOUBLE time_shift);
    ~IVP_U_Active_Sine();
    
    void active_float_changed(IVP_U_Active_Float *calling_mod);    
    int print();
};

class IVP_U_Active_Square : public IVP_U_Active_Float, public IVP_U_Active_Float_Listener
{
private:
    IVP_U_Active_Float *time_mod; // depends on this
    IVP_DOUBLE frequence;	// Hz
    IVP_DOUBLE low_val;
    IVP_DOUBLE high_val;
public:
    IVP_U_Active_Square(const char *name,
		 IVP_U_Active_Float *time_mod,
		 IVP_DOUBLE freq,
		 IVP_DOUBLE low_val,
		 IVP_DOUBLE high_val);
    ~IVP_U_Active_Square();
    
    void active_float_changed(IVP_U_Active_Float *calling_mod);
    int print();
};


class IVP_U_Active_Pulse : public IVP_U_Active_Float, public IVP_U_Active_Float_Listener
{
private:
    IVP_U_Active_Float *time_mod; // depends on this
    IVP_DOUBLE frequence;	// Hz
    IVP_DOUBLE low_val;
    IVP_DOUBLE high_val;
    int m1; // m1 out of m2 defines interval prop.
    int m2;
public:
    IVP_U_Active_Pulse(const char *name,
		IVP_U_Active_Float *time_mod,
		IVP_DOUBLE freq,
		int m1,	// m1 out of m2 defines interval proportion
		int m2,
		IVP_DOUBLE low_val,
		IVP_DOUBLE high_val);
    ~IVP_U_Active_Pulse();
    
    void active_float_changed(IVP_U_Active_Float *calling_mod);
    int print();
};


/***** MIXER ***********************************/
/***** MIXER ***********************************/
/***** MIXER ***********************************/

class IVP_U_Active_Add : public IVP_U_Active_Float, public IVP_U_Active_Float_Listener
{
    // adds two active_floats
private:
    IVP_U_Active_Float* mod0;
    IVP_U_Active_Float* mod1;
public:
    IVP_U_Active_Add(const char *name,IVP_U_Active_Float *mod0, IVP_U_Active_Float *mod1);
    ~IVP_U_Active_Add();
    
    void active_float_changed(IVP_U_Active_Float *calling_mod);
    int print();
};


class IVP_U_Active_Sub : public IVP_U_Active_Float, public IVP_U_Active_Float_Listener
{
    // subtracts two active_floats (mod0 - mod1)
private:
    IVP_U_Active_Float* mod0;
    IVP_U_Active_Float* mod1;
public:
    IVP_U_Active_Sub(const char *name,   IVP_U_Active_Float *mod0,   IVP_U_Active_Float *mod1);
    ~IVP_U_Active_Sub();
    
    void active_float_changed(IVP_U_Active_Float *calling_mod);
    int print();
};


class IVP_U_Active_Add_Multiple : public IVP_U_Active_Float, public IVP_U_Active_Float_Listener
{
    // adds a synthesizer active_IVP_FLOAT to another, using a factor
private:
    IVP_U_Active_Float* mod0;
    IVP_U_Active_Float* mod1;
    IVP_DOUBLE factor;
    
public:
    IVP_U_Active_Add_Multiple(const char *name,
		       IVP_U_Active_Float *mod0, IVP_U_Active_Float *mod1, IVP_DOUBLE factor);
    ~IVP_U_Active_Add_Multiple();
    
    void active_float_changed(IVP_U_Active_Float *calling_mod);
    int print();
};

class IVP_U_Active_Mult : public IVP_U_Active_Float, public IVP_U_Active_Float_Listener
{
    // mults two synthesizer active_float
private:
    IVP_U_Active_Float *mod0;
    IVP_U_Active_Float *mod1;

public:
    IVP_U_Active_Mult(const char *name,IVP_U_Active_Float *mod0, IVP_U_Active_Float *mod1);
    ~IVP_U_Active_Mult();
    
    void active_float_changed(IVP_U_Active_Float *calling_mod);
    int print();
};


/**** FILTER **************************/
/**** FILTER **************************/
/**** FILTER **************************/

class IVP_U_Active_Limit : public IVP_U_Active_Float, public IVP_U_Active_Float_Listener
{
    // limits value range of a synthesizer active_float
private:
    IVP_U_Active_Float *mod;
    IVP_DOUBLE low_val;
    IVP_DOUBLE high_val;
    
public:
    IVP_U_Active_Limit(const char *name,
		IVP_U_Active_Float *mod, IVP_DOUBLE low_val, IVP_DOUBLE high_val);
    ~IVP_U_Active_Limit();

    void active_float_changed(IVP_U_Active_Float *calling_mod);
    int print();
};


/********** LOGICS ****************/
/********** LOGICS ****************/
/********** LOGICS ****************/


class IVP_U_Active_Test_Range : public IVP_U_Active_Int, public IVP_U_Active_Float_Listener
{
    // tests range of a active_float
    // in range : 1
    // otherwise: 0
    
    // returns INTEGER value
    
private:
    IVP_U_Active_Float *mod_test;
    IVP_U_Active_Float *mod_low_val;
    IVP_U_Active_Float *mod_high_val;
    
public:
    IVP_U_Active_Test_Range(const char *name,
		     IVP_U_Active_Float *mod_test, IVP_U_Active_Float *mod_low_val, IVP_U_Active_Float *mod_high_val);
    ~IVP_U_Active_Test_Range();

    void active_float_changed(IVP_U_Active_Float *calling_mod);
    int print();
};

class IVP_U_Active_Switch : public IVP_U_Active_Float, public IVP_U_Active_Float_Listener, public IVP_U_Active_Int_Listener
{
    // if mod_cond==1 returns mod_true, else mod_false 

    // returns DOUBLE value
    // depends on INT module!
    
private:
    IVP_U_Active_Int *mod_cond;
    IVP_U_Active_Float *mod_true;
    IVP_U_Active_Float *mod_false;
public:
    IVP_U_Active_Switch(const char *name,
		     IVP_U_Active_Int *mod_cond, IVP_U_Active_Float *mod_true, IVP_U_Active_Float *mod_false);
    ~IVP_U_Active_Switch();

    void active_float_changed(IVP_U_Active_Float *calling_mod);
    void active_int_changed(IVP_U_Active_Int *calling_mod);
    int print();
};

#endif
