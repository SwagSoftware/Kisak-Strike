// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC


#include <ivp_physics.hxx>

#include <string.h>

#include <ivu_vhash.hxx>
#include <ivp_collision_filter.hxx>

IVP_Collision_Filter::~IVP_Collision_Filter() {
    return;
}

// ***************************************************************************

IVP_BOOL IVP_Collision_Filter_Coll_Group_Ident::check_objects_for_collision_detection(IVP_Real_Object *object0, IVP_Real_Object *object1) {
    char *id0 = object0->nocoll_group_ident;
    char *id1 = object1->nocoll_group_ident;
    if (id0[0] == 0 || id1[0] == 0){
	return IVP_TRUE;
    }
    
    if ( strncmp(id0, id1,IVP_NO_COLL_GROUP_STRING_LEN) == 0 ) {
	return(IVP_FALSE);
    }
    
    return(IVP_TRUE);
}

IVP_Collision_Filter_Coll_Group_Ident::~IVP_Collision_Filter_Coll_Group_Ident() {
    return;
}

IVP_Collision_Filter_Coll_Group_Ident::IVP_Collision_Filter_Coll_Group_Ident(IVP_BOOL delete_on_env_delete_in) {
    this->delete_on_env_delete = delete_on_env_delete_in;
    return;
}

void IVP_Collision_Filter_Coll_Group_Ident::environment_will_be_deleted(IVP_Environment *) {
    if ( this->delete_on_env_delete ) {
	P_DELETE_THIS(this);
    }
    return;
}

// ------------------------------------------------------------------------------


/********************************************************************************
 *	Class:	       	IVP_CFEP_Objectpair
 *	Description:	help-class for "Exclusive Pair" collision filter hash table
 ********************************************************************************/
class IVP_CFEP_Objectpair {
public:
    IVP_Real_Object *object0;
    IVP_Real_Object *object1;
};

/********************************************************************************
 *	Class:	       	IVP_CFEP_Hash
 *	Description:	hash table needed for fast storage of object pairs in
 *					"Exclusive Pair" collision filter
 ********************************************************************************/
class IVP_CFEP_Hash : protected IVP_VHash {
protected:
    IVP_BOOL compare(void *elem0, void *elem1) const;
    int pair_to_index(IVP_CFEP_Objectpair *pair);

public:
    void                 add(IVP_CFEP_Objectpair *pair);
    IVP_CFEP_Objectpair *remove(IVP_CFEP_Objectpair *pair);
    IVP_CFEP_Objectpair *find(IVP_CFEP_Objectpair *pair);

    IVP_CFEP_Hash(int create_size);
    ~IVP_CFEP_Hash();
};

IVP_BOOL IVP_CFEP_Hash::compare(void *elem0, void *elem1) const
{
    IVP_CFEP_Objectpair *pair0 = (IVP_CFEP_Objectpair *)elem0;
    IVP_CFEP_Objectpair *pair1 = (IVP_CFEP_Objectpair *)elem1;
    
    if ( pair0->object0 != pair1->object0) return(IVP_FALSE);
    if ( pair0->object1 != pair1->object1) return(IVP_FALSE);
    
    return(IVP_TRUE);
}

int IVP_CFEP_Hash::pair_to_index(IVP_CFEP_Objectpair *pair)
{
    return hash_index( (char *)pair, sizeof(pair));
};

void IVP_CFEP_Hash::add(IVP_CFEP_Objectpair *pair)
{
    add_elem(pair, pair_to_index(pair));
}

IVP_CFEP_Objectpair *IVP_CFEP_Hash::remove(IVP_CFEP_Objectpair *pair)
{
    return (IVP_CFEP_Objectpair *)remove_elem(pair, pair_to_index(pair));
}

IVP_CFEP_Objectpair *IVP_CFEP_Hash::find(IVP_CFEP_Objectpair *pair)
{
    return (IVP_CFEP_Objectpair *)find_elem(pair, pair_to_index(pair));
}

IVP_CFEP_Hash::IVP_CFEP_Hash(int create_size) : IVP_VHash(create_size)
{
    return;
}

IVP_CFEP_Hash::~IVP_CFEP_Hash()
{
    // cleanup hash content
    IVP_VHash_Enumerator<IVP_CFEP_Objectpair> enumerator(this);
    int i;
    for (i=0; i<this->n_elems(); i++) {
	delete enumerator.get_next_element(this);
    }
    return;
}

/********************************************************************************
 *	Class:	       	IVP_Collision_Filter_Exclusive_Pair
 *	Description:	"Exclusive Pair" collision filter; stores pairs of object 
 *					within a fast hash; collision detection will be disabled for 
 *					these pairs
 ********************************************************************************/
void IVP_Collision_Filter_Exclusive_Pair::generate_hash_entry(IVP_Real_Object *obj0, IVP_Real_Object *obj1, IVP_CFEP_Objectpair *entry)
{
    if ( obj1 > obj0 ) { // always sort objects by their memory address!
	entry->object0 = obj0;
	entry->object1 = obj1;
    }
    else {
	entry->object0 = obj1;
	entry->object1 = obj0;
    }

    return;
}

void IVP_Collision_Filter_Exclusive_Pair::disable_collision_between_objects(IVP_Real_Object *obj0, IVP_Real_Object *obj1)
{
    IVP_CFEP_Objectpair *new_pair = new IVP_CFEP_Objectpair();

    generate_hash_entry(obj0, obj1, new_pair);

    if ( !this->hash_table->find(new_pair) ) { // pair already stored?
	this->hash_table->add(new_pair);       // no, then store now...
    }

    return;
}

void IVP_Collision_Filter_Exclusive_Pair::enable_collision_between_objects(IVP_Real_Object *obj0, IVP_Real_Object *obj1)
{
    IVP_CFEP_Objectpair old_pair;

    generate_hash_entry(obj0, obj1, &old_pair);

    if ( this->hash_table->find(&old_pair) ) { // pair stored in hash?
	IVP_CFEP_Objectpair *removed_pair = this->hash_table->remove(&old_pair); // yes, remove...
	P_DELETE(removed_pair);
    }

    return;
}

IVP_BOOL IVP_Collision_Filter_Exclusive_Pair::check_objects_for_collision_detection(IVP_Real_Object *obj0, IVP_Real_Object *obj1)
{
    IVP_CFEP_Objectpair pair;

    generate_hash_entry(obj0, obj1, &pair);

    if ( this->hash_table->find(&pair) ) { // pair stored in hash?
	return(IVP_FALSE);                 // yes, skip collision...
    }

    return(IVP_TRUE);
}

void IVP_Collision_Filter_Exclusive_Pair::environment_will_be_deleted(IVP_Environment *)
{
    delete(this);
    return;
}

IVP_Collision_Filter_Exclusive_Pair::IVP_Collision_Filter_Exclusive_Pair()
{
    this->hash_table = new IVP_CFEP_Hash(1024);
    return;
}

IVP_Collision_Filter_Exclusive_Pair::~IVP_Collision_Filter_Exclusive_Pair()
{
    P_DELETE(this->hash_table);
    return;
}



//*******************************************************************************

void IVP_Meta_Collision_Filter::add_collision_filter(IVP_Collision_Filter *filter) {
    filter_set.add( filter );
}

void IVP_Meta_Collision_Filter::remove_collision_filter(IVP_Collision_Filter *filter) {
    filter_set.remove( filter );
}

IVP_BOOL IVP_Meta_Collision_Filter::check_objects_for_collision_detection(IVP_Real_Object *object0, IVP_Real_Object *object1) {
    int do_collision=1;

    int i;
    for(i=filter_set.len()-1;i>=0;i--) {
	IVP_Collision_Filter *coll_filter=filter_set.element_at(i);
	do_collision=do_collision & (int)coll_filter->check_objects_for_collision_detection( object0, object1 );
    }

    return (IVP_BOOL)do_collision;
}

IVP_Meta_Collision_Filter::~IVP_Meta_Collision_Filter() {
    return;
}

IVP_Meta_Collision_Filter::IVP_Meta_Collision_Filter(IVP_BOOL delete_on_env_delete_in) {
    this->delete_on_env_delete = delete_on_env_delete_in;
    return;
}

void IVP_Meta_Collision_Filter::environment_will_be_deleted(IVP_Environment *env) {
    int i;
    for(i=filter_set.len()-1;i>=0;i--) {
	IVP_Collision_Filter *coll_filter=filter_set.element_at(i);
	coll_filter->environment_will_be_deleted(env);
	filter_set.remove_at(i);
    }

    if ( this->delete_on_env_delete ) {
	P_DELETE_THIS(this);
    }
    return;
}


