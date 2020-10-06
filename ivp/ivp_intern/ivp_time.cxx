// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.



#include <ivp_physics.hxx>
#include <ivp_performancecounter.hxx>
#include <stdlib.h>

#include <ivu_min_list.hxx>

#ifndef WIN32
#   pragma implementation "ivp_time.hxx"
#   pragma implementation "ivp_time_event.hxx"
#endif
#include <ivp_time.hxx>

#if defined(WIN32) && !defined(_XBOX)
#	include "wtypes.h"
#elif defined(_XBOX)
#	ifndef WINVER
#		define WINVER 0x0500
#	endif
#	ifndef _X86_
#		define _X86_
#	endif  /* _X86_ */
#	include <excpt.h>
#	include <stdarg.h>
#	include <windef.h>
#endif


void IVP_Event_Manager_Standard::simulate_time_events(IVP_Time_Manager *tman,IVP_Environment *env,IVP_Time time) {
  IVP_FLOAT event_time;
  while ( (event_time = tman->min_hash->find_min_value()) < time - tman->base_time ){
      IVP_Time_Event *event = (IVP_Time_Event *)tman->min_hash->find_min_elem();
      tman->min_hash->remove_minlist_elem(event->index);
      event->index = IVP_U_MINLIST_UNUSED;
      IVP_ASSERT(event_time - tman->last_time >= -P_FLOAT_RES);
      tman->last_time = event_time;
      tman->env_set_current_time(env,tman->base_time + event_time);
      event->simulate_time_event(env);
      if(tman->event_manager->mode==1) {
	break;
      }
  }
  tman->env_set_current_time(env,time);
};

void IVP_Event_Manager_D::simulate_time_events(IVP_Time_Manager *tman,IVP_Environment *env,IVP_Time time) {
  IVP_FLOAT event_time;
  event_time = tman->min_hash->find_min_value();
  if( time - tman->base_time ){
    //while ( (event_time = tman->min_hash->find_min_value()) < time - tman->base_time ){
      IVP_Time_Event *event = (IVP_Time_Event *)tman->min_hash->find_min_elem();
      tman->min_hash->remove_minlist_elem(event->index);
      event->index = IVP_U_MINLIST_UNUSED;
      IVP_ASSERT(event_time - tman->last_time >= -P_FLOAT_RES);
      tman->last_time = event_time;
      tman->env_set_current_time(env,tman->base_time + event_time);
      event->simulate_time_event(env);
  }
  tman->env_set_current_time(env,time);
};

void IVP_Event_Manager::simulate_variable_time_step(IVP_Time_Manager *tman,IVP_Environment *env,IVP_Time_Event_PSI *psi_event, IVP_FLOAT delta) {

	int psi_counter = 0;	// process only one psi
	while (1){
		IVP_FLOAT event_time = tman->min_hash->find_min_value();
		IVP_Time_Event *event = (IVP_Time_Event *)tman->min_hash->find_min_elem();

		tman->env_set_current_time(env,tman->base_time + event_time);

		if ( event == psi_event){
			psi_counter ++;
			if (psi_counter == 2) {
				break;
			}
			env->set_delta_PSI_time( delta );
		}
		tman->last_time = event_time;
		tman->min_hash->remove_minlist_elem(event->index);
		event->index = IVP_U_MINLIST_UNUSED;

		event->simulate_time_event(env);
  }
};



IVP_Time_Manager::IVP_Time_Manager()
{
    P_MEM_CLEAR(this);
    event_manager=new IVP_Event_Manager_Standard();
    event_manager->mode=0;
    this->min_hash = new IVP_U_Min_List(16);
    this->psi_event = new IVP_Time_Event_PSI();

    this->insert_event(psi_event, 0.0f);
}

IVP_Time_Manager::~IVP_Time_Manager()
{
    IVP_Time_Event *event;
    while ( min_hash->has_elements()){
	event = get_next_event();
	P_DELETE(event);
    }
    P_DELETE(this->event_manager);
    P_DELETE(this->min_hash);
}

void IVP_Time_Manager::insert_event(IVP_Time_Event *event, IVP_Time time)
{
    IVP_FLOAT event_time = time - base_time;
    IVP_ASSERT(event_time - last_time >= -P_FLOAT_RES);
    event->index = this->min_hash->add((void *)event, event_time);
}


void IVP_Time_Manager::remove_event(IVP_Time_Event *event)
{
    this->min_hash->remove_minlist_elem(event->index);
}

void IVP_Time_Manager::update_event(IVP_Time_Event *event, IVP_Time time)
{
    IVP_FLOAT event_time = time - base_time;
    IVP_ASSERT(event_time - last_time >= -P_FLOAT_RES);
    this->min_hash->remove_minlist_elem(event->index);
    event->index = this->min_hash->add((void *)event, event_time);
}



IVP_Time_Event *IVP_Time_Manager::get_next_event()
{
    if (min_hash->has_elements()){
	IVP_Time_Event *event = (IVP_Time_Event *)this->min_hash->find_min_elem();
	this->min_hash->remove_minlist_elem(event->index);
	return event;
    }
    return NULL;
}

IVP_Time_Event *IVP_Time_Manager::get_next_event(IVP_Time ttime)
{
    // removes event from manager afterwards!
    IVP_DOUBLE rel_time = ttime - base_time;
    if ( min_hash->find_min_value() < rel_time ){
	IVP_Time_Event *event = (IVP_Time_Event *)this->min_hash->find_min_elem();
	this->min_hash->remove_minlist_elem(event->index);
	return event;
    }
    return NULL;
}

int IVP_Time_Manager::get_event_count()
{
    return this->min_hash->counter;
}

void IVP_Event_Manager::simulate_time_events(IVP_Time_Manager *tman,IVP_Environment *env, IVP_Time time) {
    tman=NULL;
    env=NULL;
    time=IVP_Time(0.0);
};

void IVP_Time_Manager::event_loop(IVP_Environment *env, IVP_Time time) {
  /************************************************
  * FPU mode
  ************************************************/
  //doesnt work with threads !!
#ifdef WIN32
  WORD tmpflag;
  __asm FSTCW tmpflag;

  WORD newFPUflag = tmpflag | 0x0300;
  __asm FLDCW newFPUflag;
#endif

#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
	env->get_performancecounter()->start_pcount();
#endif

  event_manager->simulate_time_events(this,env,time);

#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
	env->get_performancecounter()->stop_pcount();
#endif

#ifdef WIN32
  __asm FLDCW tmpflag;
#endif
}

void IVP_Time_Manager::simulate_variable_time_step(IVP_Environment *env, IVP_FLOAT delta_time) {
  /************************************************
  * FPU mode
  ************************************************/
  //doesnt work with threads !!
#ifdef WIN32
  WORD tmpflag;
  __asm FSTCW tmpflag;

  WORD newFPUflag = tmpflag | 0x0300;
  __asm FLDCW newFPUflag;
#endif
  if ( delta_time < IVP_MIN_DELTA_PSI_TIME ){
	  delta_time = IVP_MIN_DELTA_PSI_TIME;
  }

  if ( delta_time > IVP_MAX_DELTA_PSI_TIME ){
	  delta_time = IVP_MAX_DELTA_PSI_TIME;
  }

#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
	env->get_performancecounter()->start_pcount();
#endif

  event_manager->simulate_variable_time_step(this,env,psi_event, delta_time);
#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
	env->get_performancecounter()->stop_pcount();
#endif

#ifdef WIN32
  __asm FLDCW tmpflag;
#endif
}



void IVP_Time_Manager::env_set_current_time(IVP_Environment *env, IVP_Time time){
    env->set_current_time(time);
}

void IVP_Time_Event_PSI::simulate_time_event(IVP_Environment *env)
{
    IVP_Time_Manager *tm = env->get_time_manager();
    IVP_DOUBLE delta_psi_time = env->get_delta_PSI_time();
    env->time_of_last_psi = env->get_current_time();
    env->time_of_next_psi = env->time_of_last_psi + delta_psi_time;
    IVP_FLOAT event_time = env->time_of_last_psi.get_time();
    // reset min_list
    {
	IVP_U_Min_List_Enumerator enumerator(tm->min_hash);
	while (  IVP_U_Min_List_Element *h = enumerator.get_next_element_header()){
	    h->value -= event_time;
	}
	tm->min_hash->min_value -= event_time;
    }
    tm->base_time = env->time_of_last_psi;
    tm->last_time = 0.0f;
    env->simulate_psi(env->time_of_last_psi);
    env->time_manager->insert_event(this, env->time_of_next_psi);
}

void IVP_Time_Manager::reset_time( IVP_Time offset){
    IVP_Time_Manager *tm = this;
    IVP_FLOAT event_time = offset - base_time;

    // reset min_list
    {
	IVP_U_Min_List_Enumerator enumerator(tm->min_hash);
	while (  IVP_U_Min_List_Element *h = enumerator.get_next_element_header()){
	    h->value -= event_time;
	}
	tm->min_hash->min_value -= event_time;
    }
    tm->base_time = 0.0f;
    tm->last_time = 0.0f;
}





