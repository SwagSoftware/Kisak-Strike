// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#if defined(LINUX) || defined(SUN) || (__MWERKS__ && __POWERPC__)
#   include <alloca.h>
#endif

// IVP_EXPORT_PRIVATE

#include <ivu_float.hxx>
#include <ivp_debug_manager.hxx>

#include <ivu_hash.hxx>

#include <ivp_mindist_intern.hxx>
#include <ivp_mindist_minimize.hxx>

#include <ivp_debug.hxx>

#include <ivp_anomaly_manager.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_cache_ledge_point.hxx>
#include <ivp_compact_ledge_solver.hxx>
#include <tier0/dbg.h>


// for english speakers:
// the collision files sometimes make use of abbreviations
// that have got german roots:

// P means Point or Vertex
// F means Face
// K means Edge (german: Kante)
// B means Ball


struct IVP_Loop_Key_Struct
{
    IVP_SYNAPSE_POLYGON_STATUS s0;
    IVP_SYNAPSE_POLYGON_STATUS s1;
    const IVP_Compact_Edge *e0;
    const IVP_Compact_Edge *e1;
};

#if !defined(IVP_LOOP_LIST_SIZE)
IVP_BOOL IVP_Mindist_Minimize_Solver::check_loop_hash(IVP_SYNAPSE_POLYGON_STATUS i_s0,
				      const IVP_Compact_Edge *i_e0,
				      IVP_SYNAPSE_POLYGON_STATUS i_s1,
				      const IVP_Compact_Edge *i_e1)
{
    if (! loop_hash ){
#ifdef	IVP_MINDIST_BEHAVIOUR_DEBUG
	printf("start loop check (max passes exceeded).\n");
#endif	
	this->init_loop_hash();
    }
    IVP_Loop_Key_Struct key_struct;

    if ( i_e0 < i_e1){
	key_struct.s0 = i_s0;
	key_struct.s1 = i_s1;
	key_struct.e0 = i_e0;
	key_struct.e1 = i_e1;
    }else{
	key_struct.s0 = i_s1;
	key_struct.s1 = i_s0;
	key_struct.e0 = i_e1;
	key_struct.e1 = i_e0;
    }
    void *found = loop_hash->find((char *)&key_struct);
    if(found){
	// we had this situation before
	return IVP_TRUE;
    }
    loop_hash->add((char *)&key_struct, (void *)1);
    return IVP_FALSE;
}


void IVP_Mindist_Minimize_Solver::init_loop_hash()
{    
    P_DELETE(loop_hash);
    loop_hash = new IVP_Hash(IVP_LOOP_HASH_SIZE/*size*/, sizeof(IVP_Loop_Key_Struct) );
}
#else
void IVP_Mindist_Minimize_Solver::init_loop_hash(){;};

IVP_BOOL IVP_Mindist_Minimize_Solver::check_loop_hash(IVP_SYNAPSE_POLYGON_STATUS i_s0,     const IVP_Compact_Edge *i_e0,
						      IVP_SYNAPSE_POLYGON_STATUS i_s1,     const IVP_Compact_Edge *i_e1){
    IVP_ASSERT( i_s0 < 4);
    IVP_ASSERT( i_s1 < 4);

    //lwss -x64 fixes
    //int x0 = int(i_e0) | i_s0;
    //int x1 = int(i_e1) | i_s1;
    intptr_t x0 = intptr_t (i_e0) | i_s0;
    intptr_t x1 = intptr_t (i_e1) | i_s1;
    //lwss end

    if (x0 < x1) {
	int h = x0; x0 = x1; x1 = h;
    }

    IVP_MM_Loop_Hash_Struct *s = & loop_hash[loop_hash_size-1];
    for(int i = loop_hash_size-1; i>=0; i--){
	if ( s->a == x0 && s->b == x1){
	    return IVP_TRUE;
	}
	s--;
    }
    if ( loop_hash_size >= IVP_LOOP_LIST_SIZE){
	return IVP_TRUE;
    }
    loop_hash[ loop_hash_size ].a = x0;
    loop_hash[ loop_hash_size ].b = x1;
    loop_hash_size++;
    return IVP_FALSE;
}
#endif

///////////////////////////////////////////
///////////////////////////////////////////
void IVP_Mindist::mindist_rescue_push(){
    // This method is called whenever the simualtion finds interpenetrating
    // moveable objects.

    // The mass centers of the objects are then pushed apart - to
    // hopefully separate them und thus bring them into a sane state.
    
    IVP_Real_Object *obj0 = get_synapse(0)->get_object();
    IVP_Real_Object *obj1 = get_synapse(1)->get_object();

    obj0->get_environment()->get_anomaly_manager()->inter_penetration( this, obj0, obj1 );
}


void IVP_Mindist_Minimize_Solver::pierce_mindist(){

    // Find out the best place to continue on the
    // opposite side of the object
    
    // calc world coords of syn1
    IVP_Synapse_Real *syn_pierce;
    IVP_Synapse_Real    *syn_other = mindist->get_sorted_synapse(0);
    if (syn_other->get_status() != IVP_ST_BACKSIDE){
	syn_pierce = mindist->get_sorted_synapse(1);
    }else{ //@@@ This actually should not happen
	//CORE;
	syn_pierce = syn_other;
	syn_other = mindist->get_sorted_synapse(1);
    }
    IVP_ASSERT(syn_pierce->get_status() == IVP_ST_BACKSIDE);
    
    IVP_U_Point &syn_other_Fos = pos_opposite_BacksideOs;

    // syn0
    const IVP_Compact_Edge *F = syn_pierce->edge;
    const IVP_Compact_Edge *pierced_tri;

    pierced_tri = IVP_CLS.minimize_on_other_side(F, &syn_other_Fos);

    // minimize again with this new start situation
    syn_pierce->update_synapse(pierced_tri, IVP_ST_TRIANGLE);
}
////////////////////////////////////////
////////////////////////////////////////



/** Recalculation of mindist,
  requires:	calc_next_PSI_matrix called
  result:	updates synapses
  */
IVP_MRC_TYPE IVP_Mindist::recalc_invalid_mindist()
{
    IVP_Time_CODE tc = this->get_environment()->get_current_time_code();
    if(recalc_time_stamp==tc) {
	return IVP_MRC_ALREADY_CALCULATED;
    }
    recalc_time_stamp=tc;

    IVP_Mindist_Minimize_Solver mms(this);
    mms.P_Finish_Counter = 0;   // create hash instantly
    
    while(1){
	IVP_MRC_TYPE res = mms.recalc_mindist_sub();
	if (res == IVP_MRC_OK){  // fast return if ok
	  recalc_result = IVP_MDRR_OK;
	  return res;
	}
	recalc_result = IVP_MDRR_INTRUSION;
	
	switch(res){
	case IVP_MRC_BACKSIDE:{
	    // find best triangle on opposite of convex object
	    mms.pierce_mindist();
	    // case ENDLESS must follow!
	}
	case IVP_MRC_ENDLESS_LOOP:{
	  if (this->mindist_function == IVP_MF_PHANTOM ||
	      this->mindist_status == IVP_MD_HULL_RECURSIVE ){
	    return res;
	  }
	  // MINDIST RESCUE PUSH
	  mindist_rescue_push();
	  return res;
	}
	default:
	    CORE;
	    break;
	}
	CORE;	
    }	// while
    CORE;

    // will not be reached, set for compiler warning.
    return IVP_MRC_ILLEGAL;
}


IVP_MRC_TYPE IVP_Mindist::recalc_mindist()
{
    IVP_Time_CODE tc = this->get_environment()->get_current_time_code();
    if(recalc_time_stamp==tc) {
	return IVP_MRC_ALREADY_CALCULATED;
    }

    recalc_time_stamp=tc;

    IVP_Mindist_Minimize_Solver mms(this);
    int pierce_counter = 0;
    while(1){
	IVP_MRC_TYPE res = mms.recalc_mindist_sub();
	if (res == IVP_MRC_OK){  // fast return if ok
	  recalc_result = IVP_MDRR_OK;
	  return res;
	}
	recalc_result = IVP_MDRR_INTRUSION;
	
	switch(res){
	case IVP_MRC_BACKSIDE:{
#ifdef IVP_MINDIST_BEHAVIOUR_DEBUG	    
	    IVP_ASSERT(this->detect_collision(synapse[0]->to_poly()->get_ivp_polygon()->tetras,
					    synapse[1]->to_poly()->get_ivp_polygon()->tetras) == IVP_FALSE);
#endif	    
	    // find best triangle on opposite of convex object
	    mms.pierce_mindist();	    
	    if (++pierce_counter < IVP_MAX_PIERCINGS){
	      continue;
	    }
	    // case ENDLESS must follow!
	}
	case IVP_MRC_ENDLESS_LOOP:{
	  if (this->mindist_function == IVP_MF_PHANTOM ||
	      this->mindist_status == IVP_MD_HULL_RECURSIVE ){
	    return res;
	  }
	  IVP_IF(1){
		const char *name0 = get_synapse(0)->get_object()->get_name();
		if (!name0) name0 = "(null)";
		const char *name1 = get_synapse(1)->get_object()->get_name();
		if (!name1) name1 = "(null)";
		printf("recalc_mindist: Endless Loop without collision or termination problem.%s %s\n",
		       name0,name1);
	    }
	    // MINDIST RESCUE PUSH
	    mindist_rescue_push();

#ifdef IVP_MINDIST_BEHAVIOUR_DEBUG	    
	    if(detect_collision(psyn_0->get_ivp_polygon()->tetras,
				psyn_1->get_ivp_polygon()->tetras)){
		CORE;
	    }else{
		IVP_IF(1){ mms.termination_len = P_DOUBLE_MAX; }
		P_Finish_Counter = 10; // debug purposes
		IVP_IF(1) {
		    printf("recalc_mindist : Endless Loop without collision or termination problem.\n");
		}
		continue; // helps debugging
//		CORE;
		return res;
	    }
#endif
	    return res;
	}
	default:
	    CORE;
	    break;
	}
	CORE;
    }	// while
    CORE;
    // will not be reached, set for compiler warning.
    return IVP_MRC_ILLEGAL;
}

IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::p_minimize_FF( const IVP_Compact_Edge *A,const IVP_Compact_Edge *B, IVP_Cache_Ledge_Point *m_cache_A, IVP_Cache_Ledge_Point *m_cache_B)
{

    // Case: Surface - Surface
    
    IVP_DOUBLE min_qdist = P_DOUBLE_MAX;
    //lwss add - this variable is for the Error() string. From retail, the purpose seems to be to know which loop ran last.
    int dist = -1;
    //lwss end
    // check all 9 point point combinations first:
    {
	const IVP_Compact_Edge *pA, *pB;
	int a, b;
	for (pA=A, a=0; a<3; pA=pA->get_next(),a++){
	    for (pB=B,b=0; b<3; pB=pB->get_next(), b++){
		IVP_U_Point pos_B_ws; IVP_CLS.give_world_coords_AT(pB, m_cache_B,&pos_B_ws);
		IVP_U_Point pos_A_ws; IVP_CLS.give_world_coords_AT(pA, m_cache_A,&pos_A_ws);
		IVP_DOUBLE qdist = pos_A_ws.quad_distance_to(&pos_B_ws);
		    ;
		if (qdist < min_qdist){
		    m_cache_A->tmp.synapse->update_synapse(pA, IVP_ST_POINT);
		    m_cache_B->tmp.synapse->update_synapse(pB, IVP_ST_POINT);
		    min_qdist = qdist;
		    dist = 0; //lwss add
		}
	    }
	}
    }
    // check all 6 point area combinations
    IVP_Cache_Ledge_Point *cc_A[2]; cc_A[0] = m_cache_A; cc_A[1] = m_cache_B;
    IVP_Cache_Ledge_Point *cc_B[2]; cc_B[0] = m_cache_B; cc_B[1] = m_cache_A;
    const IVP_Compact_Edge *tabA[2]; tabA[0] = A; tabA[1] = B;
    const IVP_Compact_Edge *tabB[2]; tabB[0] = B; tabB[1] = A;
    {
	for (int i=0;i<2;i++){
	    const IVP_Compact_Edge *pA;
	    const IVP_Compact_Edge *pB= tabB[i];
	    int a;
	    IVP_U_Hesse hesse_B;
	    IVP_CLS.calc_hesse_object(pB, cc_B[i]->get_compact_ledge(), &hesse_B);

	    for ( pA=tabA[i],a=0; a<3; pA=pA->get_next(),a++){
	      IVP_U_Point pA_Bos;
	      IVP_CLS.calc_pos_other_space( pA, cc_A[i], cc_B[i], &pA_Bos );
	      IVP_Unscaled_QR_Result qr;
	      IVP_CLS.calc_unscaled_qr_vals_F_space(cc_B[i]->get_compact_ledge(), pB, &pA_Bos, &qr);
	      if ( !qr.is_outside()){
		IVP_DOUBLE qdist = hesse_B.get_dist( &pA_Bos);
		qdist *= qdist;
		if ( qdist * (1+P_DOUBLE_RES) < min_qdist){
		  cc_A[i]->tmp.synapse->update_synapse(pA, IVP_ST_POINT);
		  cc_B[i]->tmp.synapse->update_synapse(pB, IVP_ST_TRIANGLE);
		  min_qdist = qdist;
          dist = 1; //lwss add
		}
	      }
	    }
	}
    }

    // check all 18 point edge combinations
    {
	int i;
	for (i=0; i<2; i++){
	    const IVP_Compact_Edge *pA, *pB;
	    int a, b;
	    for (pB=tabB[i],b=0; b<3; pB=pB->get_next(),b++){
	      IVP_U_Point pB_Aos;
	      IVP_CLS.calc_pos_other_space( pB, cc_B[i], cc_A[i], &pB_Aos );
	      
	      for (pA=tabA[i],a=0; a<3; pA=pA->get_next(),a++){
		IVP_Unscaled_S_Result sr;
		IVP_CLS.calc_unscaled_s_val_K_space(  cc_A[i]->get_compact_ledge(), pA, &pB_Aos, &sr);
		if (sr.is_outside()) continue;
		IVP_DOUBLE qdist = IVP_CLS.quad_dist_edge_to_point_K_space(cc_A[i]->get_compact_ledge(), pA, &pB_Aos );
		if ( qdist * (1+P_DOUBLE_RES) < min_qdist){
		  cc_A[i]->tmp.synapse->update_synapse(pA, IVP_ST_EDGE);
		  cc_B[i]->tmp.synapse->update_synapse(pB, IVP_ST_POINT);
		  min_qdist = qdist;
          dist = 2; //lwss add
		}
	      }
	    }
	}
    }
    {
	// check all 9 edge edge combinations
	const IVP_Compact_Edge *pA, *pB;
	int a, b;
	for (pA=A,a=0; a<3; pA=pA->get_next(),a++){
	    for (pB=B,b=0; b<3; pB=pB->get_next(),b++){
	      IVP_KK_Input kkin( pA, pB, m_cache_A, m_cache_B);
	      IVP_Unscaled_KK_Result kkr;
	      IVP_CLS.calc_unscaled_KK_vals(kkin, &kkr);
	      if (kkr.is_outside_K()) continue;
	      if (kkr.is_outside_L()) continue;
	      
	      IVP_DOUBLE qdist = kkin.calc_quad_distance_edge_edge();
		if (qdist * (1+P_DOUBLE_RES) < min_qdist){
		    m_cache_A->tmp.synapse->update_synapse(pA, IVP_ST_EDGE);
		    m_cache_B->tmp.synapse->update_synapse(pB, IVP_ST_EDGE);
		    min_qdist = qdist;
            dist = 3; //lwss add
		}
	    }
	}
    }



    IVP_Synapse_Real *syn0, *syn1, *syn_h;
    syn0 = m_cache_A->tmp.synapse;
    syn1 = m_cache_B->tmp.synapse;
    IVP_Cache_Ledge_Point *m_cache_0, *m_cache_1;

    m_cache_0 = m_cache_A;
    m_cache_1 = m_cache_B;

    
    if((syn1->get_status() == IVP_ST_POINT) && (syn0->get_status() != IVP_ST_POINT)){
	// @@@ one might drop the second && case
	syn_h=syn0; syn0=syn1; syn1 = syn_h;
	m_cache_0 = m_cache_B;
	m_cache_1 = m_cache_A;
    }
    
    // like recalc_mindist sub
    sort_synapses(m_cache_0->tmp.synapse,m_cache_1->tmp.synapse);

    const IVP_Compact_Edge *e0 = syn0->edge;
    const IVP_Compact_Edge *e1 = syn1->edge;

    IVP_ASSERT(m_cache_0->tmp.synapse == syn0);
    IVP_ASSERT(m_cache_1->tmp.synapse == syn1);

    IVP_MRC_TYPE ret_val = IVP_MRC_UNINITIALIZED; // (un)initialize

    switch(syn0->get_status()){
      case IVP_ST_POINT:{
	  switch(syn1->get_status()){  
	    case IVP_ST_POINT:{
		ret_val = p_minimize_PP(e0, e1, m_cache_0, m_cache_1);
		break;
	    }
	    case IVP_ST_EDGE:{
		ret_val =  p_minimize_PK(e0, e1, m_cache_0, m_cache_1);
		break;
	    }
	    case IVP_ST_TRIANGLE:{
	      ret_val = p_minimize_PF(e0, e1, m_cache_0, m_cache_1);
	      break;
	    }
	    default:
        //lwss change CORE->error statement in retail version. Seems Valve had trouble here as well at some point.
        //CORE;
        Error("%s in contact with %s, crash. dist = %d, minq = %1f\n", syn0->get_object()->get_name(), syn1->get_object()->get_name(), dist, min_qdist );
	  }
	  break;
      };
      case IVP_ST_EDGE:{
	  switch(syn1->get_status()){  
	    case IVP_ST_EDGE:{
		ret_val = p_minimize_KK(e0, e1, m_cache_0, m_cache_1);
		break;
	    }
	  default:
	      //lwss change CORE->error statement in retail version. Seems Valve had trouble here as well at some point.
	      //CORE;
	      Error("%s in contact with %s, crash. dist = %d, minq = %1f\n", syn0->get_object()->get_name(), syn1->get_object()->get_name(), dist, min_qdist );
	      break;
	  }
	  break;
      }
      default:
    //lwss change CORE->error statement in retail version. Seems Valve had trouble here as well at some point.
    //CORE;
    Error("%s in contact with %s, crash. dist = %d, minq = %1f\n", syn0->get_object()->get_name(), syn1->get_object()->get_name(), dist, min_qdist );
    }
    
    return ret_val;
}


IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::minimize_default_poly_poly(IVP_Mindist_Minimize_Solver *mms)
{
    IVP_Synapse_Real *syn0, *syn1;
    
    syn0 = mms->mindist->get_sorted_synapse(0);
    syn1 = mms->mindist->get_sorted_synapse(1);

    // get caches
    IVP_Polygon *poly_0 = syn0->get_object()->to_poly();
    IVP_Polygon *poly_1 = syn1->get_object()->to_poly();
    
    
    const IVP_Compact_Edge *e0 = syn0->edge;
    const IVP_Compact_Edge *e1 = syn1->edge;
    
    IVP_Cache_Ledge_Point m_cache_0(poly_0,e0->get_compact_ledge());
    IVP_Cache_Ledge_Point m_cache_1(poly_1,e1->get_compact_ledge());

    
    m_cache_0.tmp.synapse = syn0;	// remember order of synapses
    m_cache_1.tmp.synapse = syn1;

    IVP_MRC_TYPE ret_val = IVP_MRC_UNINITIALIZED; // (un)initialize

    IVP_IF(ivp_check_debug_mindist(mms->mindist)){
	printf("%32s statii: %i:%i \n","minimize_default_poly_poly", syn0->get_status(), syn1->get_status());
    }

#define SYN_COMBINE(a,b) (a * IVP_ST_MAX_LEGAL + b)  //IVP_ST_MAX_LEGAL = 4
    switch( SYN_COMBINE(syn0->get_status(), syn1->get_status())){
      case SYN_COMBINE(IVP_ST_POINT,IVP_ST_POINT):{
		ret_val = mms->p_minimize_PP(e0, e1, &m_cache_0, &m_cache_1);
		break;
      }
      case SYN_COMBINE(IVP_ST_POINT,IVP_ST_EDGE):{
		ret_val = mms->p_minimize_PK(e0, e1, &m_cache_0, &m_cache_1);
		break;
	    }
      case SYN_COMBINE(IVP_ST_POINT,IVP_ST_TRIANGLE):{
	      ret_val = mms->p_minimize_PF(e0, e1, &m_cache_0, &m_cache_1);
	      break;
	    }
      case SYN_COMBINE(IVP_ST_EDGE,IVP_ST_EDGE):{
		ret_val = mms->p_minimize_KK(e0, e1, &m_cache_0, &m_cache_1);
		break;
	    }
      default:{
	ret_val = mms->p_minimize_FF(e0, e1, &m_cache_0, &m_cache_1);
      }
    }
    
    m_cache_0.remove_reference();
    m_cache_1.remove_reference();

    //mms->proove_polypoly();

    IVP_IF(ivp_check_debug_mindist(mms->mindist)){
      	printf("%32s statii: %i:%i len %f\n", " '' ", syn0->get_status(), syn1->get_status(), mms->mindist->get_length());
    }

    return ret_val;
}

IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::minimize_PB(IVP_Mindist_Minimize_Solver *mms){
    CORE;
    mms->swap_synapses();
    return minimize_B_POLY(mms);
}

IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::minimize_B_POLY(IVP_Mindist_Minimize_Solver *mms){
    IVP_Synapse_Real *syn_B = mms->mindist->get_synapse(0);
    IVP_Ball *ball = syn_B->get_object()->to_ball();
    IVP_Cache_Ball	m_cache_B;
    m_cache_B.object = ball;
    m_cache_B.cache_object = ball->get_cache_object();
    m_cache_B.tmp.synapse = syn_B;	// remember order of synapses

    IVP_Synapse_Real *syn_P = mms->mindist->get_synapse(1);
    IVP_Polygon *poly_P = syn_P->get_object()->to_poly();
    const IVP_Compact_Edge *P = syn_P->edge;
    
    IVP_Cache_Ledge_Point m_cache_P(poly_P,P->get_compact_ledge());
    m_cache_P.tmp.synapse = syn_P;

    IVP_IF(ivp_check_debug_mindist(mms->mindist)){
	printf("%32s statii: %i \n","minimize_default_ball_poly", syn_P->get_status());
    }

    IVP_MRC_TYPE ret_val;
    switch(syn_P->get_status()){  
    case IVP_ST_POINT:
	ret_val = mms->p_minimize_BP( &m_cache_B, P, &m_cache_P );
	break;
	
    case IVP_ST_EDGE:
	ret_val = mms->p_minimize_BK( &m_cache_B, P, &m_cache_P );
	break;
    case IVP_ST_TRIANGLE:
	ret_val = mms->p_minimize_BF( &m_cache_B, P, &m_cache_P );
	break;
    default:
      ret_val = IVP_MRC_UNINITIALIZED;
	CORE;
    };
    m_cache_P.remove_reference();
    m_cache_B.cache_object->remove_reference();
    //   mms->proove_ballpoly();
    return ret_val;
}


IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::minimize_KB(IVP_Mindist_Minimize_Solver *mms){
    CORE;
    mms->swap_synapses();
    return minimize_B_POLY(mms);
}

IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::minimize_FB(IVP_Mindist_Minimize_Solver *mms){
    CORE;
    mms->swap_synapses();
    return minimize_B_POLY(mms);
}

IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::minimize_swapped_poly_poly(IVP_Mindist_Minimize_Solver *mms){
    mms->swap_synapses();
    return minimize_default_poly_poly(mms);
}


IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::minimize_BB(IVP_Mindist_Minimize_Solver *mms){
    IVP_Synapse_Real *syn_A = mms->mindist->get_synapse(0);
    IVP_Ball *ball_A = syn_A->get_object()->to_ball();
    IVP_Cache_Object *m_cache_A;
    m_cache_A = ball_A->get_cache_object();

    IVP_Synapse_Real *syn_B = mms->mindist->get_synapse(1);
    IVP_Ball *ball_B = syn_B->get_object()->to_ball();
    IVP_Cache_Object *m_cache_B;
    m_cache_B = ball_B->get_cache_object();

    IVP_U_Point *A = m_cache_A->m_world_f_object.get_position();
    IVP_U_Point *B = m_cache_B->m_world_f_object.get_position();

    mms->mindist->contact_plane.subtract(A,B);
    IVP_DOUBLE qlen = mms->mindist->contact_plane.quad_length();
    IVP_DOUBLE inv_len;
	if( IVP_Inline_Math::fabsd(qlen)>P_DOUBLE_EPS ) { // used to be fabs, which was a sml call
        inv_len = IVP_Fast_Math::isqrt(qlen,3);
	} else {
		inv_len=1.0f;
	}
    mms->mindist->len_numerator = qlen * inv_len - mms->mindist->sum_extra_radius;
    mms->mindist->contact_plane.mult(inv_len);

#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED	
    IVP_U_Float_Point diff_center;
    diff_center.subtract( &m_cache_A->core_pos, &m_cache_B->core_pos );
    mms->mindist->contact_dot_diff_center = diff_center.dot_product(&mms->mindist->contact_plane);
#endif

    m_cache_A->remove_reference();
    m_cache_B->remove_reference();
    return IVP_MRC_OK;
}


///////////////////////////////////////////
// Pierce through convex object and
// traverse triangles until valid q/r values (or valid term.len?).
// Checks for valid termination_len.
// Returned edge represents the triangle found.
///////////////////////////////////////////

const IVP_Compact_Edge *IVP_Compact_Ledge_Solver::minimize_on_other_side(
    const IVP_Compact_Edge *edge,    const IVP_U_Point *partner_os)
{    // manage pierce count
    const IVP_Compact_Ledge *c_ledge = edge->get_compact_ledge();

    int n_triangles = c_ledge->get_n_triangles();
#if defined(IVP_NO_ALLOCA)
    uchar pierce_visited_array[IVP_MAX_TRIANGLES_PER_LEDGE];
#else
    uchar *pierce_visited_array = (uchar *)alloca(n_triangles);
#endif
    
    memset(pierce_visited_array,0,n_triangles);        // @@@OG could be optimized, depending on how often piercing takes place
    

    int pierce_idx = edge->get_triangle()->get_pierce_index();
    const IVP_Compact_Triangle *pierced_tri = &c_ledge->get_first_triangle()[pierce_idx];
    
    const IVP_Compact_Edge *F = pierced_tri->get_first_edge(); // precalculated piercing

    while(1){
	pierce_visited_array[F->get_triangle()->get_tri_index()] = 1; // tag triangle as visited
	
	IVP_Unscaled_QR_Result qr;
	IVP_CLS.calc_unscaled_qr_vals_F_space(c_ledge, F, partner_os, &qr);

	// search best edge and take corresponding neighbor triangle
	const IVP_Compact_Edge *e;
	int j;
	int moved = 0;
	for (e=F,j=0;j<3;e=e->get_next(),j++){
	    if (qr.checks[j] > 0.0f ) continue;  // inside triangle
	    int tri_idx = e->get_opposite()->get_triangle()->get_tri_index();
	    if(pierce_visited_array[tri_idx]) continue; // already visited

	    F = e->get_opposite();
	    moved = 1;
	    break;
	}
	if(!moved){
	    // no movement possible anymore, take this triangle as final triangle
	    break;
	}
    }
    return F;
}



IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::minimize_illegal(IVP_Mindist_Minimize_Solver *){
    CORE;
    return IVP_MRC_ILLEGAL;
}

IVP_MRC_TYPE (*IVP_Mindist_Minimize_Solver::mms_function_table[IVP_ST_MAX_LEGAL][IVP_ST_MAX_LEGAL])(IVP_Mindist_Minimize_Solver *mms);

void IVP_Mindist_Minimize_Solver::init_mms_function_table(){
    for (int i=0; i< IVP_ST_MAX_LEGAL; i++){
	for (int j=0; j< IVP_ST_MAX_LEGAL; j++){
	    mms_function_table[i][j] = minimize_illegal;
	}
    }
    mms_function_table[IVP_ST_POINT] [IVP_ST_POINT]    = minimize_default_poly_poly;
    mms_function_table[IVP_ST_POINT] [IVP_ST_EDGE]     = minimize_default_poly_poly;
    mms_function_table[IVP_ST_POINT] [IVP_ST_TRIANGLE] = minimize_default_poly_poly;
    mms_function_table[IVP_ST_POINT] [IVP_ST_BALL]     = minimize_PB;

    mms_function_table[IVP_ST_EDGE] [IVP_ST_POINT]    = minimize_swapped_poly_poly;
    mms_function_table[IVP_ST_EDGE] [IVP_ST_EDGE]     = minimize_default_poly_poly;
    mms_function_table[IVP_ST_EDGE] [IVP_ST_TRIANGLE] = minimize_illegal;
    mms_function_table[IVP_ST_EDGE] [IVP_ST_BALL]     = minimize_KB;

    mms_function_table[IVP_ST_TRIANGLE] [IVP_ST_POINT]    = minimize_swapped_poly_poly;
    mms_function_table[IVP_ST_TRIANGLE] [IVP_ST_EDGE]     = minimize_illegal;
    mms_function_table[IVP_ST_TRIANGLE] [IVP_ST_TRIANGLE] = minimize_default_poly_poly;
    mms_function_table[IVP_ST_TRIANGLE] [IVP_ST_BALL]     = minimize_FB;

    mms_function_table[IVP_ST_BALL] [IVP_ST_POINT]    = minimize_B_POLY;
    mms_function_table[IVP_ST_BALL] [IVP_ST_EDGE]     = minimize_B_POLY;
    mms_function_table[IVP_ST_BALL] [IVP_ST_TRIANGLE] = minimize_B_POLY;
    mms_function_table[IVP_ST_BALL] [IVP_ST_BALL]     = minimize_BB;
}

