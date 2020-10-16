// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.



#include <ivp_physics.hxx>
#if defined(LINUX)
#	include <string.h>
#endif

#include <ivu_memory.hxx>
#include <ivp_object_polygon_tetra.hxx>

#include <ivu_hash.hxx>
#include <ivp_tetra_intrude.hxx>

IVP_Tri_Edge *IVP_Tri_Edge::search_nearest_edge_to(IVP_U_Point *reference, IVP_DOUBLE *quad_dist) {
    // searches the edge around this, must be less than dist
    // 4 Physik

    IVP_Tri_Edge *fin = this->opposite;
    IVP_Tri_Edge *tmp_edge = this->opposite->next->opposite;
    IVP_Tri_Edge *shrt = this;
    IVP_DOUBLE tdist;
    while (tmp_edge != fin) {
	tdist = reference->quad_distance_to(tmp_edge->start_point);
	if (tdist < *quad_dist) {
	    *quad_dist = tdist;
	    shrt = tmp_edge;
	}
	tmp_edge = tmp_edge->next->opposite;
    }
    return shrt;
}


void IVP_Tetra_Point::print(const char *text){
    if (!text) text = "";
    printf("%s	Tetra-Point: Mask %X  Coverbits %X %X %X  pntnum=%i\n",
	   text, this->tmp_side_of_triangle_bits,
	   cover_area_bits[0],cover_area_bits[1],cover_area_bits[2],
	   opoint->point_num());
}

int IVP_Tetra_Point::p(){
    this->print(0);
    return 0;
}

void IVP_Tetra_Edge::print(const char *text){
    if (!text) text = "";
    printf("%s	Tetra_Edge: Bits %X %X %X\n",
	   text,cover_area_bits[0],cover_area_bits[1],cover_area_bits[2]);
    tetra_points[0]->print("		");
    tetra_points[1]->print("		");
}

int IVP_Tetra_Edge::p(){
    this->print(0);
    return 0;
}


void IVP_Intrusion_Included_Points::print(const char *text){
    if (!text) text = "";
    printf("%s	Intrusion_Point",text);
    tetra_point->print("	");
}

int IVP_Intrusion_Included_Points::p(){
    this->print(0);
    return 0;
}

void IVP_Intrusion_Intersection::print(const char *text){
    if (!text){
	text = "";
    }else{
	if ( long(pairing_intersection) > long(this)) return;
    }
    printf("%s %s Intersection pnt %i - %i",text,
	   (type != IVP_INTRUSION_CHECK_OVERLAP)? "normal":"equalp",
	   line_endpoints[0]->point_num(),
	   line_endpoints[1]->point_num());
    tri_edge->print("	TriEdge:");
    if (included_point){
	included_point->print("	Included Point\n");
    }
    if (pairing_intersection){
	pairing_intersection->print("Pairing Intersection:");
    }
}

int IVP_Intrusion_Intersection::p(){
    this->print(0);
    return 0;
}


void IVP_Intrusion_Status::print(const char *text){
    if (!text) text = "";
    printf("%s	Intrusion Status\n",text);
    IVP_Intrusion_Included_Points *p;
    for (p=intruded_points; p; p=p->next){
	p->print("	");
    }
    IVP_Intrusion_Intersection *s;
    for (s=intersections; s ; s=s->next){
	s->print("	");
    }
}

    
int IVP_Intrusion_Status::p(){
    this->print(0);
    return 0;
}

IVP_Intrusion_Status::IVP_Intrusion_Status(){
    P_MEM_CLEAR(this);
}

IVP_Intrusion_Status::~IVP_Intrusion_Status(){
    IVP_Intrusion_Included_Points *p,*pn;
    for (p=intruded_points; p; p=pn){
	pn = p->next;
	delete p;
    }
    IVP_Intrusion_Intersection *s,*sn;
    for (s=intersections; s ; s=sn){
	sn = s->next;
	delete s;
    }
}

void IVP_Tetra_Point::init(IVP_Tetra_Intrude *  ){
    ;
}

IVP_Tetra_Intrude::IVP_Tetra_Intrude(IVP_Tetra_Point *i_tetra_points, int orig_pnt_anz){
    P_MEM_CLEAR(this);
    twop_2_tetra_edge_hash= new IVP_Hash(orig_pnt_anz*2,
				       sizeof(IVP_Tetra_Edge *)*2,
				       (void *)P_INVALID_TETRA_EDGE);
    tetra_points = i_tetra_points;
    n_tetra_points = orig_pnt_anz;
    memsize_of_tetra_edges = n_tetra_points * 10;
    n_tetra_edges = 0;
    tetra_edges = (IVP_Tetra_Edge *)p_calloc(memsize_of_tetra_edges,sizeof(IVP_Tetra_Edge));
    {	// search min max extends of opoints
	min_koord.set(tetra_points[0].opoint);
	max_koord.set(tetra_points[0].opoint);
	int i;
	IVP_Tetra_Point *p = &tetra_points[0];
	for (i=orig_pnt_anz-1;i>=0;i--,p++){
	    min_koord.line_min(p->opoint);
	    max_koord.line_max(p->opoint);
	}
    }
    {
    	int i;
	IVP_Tetra_Point *p = &tetra_points[0];
	for (i=orig_pnt_anz-1;i>=0;i--,p++){
	    p->init(this);
	}
    }
}

IVP_Tetra_Intrude::~IVP_Tetra_Intrude(){
    delete twop_2_tetra_edge_hash;
    P_FREE(tetra_edges);
}


int IVP_Tetra_Intrude::pe(const int pnt_num){// print all edges including pnt_num
    int i;
    for (i=0;i<n_tetra_edges;i++){
	if ( pnt_num == tetra_edges[i].tetra_points[0]->opoint->point_num()){
	    tetra_edges[i].print("");
	}
 	if ( pnt_num == tetra_edges[i].tetra_points[1]->opoint->point_num()){
	    tetra_edges[i].print("");
	}
    }
    return 0;
}

inline IVP_DOUBLE p_quad_distance_between_two_straights(IVP_U_Point *linea_0,IVP_U_Point *linea_1,IVP_U_Point *lineb_0, IVP_U_Point *lineb_1){
    IVP_U_Hesse hesse;
    IVP_U_Point linea_2;linea_2.subtract(lineb_1,lineb_0);
    hesse.calc_hesse(linea_0, linea_1, &linea_2);
    hesse.normize();
    IVP_DOUBLE dist = hesse.get_dist(lineb_0);
    return IVP_Inline_Math::fabsd(dist);
}

void IVP_Tetra_Intrude::init_tetra_edge(IVP_Tetra_Edge *edge, IVP_Tetra_Point *p0, IVP_Tetra_Point *p1,IVP_Tri_Edge *e){
    P_MEM_CLEAR(edge);
    if (p0<p1){
	edge->tetra_points[0] = p0;
	edge->tetra_points[1] = p1;
    }else{
	edge->tetra_points[1] = p0;
	edge->tetra_points[0] = p1;
    }
    edge->source_tri_edge = e;

    IVP_U_Point min,max;
    min.set(p0->opoint);
    max.set(p0->opoint);
    min.line_min(p1->opoint);
    max.line_max(p1->opoint);
    this->point_2_bits(&min,&max,edge->cover_area_bits);

    for (int i=2;i>=0;i--){
	p0->cover_area_bits[i] |= edge->cover_area_bits[i];
	p1->cover_area_bits[i] |= edge->cover_area_bits[i];
    }
}

void IVP_Tetra_Intrude::point_2_bits(IVP_U_Point *lpos,IVP_U_Point *rpos,int *result_bitmasp){
    int k;
    
    for (k=0;k<3;k++){
	int bitmask = 0;
	IVP_DOUBLE inv_nen = 31.999f / (max_koord.k[k] - min_koord.k[k]);
	int l = (int)((lpos->k[k] - min_koord.k[k] - P_Pop_Eps) * inv_nen);
	int r = (int)((rpos->k[k] - min_koord.k[k] + P_Pop_Eps) * inv_nen);
	IVP_ASSERT(l<=r);
	for(;l<=r;l++){
	    bitmask |= 1<<l;
	}
	result_bitmasp[k] = bitmask;
    }
}

void IVP_Tetra_Intrude::checkin_edge(IVP_Tri_Edge *edge){
    if (edge->tmp.gen.checked_in) return; // already check in
    char *te = 0;
    IVP_Tetra_Edge *t_edge;
    IVP_Tetra_Point *buffer[2];
    if (edge->tmp.gen.tetra_point < edge->next->tmp.gen.tetra_point){
	buffer[0] = edge->tmp.gen.tetra_point;
	buffer[1] = edge->next->tmp.gen.tetra_point;
    }else{
	buffer[1] = edge->tmp.gen.tetra_point;
	buffer[0] = edge->next->tmp.gen.tetra_point;
    }

    te = (char *)twop_2_tetra_edge_hash->find((char *)&buffer[0]);

    if (te == P_INVALID_TETRA_EDGE){
	if (n_tetra_edges == memsize_of_tetra_edges){
	    memsize_of_tetra_edges = 3*memsize_of_tetra_edges/2;
	    //printf("*********** Increasing tetra_edge memory *****\n");
	    IVP_Tetra_Edge *newmem =	(IVP_Tetra_Edge *)p_calloc(
		    memsize_of_tetra_edges,sizeof(IVP_Tetra_Edge));
	    memcpy( (char *)newmem, (char *)tetra_edges, sizeof(IVP_Tetra_Edge)*n_tetra_edges);
	    delete tetra_edges;
	    tetra_edges = newmem;
	}
	t_edge = &tetra_edges[n_tetra_edges++];
	twop_2_tetra_edge_hash->add((char *)&buffer[0],
				    (void *)(((char *)t_edge)-((char *)tetra_edges)));

	this->init_tetra_edge(t_edge,edge->tmp.gen.tetra_point,edge->next->tmp.gen.tetra_point,edge);
    }else{
	t_edge = (IVP_Tetra_Edge *)(te + long(tetra_edges));
    }
    t_edge->reference_count++;
    edge->tmp.gen.checked_in = IVP_TRUE;
}

void IVP_Tetra_Intrude::checkout_edge(IVP_Tri_Edge *edge){
    if (!edge->tmp.gen.checked_in) return; // already check in
    edge->tmp.gen.checked_in = IVP_FALSE;
    char *te = 0;
    IVP_Tetra_Edge *t_edge;

    IVP_Tetra_Point *buffer[2];
    if (edge->tmp.gen.tetra_point < edge->next->tmp.gen.tetra_point){
	buffer[0] = edge->tmp.gen.tetra_point;
	buffer[1] = edge->next->tmp.gen.tetra_point;
    }else{
	buffer[1] = edge->tmp.gen.tetra_point;
	buffer[0] = edge->next->tmp.gen.tetra_point;
    }

    te = (char *)twop_2_tetra_edge_hash->find((char *)&buffer[0]);
    t_edge = (IVP_Tetra_Edge *)(te + long(tetra_edges));
    t_edge->reference_count--;
    /* edge no more referenced -> delete it */
    if (t_edge->reference_count<=0){
	twop_2_tetra_edge_hash->remove((char *)&t_edge->tetra_points[0]);
       	IVP_Tetra_Edge *last_edge = &tetra_edges[n_tetra_edges-1];
	if (last_edge != t_edge){
	    twop_2_tetra_edge_hash->remove((char *)&last_edge->tetra_points[0]);
	    twop_2_tetra_edge_hash->add((char *)&last_edge->tetra_points[0],
				    (void *)(((char *)t_edge)-((char *)tetra_edges)));
	    *t_edge = *last_edge;
	}
	n_tetra_edges --;
    }
}

IVP_INTRUSION_CHECK_RESULTS P_THREE_CHECK_INTRUDE(int me, int other0, int other1, int other2,
						       IVP_Tetra_Point *t0,IVP_Tetra_Point *t1,
						       IVP_Tri_Edge *tri_edges[4],
						       IVP_U_Hesse hesse_of_t[4],
						       IVP_U_Hesse edge_hesses[6],
						       IVP_Tri_Edge *edge_edges[6],
						       IVP_DOUBLE	pop_eps_of_t[4],
						       IVP_U_Point &intrude_position,
						       IVP_Intrusion_Status *status){
    int bor = t0->tmp_side_of_triangle_bits | t1->tmp_side_of_triangle_bits;
    int bitsand = t0->tmp_side_of_triangle_bits	& t1->tmp_side_of_triangle_bits;
    
    IVP_INTRUSION_CHECK_RESULTS type = IVP_INTRUSION_CHECK_LINE;
    while (1){
	if ( ((bitsand & 0xf & ~(bitsand >> 8)) & (1<<me)) != 0) return IVP_INTRUSION_CHECK_NONE; // both both inside and not on the surface

	if ( (bor  & (0x1111<<me)) == (0x11<<me) ){/* real intrusion, no epsilon, no identic to point ->
					     one inside, one outside*/
	    IVP_DOUBLE d0 = hesse_of_t[me].get_dist(t0->opoint);							      
	    IVP_DOUBLE d1 = hesse_of_t[me].get_dist(t1->opoint);							      
	    IVP_U_Point vec; vec.subtract(t1->opoint,t0->opoint);							      
	    intrude_position.add_multiple(t0->opoint,&vec,-d0/(d1-d0));
	    break;
	}
	

	if ( (t0->tmp_side_of_triangle_bits & ( 1 << (me+12)) ) != 0 ){	// identic to point
	    /* continue checking only if t1 in same plane */
	    if ( (t1->tmp_side_of_triangle_bits & (1 << (me +8))) == 0) return IVP_INTRUSION_CHECK_NONE;
	    intrude_position.set(t1->opoint); // take other point as intrusion point
	    type = IVP_INTRUSION_CHECK_OVERLAP;									      
	    break;
	}

	if ( (t1->tmp_side_of_triangle_bits & ( 1 << (me+12)) ) != 0 ){	// identic to point
	    /* continue checking only if in same plane */
	    if ( (t0->tmp_side_of_triangle_bits & (1 << (me +8))) == 0) return IVP_INTRUSION_CHECK_NONE;
	    intrude_position.set(t0->opoint);
	    type = IVP_INTRUSION_CHECK_OVERLAP;									      
	    break;
	}
	if ( (t0->tmp_side_of_triangle_bits & (1 << (8+me))) != 0){ // mid of a surface
	    intrude_position.set(t0->opoint);
	    break;
	}
	if ( (t1->tmp_side_of_triangle_bits & (1 << (8+me))) != 0){ // mid of a surface
	    intrude_position.set(t1->opoint);
	    break;
	}
	CORE;
    }
    /* check real point against 3 other */
    IVP_DOUBLE dist0 = hesse_of_t[other0].get_dist(&intrude_position);
    IVP_DOUBLE dist1 = hesse_of_t[other1].get_dist(&intrude_position);
    IVP_DOUBLE dist2 = hesse_of_t[other2].get_dist(&intrude_position);
    
    if (dist0 < -pop_eps_of_t[other0]) return IVP_INTRUSION_CHECK_NONE;// outside P_Pop_Eps
    if (dist1 < -pop_eps_of_t[other1]) return IVP_INTRUSION_CHECK_NONE;
    if (dist2 < -pop_eps_of_t[other2]) return IVP_INTRUSION_CHECK_NONE;

    {
	IVP_DOUBLE meps = -pop_eps_of_t[0];
	for (int i = 5;i>=0;i--){
	    IVP_DOUBLE dist = edge_hesses[i].get_dist(&intrude_position);
	    if ( dist < meps){ // now collision point is outside eps of edges
		IVP_DOUBLE d0 = edge_hesses[i].get_dist(t0->opoint); // check whether points are outside two
		IVP_DOUBLE d1 = edge_hesses[i].get_dist(t1->opoint);
		if (d0 < meps  || d1 < meps){
		    return IVP_INTRUSION_CHECK_NONE; // both points outside-> no problem		    
		}
		IVP_Tri_Edge *e = edge_edges[i]; // get the distances between the two straights
		IVP_DOUBLE eps2 = meps*meps;
		IVP_DOUBLE qd2 = p_quad_distance_between_two_straights(t0->opoint,t1->opoint, e->start_point, e->next->start_point);
		printf("qd2 = %f\n",sqrt(qd2));
		if (qd2 > eps2) {
		    return IVP_INTRUSION_CHECK_NONE; // outside edge hesses
		}
		printf("Warning: two straights are closer than eps, but no collision by normal test\n");
		t0->print("to test");
		t1->print("to test");
		e->print("triangle\n");
	    }
	}
    }
    if (dist0 < -P_RES_EPS) return IVP_INTRUSION_CHECK_EPSILON;// outside DOUBLE Eps
    if (dist1 < -P_RES_EPS) return IVP_INTRUSION_CHECK_EPSILON;
    if (dist2 < -P_RES_EPS) return IVP_INTRUSION_CHECK_EPSILON;

    if (!status) return type;
    {
	IVP_Intrusion_Intersection *is = new IVP_Intrusion_Intersection();					      
	P_MEM_CLEAR(is);											      
	is->next = status->intersections;									      
	status->intersections = is;										      
	is->type = type;											      
	is->intersect_point.set(&intrude_position);
	is->line_endpoints[0] = t0->opoint;
	is->line_endpoints[1] = t1->opoint;
	is->tri_edge = tri_edges[me];									      
    }
    return IVP_INTRUSION_CHECK_NONE;
}

inline IVP_INTRUSION_CHECK_RESULTS
IVP_Tetra_Intrude::check_point_intrusion(int bitmask[3],
				       IVP_Tetra_Point *t_points[4],
				       IVP_U_Hesse hesse_of_t[4],
				       IVP_DOUBLE pop_eps_of_t[4],
				       IVP_U_Hesse edge_hesses[6],
				       IVP_Tetra_Point *tp,
				       IVP_Intrusion_Status *status){
    tp->tmp_side_of_triangle_bits = 0;
    if ( ((bitmask[0] & tp->cover_area_bits[0]) == 0) ||
	 ((bitmask[1] & tp->cover_area_bits[1]) == 0) ||
	 ((bitmask[2] & tp->cover_area_bits[2]) == 0) ){
	return IVP_INTRUSION_CHECK_NONE;
    }

    int tpnr;
    IVP_U_Hesse *h = &hesse_of_t[3];
    IVP_DOUBLE *pop_eps = &pop_eps_of_t[3];
    for (tpnr = 0x8;tpnr>0;pop_eps--,h--,tpnr>>=1){
	IVP_DOUBLE dist = h->get_dist(tp->opoint);
	if (dist < - P_RES_EPS){	// outside DOUBLE_EPS
	    if (dist < -pop_eps[0]){	// outside Pop_Eps
		tp->tmp_side_of_triangle_bits |= tpnr << 20;
	    }
	    tp->tmp_side_of_triangle_bits |= tpnr << 4;
	    continue; // real outside (smaller epsilon for old triangles)
	}
	if (dist < P_RES_EPS){ // eps area (surface)
	    tp->tmp_side_of_triangle_bits |= tpnr << 8; // on the surface
	    if (tp == t_points[0] || tp == t_points[1] ||
		tp == t_points[2] || tp == t_points[3]){
		tp->tmp_side_of_triangle_bits |= (1<<16) | (tpnr<<12); // identic to a point
		if (tp == t_points[2] || tp == t_points[3]){
		    tp->tmp_side_of_triangle_bits |= (1<<17); // identic to new_pop_edge
		}
		continue;
	    }else{
		tp->tmp_side_of_triangle_bits |= tpnr; // partly inside
	    }
	}else{
	    tp->tmp_side_of_triangle_bits |= tpnr;// real inside
	}
    }
    if ( (tp->tmp_side_of_triangle_bits & (1 << 16)) != 0) return IVP_INTRUSION_CHECK_NONE; // is a point
    
    if ((tp->tmp_side_of_triangle_bits & (0xf << 20)) != 0){ // one side really outside
	return IVP_INTRUSION_CHECK_NONE;
    }
   {
	for (int i = 5;i>=0;i--){
	    IVP_DOUBLE dist = edge_hesses[i].get_dist(tp->opoint);
	    if ( dist < -pop_eps_of_t[0]){
		return IVP_INTRUSION_CHECK_NONE; // outside edge hesses
	    }
	}
    }    
    if ( (tp->tmp_side_of_triangle_bits & 0xf) !=  0xf ){ // not within DOUBLE EPS -> on the surface
	return IVP_INTRUSION_CHECK_EPSILON;
    }
    if (!status)	return IVP_INTRUSION_CHECK_POINT;
    
    IVP_Intrusion_Included_Points *in = new IVP_Intrusion_Included_Points;
    in->next = status->intruded_points;
    status->intruded_points = in;
    in->tetra_point = tp;
    return IVP_INTRUSION_CHECK_NONE;
}

IVP_INTRUSION_CHECK_RESULTS IVP_Tetra_Intrude::check_intrusion(IVP_Tri_Edge *old_pop_edge_a, IVP_Tri_Edge *old_pop_edge_b,
				     IVP_Tri_Edge *pop_edge_a, IVP_Tri_Edge *pop_edge_b,
				     IVP_Extra_Point *first_extra_point,
				     int	n_new_triangles,
				     IVP_Intrusion_Status *status){
    /* returns IVP_INTRUSION_CHECK_RESULTS */
    /* 0 = no intrusion */
    
    /* expects 4 edges of 4 triangles which form a tetraeder
     * the start_points of these edges are the 4 points of the tetraeder
     * first come old_triangles than new
     * n_new_triangles is the number of the new triangles */

    /* Generates an internal bit respresentation of points:
       bit 0-3 real P_RES_EPS inside or mid on a surface (not identic to point)
       bit 4-7 real P_RES_EPS outside
       bit 8-11 on the surface (or identic to point)
       bit 12-15	identic to a point
       bit 16	identic to edge_point

       bit 20-23	real P_Pop_Eps outside

       */

    IVP_Tetra_Point *t_points[4];
    IVP_U_Hesse hesse_of_t[4];
    IVP_Tri_Edge *tri_edges[4];
    IVP_DOUBLE pop_eps_of_t[4];
    IVP_U_Hesse edge_hesses[6];	// intermediate edges between edges
    IVP_Tri_Edge *edge_edges[6];

    IVP_INTRUSION_CHECK_RESULTS result = IVP_INTRUSION_CHECK_NONE;
    
    tri_edges[0] = old_pop_edge_a;
    tri_edges[1] = old_pop_edge_b;
    tri_edges[2] = pop_edge_a;
    tri_edges[3] = pop_edge_b;
    /* ****** reset  of tetra points ***/
    {
	int k;for (k=0;k<4;k++){
	    t_points[k] = tri_edges[k]->tmp.gen.tetra_point;
	    hesse_of_t[k] = tri_edges[k]->triangle->tmp.gen.hesse;
	    if (k < 4-n_new_triangles){
		pop_eps_of_t[k] = P_Pop_Eps;
	    }else{
		pop_eps_of_t[k] = P_Pop_Eps;
	    }
	}
    }
    /* ********* reset intermediate edges ********** */
    {
	int a,b,c;
	c = 0;
	for(a=0;a<4;a++){
	    for (b=a+1;b<4;b++){
		IVP_Tri_Edge *e;
		for ( e= tri_edges[a];
		      e->start_point == tri_edges[b]->start_point ||
		      e->start_point == tri_edges[b]->next->start_point ||
		      e->start_point == tri_edges[b]->prev->start_point;
		      e = e->next){;}; // search point not on both triangles
		e = e->next;
		edge_edges [c] = e;
		edge_hesses[c].set(&hesse_of_t[a]);
		edge_hesses[c].add(&hesse_of_t[b]);
		edge_hesses[c].calc_hesse_val(e->start_point);
		edge_hesses[c].normize();
		c++;
	    }
	}
	IVP_ASSERT(c==6);
    }


    IVP_U_Point lmax;
    IVP_U_Point lmin;

    /*	****** Generate Bitmask ********** */
    int bitmask[3];
    {
	int pnr;
	lmax.set(t_points[0]->opoint);
	lmin.set(t_points[0]->opoint);
	for (pnr = 1; pnr <4; pnr++){
	    lmax.line_max(t_points[pnr]->opoint);
	    lmin.line_min(t_points[pnr]->opoint);
	}
	point_2_bits(&lmin,&lmax,&bitmask[0]);
    }
    /* ***** check all points by bitmask, if not
       ok -> check hesse to 4 planes ***** */
    {
	int pnr;
	IVP_Tetra_Point *tp = tetra_points;
	for (pnr = n_tetra_points-1;pnr>=0;pnr--,tp++){
	    IVP_INTRUSION_CHECK_RESULTS s = check_point_intrusion(bitmask, t_points,hesse_of_t,pop_eps_of_t,edge_hesses, tp,status);
	    if (s>result) result = s;
	    if (result > IVP_INTRUSION_CHECK_EPSILON) return result;
	}
	{
	    IVP_Extra_Point *op;
	    for (op = first_extra_point; op; op = op->next){
		tp = op->tmp.tetra_point;
		IVP_INTRUSION_CHECK_RESULTS s = check_point_intrusion(bitmask, t_points,hesse_of_t,pop_eps_of_t,edge_hesses, tp,status);
		if (s>result) result = s;
		if (result > IVP_INTRUSION_CHECK_EPSILON) return result;
	    }
	}
    }
    /* **** check all lines, check area bitmask first, then
       tmp.tetra_point bits, then real check ****/
    {
	int tnr = n_tetra_edges;
	IVP_Tetra_Edge *te = tetra_edges;
	for (tnr--;tnr>=0;te++,tnr--){
				// check area bitmask
	    if ( ((bitmask[0] & te->cover_area_bits[0]) == 0) ||
		 ((bitmask[1] & te->cover_area_bits[1]) == 0) ||
		 ((bitmask[2] & te->cover_area_bits[2]) == 0) ){
		continue;
	    }
				// check real outside bitmask
	    IVP_Tetra_Point *t0 = te->tetra_points[0];
	    IVP_Tetra_Point *t1 = te->tetra_points[1];
	    int bitsand = t0->tmp_side_of_triangle_bits & t1->tmp_side_of_triangle_bits;
	    
	    if ( (bitsand & 0xf0)!= 0)		continue; // both points real outside	    
	    if ( (bitsand & (1<<16) ) ){
		continue;
	    }

	    // real check intrusion only to plane 2 and 3
	    {			// check 3
		IVP_U_Point intrude_position; // point of intrusion
		if (n_new_triangles>=1){
		    IVP_INTRUSION_CHECK_RESULTS s = P_THREE_CHECK_INTRUDE(3,2,1,0,t0,t1,tri_edges,hesse_of_t,edge_hesses,edge_edges,pop_eps_of_t,intrude_position,status);
		    if (s>result) result = s;
		    if (result > IVP_INTRUSION_CHECK_EPSILON) return result;
		}
		if (n_new_triangles>=2){
		    IVP_INTRUSION_CHECK_RESULTS s = P_THREE_CHECK_INTRUDE(2,3,1,0,t0,t1,tri_edges,hesse_of_t,edge_hesses,edge_edges,pop_eps_of_t,intrude_position,status);
		    if (s>result) result = s;
		    if (result > IVP_INTRUSION_CHECK_EPSILON) return result;
		}
		if (n_new_triangles>=3){
		    IVP_INTRUSION_CHECK_RESULTS s = P_THREE_CHECK_INTRUDE(1,3,2,0,t0,t1,tri_edges,hesse_of_t,edge_hesses,edge_edges, pop_eps_of_t,intrude_position,status);
		    if (s>result) result = s;
		    if (result > IVP_INTRUSION_CHECK_EPSILON) return result;
		}
	    }
	}
    }
    return result;
}


IVP_Intrusion_Status *IVP_Tetra_Intrude::calc_intrusion_status(
	IVP_Tri_Edge *old_pop_edge_a, IVP_Tri_Edge *old_pop_edge_b,
	IVP_Tri_Edge *pop_edge_a, IVP_Tri_Edge *pop_edge_b, IVP_Extra_Point *first_extra_point, int n_new_triangles){

    IVP_Intrusion_Status *status = new IVP_Intrusion_Status;
    this->check_intrusion(old_pop_edge_a,old_pop_edge_b,
			  pop_edge_a,pop_edge_b,
			  first_extra_point,n_new_triangles,status);
    return status;
}
