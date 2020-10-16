// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#define abs(x) ((x) >= 0 ? (x) : -(x))
#define min(a,b) ((a) <= (b) ? (a) : (b))
#define max(a,b) ((a) >= (b) ? (a) : (b))

class IVP_Geompack {
private:
    double tolerance;
    int    ierr;
//    int	   debug_level;
    double angacc;
    double rdacc;


    int		size_intworkarray; // orig: maxiw [5000]. Should be divisible by 3 & 4!;
    int		*g_intworkarray;

    int		size_doubleworkarray; // orig: maxwk [5000]. Should be divisible by 3 & 4!;
    double	*g_doubleworkarray;

    int		size_vcl; // number of vertices (NOT bytesize of array!)
    double	*g_vcl; // 3 doubles for each vertex

    int		size_polyhedronfirstfaceoffset; // Leave this to "2" if there will never be more than one polyhedron to decompose. <orig. maxhf [200]>
    int		*g_polyhedronfirstfaceoffset; // 1 int for each polyhedron <orig. hfl>

    int		size_polyhedronfaceindices; // number of entries (NOT bytesize of array!) <orig: maxpf [2000]>
    int		*g_polyhedronfaceindices; // 2 ints for each face. <orig. pfl>

    int		size_facearrays; // <orig. maxfp [800]>
    double	*g_normals;   // 3 doubles for each face normal <orig. nrml>
    int		*g_facesdata; // 3 ints for each face <orig. facep>
    int		*g_facestype; // 1 int for each face <orig. factyp>

    int		size_facevertexarrays; // <orig. maxfv>
    int		*g_faceverticeslist; // 6 ints for each entry
    double	*g_edge_angles;      // 1 double for each entry

    int		size_ev;
    int		*g_ev;

    int		hashtable_maxsize;
    int		hashtable_size;
    int		*g_hashtable;


    int		n_original_vertices; // orig. nvc
    int		n_polyhedronfaces; // orig. npf
    int		n_work_vertices; // orig. nvert
    int		nface;
    int		npolh;

public:
    void decompose(struct geompack_parameters *params);

    void initcb_(double tolin);

    void cvdec3_();

    void cutfac_(
		 int		*p,
		 double		*nrmlc,
		 double		*dtol,
		 int		*nedgc,
		 int		*pedge,
		 int		*nce,
		 int		*cedge,
		 double		*cdang,
		 long int	*rflag
		);


    void dsphdc_();

    void edght_(
		int	*a,
		int	*b,
		int	*v,
		int	*hdfree,
		int	*last,
		int	*w
		);

    void insed3_(
		 int	*a,
		 int	*b,
		 int	*hfl
		 );

    void insfac_(
		 int	*p,
		 double *nrmlc,
		 int	*nce,
		 int	*cedge,
		 double *cdang
		 );

    void insvr3_(int a);

    int prime_(int k);

    void ptpolg_(
		 int	dim,
		 int	ldv,
		 int	*nv,
		 int	inc,
		 int	*pgind,
		 double *vcl,
		 double *pt,
		 double *nrml,
		 double *dtol,
		 int	*inout);

    void resedg_(
		 int		u,
		 long int	*rflag
		);

    int    i_sign(int a, int b);
    double d_sign(double a, double b);

    int increase_memory(void **mem_block, int *mem_size, int size_of_element);

};

struct geompack_parameters {

    float       tolin;
    float       aspc2d;
    float       atol2d;
    float       angacc;
    float       rdacc;

    int		nvc;
    int		nface;
    double *	vcl;
    int *	facep;
    int *	fvl;
    int *	nvc_out;
    int *	nface_out;
    int *	nvert_out;
    int *	npolh_out;
    double **	vcl_out;
    int **	facep_out;
    int **	fvl_out;
};

