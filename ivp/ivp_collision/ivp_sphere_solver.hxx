#ifndef IVP_SPHERE_SOLVER_INCLUDED
#define IVP_SPHERE_SOLVER_INCLUDED

class IVP_Environment;
class IVP_Real_Object;
class IVP_OV_Node;
class IVP_OV_Tree_Manager;

#ifndef IVP_SET_INCLUDED
#include <ivu_set.hxx>
#endif // IVP_SET_INCLUDED

struct IVP_Sphere_Solver_Template
{
	IVP_U_Point center;
	IVP_DOUBLE radius;
	int max_traversal_depth;
};

class IVP_Vector_of_Objects_128 : public IVP_U_Vector<IVP_Real_Object>
{
public:

    IVP_Vector_of_Objects_128() : IVP_U_Vector<IVP_Real_Object>((void**)&elem_buffer[0],128) {;};

    void reset()
	{ 
		elems = (void**)&elem_buffer[0]; 
		memsize = 128; 
	};

private:
	IVP_Real_Object* elem_buffer[128];
};

/**
 * Sphere solver. A static (i.e., callable outside simulation) query of
 * the Ipion collision detection subsystem. This class is intended to be
 * allocated on the stack, so that it will be destroyed when it goes out
 * of context. As an example:
 *
 * {
 *		IVP_Sphere_Solver_Template sst;
 *		sst.m_center.set(x, y, z);
 *		sst.radius = radius;
 *
 *		IVP_Sphere_Solver ss;
 *		ss.check_sphere_against_environment(&sst, environment);
 *		...
 * }
 *
 * ... sphere solver is destroyed ...
 *
 */
class IVP_Sphere_Solver
{
public:

	/**
	 * Constructor.
	 * Does nothing, inline.
	 */
	IVP_Sphere_Solver(void) {;};

	/**
	 * Destructor.
	 * Does nothing, inline.
	 */
	~IVP_Sphere_Solver() {;};

	/**
	 * Check the sphere against the environment and fill in the m_intruding_objects
	 * vector accordingly.
	 *
	 * @param tmpl A IVP_Sphere_Solver_Template structure to initialize the sphere solver
	 * @param env The IVP_Environment to query
	 */
	void check_sphere_against_environment(const IVP_Sphere_Solver_Template* tmpl, IVP_Environment* env);

	/**
	 * Check the sphere against a single object - boolean return value
	 *
	 * @param tmpl A IVP_Sphere_Solver_Template structure to initialize the sphere solver
	 * @param obj The object to test against
	 */
	IVP_BOOL check_sphere_against_object(const IVP_Sphere_Solver_Template* tmpl, IVP_Real_Object* obj); 

	/**
	 * Publicly readable vector of intruding objects (valid after a call to
	 * check_sphere_against_environment).
	 */
	IVP_Vector_of_Objects_128 m_intruding_objects;

	IVP_U_Point m_center;
	IVP_DOUBLE m_radius;
	int m_max_traversal_depth;

private:
	IVP_OV_Tree_Manager* m_tree_manager;

	void recursively_collect_intruding_objects(IVP_OV_Node* node, IVP_U_Float_Point* query_rlb, IVP_U_Float_Point* query_pos);
};

#endif // IVP_SPHERE_SOLVER_INCLUDED