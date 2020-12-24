#if 0

#ifndef PHYSICS_SOFTBODY_H
#define PHYSICS_SOFTBODY_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

#include "vphysics_bullet/softbodyV32.h"

// Purpose: Dynamically deforming meshes (blankets, dents in objects, etc.)

// Class declarations
class CPhysicsEnvironment;
class CPhysicsObject;

class btSoftBody;

class CPhysicsSoftBody : public IPhysicsSoftBody {
	public:
		CPhysicsSoftBody();
		~CPhysicsSoftBody();

		bool			IsAsleep() const;

		void			SetTotalMass(float fMass, bool bFromFaces = false);
		void			Anchor(int node, IPhysicsObject *pObj);

		int				GetNodeCount() const;
		int				GetFaceCount() const;
		int				GetLinkCount() const;

		softbodynode_t	GetNode(int i) const;
		softbodylink_t	GetLink(int i) const;
		softbodyface_t	GetFace(int i) const;

		void			SetNode(int i, softbodynode_t &node);

		void			AddNode(const Vector &pos, float mass); // Appends a new node to the end of the list (size-1)
		void			AddLink(int node1, int node2, bool bCheckExist = false);

		void			RemoveNode(int i);
		void			RemoveLink(int i);
		void			RemoveFace(int i);

		// Get soft body AABB (cannot be implemented in collision interface because soft bodies change shape)
		void			GetAABB(Vector *mins, Vector *maxs) const;
		void			RayTest(Ray_t &ray, trace_t *pTrace) const;
		void			BoxTest(Ray_t &ray, trace_t *pTrace) const;

		void			Transform(const matrix3x4_t &mat);
		void			Transform(const Vector *vec, const QAngle *ang);
		void			Scale(const Vector &scale);

		IPhysicsEnvironment32 *GetPhysicsEnvironment() const { return (IPhysicsEnvironment32 *)m_pEnv; }

		// UNEXPOSED FUNCTIONS
	public:
		void			Init(CPhysicsEnvironment *pEnv, btSoftBody *pSoftBody, const softbodyparams_t *pParams);

		btSoftBody *	GetSoftBody();

	private:
		CPhysicsEnvironment *	m_pEnv;
		btSoftBody *			m_pSoftBody;
};

CPhysicsSoftBody *CreateSoftBody(CPhysicsEnvironment *pEnv);
CPhysicsSoftBody *CreateSoftBodyFromTriMesh(CPhysicsEnvironment *pEnv); // TODO: Not complete
// Vertices are in world space! (You can create this in local space then call Transform to move this to the start position)
CPhysicsSoftBody *CreateSoftBodyFromVertices(CPhysicsEnvironment *pEnv, const Vector *vertices, int numVertices, const softbodyparams_t *pParams);
CPhysicsSoftBody *CreateSoftBodyRope(CPhysicsEnvironment *pEnv, const Vector &position, const Vector &end, int resolution, const softbodyparams_t *pParams);
CPhysicsSoftBody *CreateSoftBodyPatch(CPhysicsEnvironment *pEnv, const Vector *corners, int resx, int resy, const softbodyparams_t *pParams);

#endif // PHYSICS_SOFTBODY_H
#endif