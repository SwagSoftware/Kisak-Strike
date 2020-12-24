#include "StdAfx.h"
#if 0
#include "convert.h"
#include "Physics_SoftBody.h"
#include "Physics_Environment.h"
#include "Physics_Object.h"

#include "BulletSoftBody/btSoftBodyHelpers.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

void ConvertNodeToHL(const btSoftBody::Node *node, softbodynode_t &nodeOut) {
	ConvertPosToHL(node->m_x, nodeOut.pos);
	ConvertPosToHL(node->m_v, nodeOut.vel);
	nodeOut.invMass = node->m_im;
}

void ConvertNodeToBull(const softbodynode_t &node, btSoftBody::Node &nodeOut) {
	ConvertPosToBull(node.pos, nodeOut.m_x);
	ConvertPosToBull(node.vel, nodeOut.m_v);
	nodeOut.m_im = node.invMass;
}

void ConvertFaceToHL(const btSoftBody::Face *face, softbodyface_t &faceOut) {
	for (int i = 0; i < 3; i++) {
		btSoftBody::Node *node = face->m_n[i];
		ConvertNodeToHL(node, faceOut.nodes[i]);
	}

	ConvertDirectionToHL(face->m_normal, faceOut.normal);
}

void ConvertLinkToHL(const btSoftBody::Link *link, softbodylink_t &linkOut) {
	for (int i = 0; i < 2; i++) {
		btSoftBody::Node *node = link->m_n[i];
		ConvertNodeToHL(node, linkOut.nodes[i]);
	}
}

/*************************
* CLASS CPhysicsSoftBody
*************************/

CPhysicsSoftBody::CPhysicsSoftBody() {
	m_pEnv = NULL;
	m_pSoftBody = NULL;
}

CPhysicsSoftBody::~CPhysicsSoftBody() {
	m_pEnv->GetBulletEnvironment()->removeSoftBody(m_pSoftBody);
	delete m_pSoftBody;
}

bool CPhysicsSoftBody::IsAsleep() const {
	return m_pSoftBody->getActivationState() == ISLAND_SLEEPING || m_pSoftBody->getActivationState() == DISABLE_SIMULATION;
}

void CPhysicsSoftBody::SetTotalMass(float fMass, bool bFromFaces) {
	m_pSoftBody->setTotalMass(fMass, bFromFaces);
}

void CPhysicsSoftBody::Anchor(int node, IPhysicsObject *pObj) {
	m_pSoftBody->appendAnchor(node, ((CPhysicsObject *)pObj)->GetObject());
}

int CPhysicsSoftBody::GetNodeCount() const {
	return m_pSoftBody->m_nodes.size();
}

int CPhysicsSoftBody::GetFaceCount() const {
	return m_pSoftBody->m_faces.size();
}

int CPhysicsSoftBody::GetLinkCount() const {
	return m_pSoftBody->m_links.size();
}

softbodynode_t CPhysicsSoftBody::GetNode(int i) const {
	Assert(i >= 0 && i < m_pSoftBody->m_nodes.size());
	btSoftBody::Node &node = m_pSoftBody->m_nodes[i];

	softbodynode_t out;
	ConvertNodeToHL(&node, out);
	return out;
}

softbodylink_t CPhysicsSoftBody::GetLink(int i) const {
	Assert(i >= 0 && i < m_pSoftBody->m_links.size());
	btSoftBody::Link &link = m_pSoftBody->m_links[i];

	softbodylink_t out;
	ConvertLinkToHL(&link, out);

	// Find the node ids with an educated guess (this will be some huge number if something bad happened)
	for (int i = 0; i < 2; i++) {
		out.nodeIndexes[i] = (int)(link.m_n[i] - &m_pSoftBody->m_nodes[0]);
	}

	return out;
}

softbodyface_t CPhysicsSoftBody::GetFace(int i) const {
	Assert(i >= 0 && i < m_pSoftBody->m_faces.size());
	btSoftBody::Face &face = m_pSoftBody->m_faces[i];

	softbodyface_t out;
	ConvertFaceToHL(&face, out);

	// Find the node ids with an educated guess
	for (int i = 0; i < 3; i++) {
		out.nodeIndexes[i] = (int)(face.m_n[i] - &m_pSoftBody->m_nodes[0]);
	}

	return out;
}

void CPhysicsSoftBody::SetNode(int i, softbodynode_t &node) {
	Assert(i >= 0 && i < m_pSoftBody->m_nodes.size());

	btSoftBody::Node &bnode = m_pSoftBody->m_nodes[i];
	ConvertNodeToBull(node, bnode);
}

void CPhysicsSoftBody::AddNode(const Vector &pos, float mass) {
	btVector3 btpos;
	ConvertPosToBull(pos, btpos);

	m_pSoftBody->appendNode(btpos, mass);
}

void CPhysicsSoftBody::AddLink(int node1, int node2, bool bCheckExist) {
	m_pSoftBody->appendLink(node1, node2, NULL, bCheckExist);
}

void CPhysicsSoftBody::GetAABB(Vector *mins, Vector *maxs) const {
	if (!mins && !maxs) return;

	btVector3 btmins, btmaxs;
	m_pSoftBody->getAabb(btmins, btmaxs);

	Vector tMins, tMaxs;
	ConvertAABBToHL(btmins, btmaxs, tMins, tMaxs);

	if (mins)
		*mins = tMins;

	if (maxs)
		*maxs = tMaxs;
}

void CPhysicsSoftBody::RemoveNode(int i) {
	Assert(i >= 0 && i < m_pSoftBody->m_nodes.size());

	m_pSoftBody->pointersToIndices();
	//m_pSoftBody->m_nodes.remove(m_pSoftBody->m_nodes[i]);
	m_pSoftBody->indicesToPointers();
}

void CPhysicsSoftBody::RemoveLink(int i) {
	Assert(i >= 0 && i < m_pSoftBody->m_links.size());

	btSoftBody::Link &link = m_pSoftBody->m_links[i];

	m_pSoftBody->pointersToIndices();
	// TODO: Need to update the indices of anything referencing
	// the link that is swapped to this position
	//m_pSoftBody->m_links.remove(link);
	m_pSoftBody->indicesToPointers();
}

void CPhysicsSoftBody::RemoveFace(int i) {
	Assert(i >= 0 && i < m_pSoftBody->m_faces.size());

	btSoftBody::Face &face = m_pSoftBody->m_faces[i];

	m_pSoftBody->pointersToIndices();
	//m_pSoftBody->m_faces.remove(face);
	m_pSoftBody->indicesToPointers();
}

void CPhysicsSoftBody::RayTest(Ray_t &ray, trace_t *pTrace) const {
	if (!pTrace) return;

	btVector3 start, end;
	ConvertPosToBull(ray.m_Start, start);
	ConvertPosToBull(ray.m_Start + ray.m_Delta, end);

	btSoftBody::sRayCast rayCast;
	m_pSoftBody->rayTest(start, end, rayCast);
	pTrace->fraction = rayCast.fraction;
	pTrace->startpos = ray.m_Start + ray.m_StartOffset;
	pTrace->endpos = ray.m_Start + ray.m_StartOffset + (ray.m_Delta * rayCast.fraction);
}

void CPhysicsSoftBody::BoxTest(Ray_t &ray, trace_t *pTrace) const {
	if (!pTrace) return;

	btVector3 start, end;
	ConvertPosToBull(ray.m_Start, start);
	ConvertPosToBull(ray.m_Start + ray.m_Delta, end);

	NOT_IMPLEMENTED
}

void CPhysicsSoftBody::Transform(const matrix3x4_t &mat) {
	btTransform trans;
	ConvertMatrixToBull(mat, trans);

	m_pSoftBody->transform(trans);
}

void CPhysicsSoftBody::Transform(const Vector *vec, const QAngle *ang) {
	if (!vec && !ang) return;

	if (vec) {
		btVector3 bVec;
		ConvertPosToBull(*vec, bVec);

		m_pSoftBody->translate(bVec);
	}

	if (ang) {
		btQuaternion quat;
		ConvertRotationToBull(*ang, quat);

		m_pSoftBody->rotate(quat);
	}
}

void CPhysicsSoftBody::Scale(const Vector &scale) {
	// No conversion
	btVector3 btScale;
	btScale.setX(scale.x);
	btScale.setY(scale.z);
	btScale.setZ(scale.y);

	m_pSoftBody->scale(btScale);
}

void CPhysicsSoftBody::Init(CPhysicsEnvironment *pEnv, btSoftBody *pSoftBody, const softbodyparams_t *pParams) {
	m_pEnv			= pEnv;
	m_pSoftBody		= pSoftBody;

	m_pSoftBody->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
	m_pSoftBody->m_cfg.viterations = 1;
	pEnv->GetBulletEnvironment()->addSoftBody(m_pSoftBody);
}

btSoftBody *CPhysicsSoftBody::GetSoftBody() {
	return m_pSoftBody;
}

/*************************
* CREATION FUNCTIONS
*************************/

CPhysicsSoftBody *CreateSoftBody(CPhysicsEnvironment *pEnv) {
	btSoftBody *pSoftBody = new btSoftBody(&pEnv->GetSoftBodyWorldInfo());
	CPhysicsSoftBody *pPhysBody = new CPhysicsSoftBody;
	return pPhysBody;
}

CPhysicsSoftBody *CreateSoftBodyFromTriMesh(CPhysicsEnvironment *pEnv, const Vector *vertices, int numVertices, const int *indices, int numIndices, const Vector &position, const QAngle &angles, const softbodyparams_t *pParams) {
	btVector3 *bullVerts = new btVector3[numVertices];

	// Make sure numIndices is evenly divisible by 3
	Assert(numIndices % 3 == 0);

	for (int i = 0; i < numVertices; i++) {
		ConvertPosToBull(vertices[i], bullVerts[i]);
	}

	/*
	btSoftBody *pSoftBody = new btSoftBody(&pEnv->GetSoftBodyWorldInfo(), numVertices, bullVerts, NULL);
	for (int i = 0; i < numIndices; i++) {

	}
	*/

	delete [] bullVerts;

	NOT_IMPLEMENTED
	return NULL;
}

CPhysicsSoftBody *CreateSoftBodyFromVertices(CPhysicsEnvironment *pEnv, const Vector *vertices, int numVertices, const softbodyparams_t *pParams) {
	btVector3 *bullVerts = new btVector3[numVertices];

	for (int i = 0; i < numVertices; i++) {
		ConvertPosToBull(vertices[i], bullVerts[i]);
	}

	btSoftBodyWorldInfo &wi = pEnv->GetSoftBodyWorldInfo();

	btSoftBody *pSoftBody = btSoftBodyHelpers::CreateFromConvexHull(wi, bullVerts, numVertices);
	delete [] bullVerts;

	CPhysicsSoftBody *pBody = new CPhysicsSoftBody;
	pBody->Init(pEnv, pSoftBody, pParams);

	return pBody;
}

CPhysicsSoftBody *CreateSoftBodyRope(CPhysicsEnvironment *pEnv, const Vector &position, const Vector &end, int resolution, const softbodyparams_t *pParams) {
	btSoftBodyWorldInfo &wi = pEnv->GetSoftBodyWorldInfo();

	btVector3 btStart, btEnd;
	ConvertPosToBull(position, btStart);
	ConvertPosToBull(end, btEnd);

	// Last parameter is fixed sides of the soft body (which we don't set - dev can set these elsewhere)
	btSoftBody *pSoftBody = btSoftBodyHelpers::CreateRope(wi, btStart, btEnd, resolution, 0);
	if (pParams) {
		pSoftBody->setTotalMass(pParams->totalMass);
	}

	CPhysicsSoftBody *pPhysBody = new CPhysicsSoftBody;
	pPhysBody->Init(pEnv, pSoftBody, pParams);

	return pPhysBody;
}

CPhysicsSoftBody *CreateSoftBodyPatch(CPhysicsEnvironment *pEnv, const Vector *corners, int resx, int resy, const softbodyparams_t *pParams) {
	btSoftBodyWorldInfo &wi = pEnv->GetSoftBodyWorldInfo();

	btVector3 bcorners[4];
	for (int i = 0; i < 4; i++) {
		ConvertPosToBull(corners[i], bcorners[i]);
	}

	btSoftBody *pSoftBody = btSoftBodyHelpers::CreatePatch(wi, bcorners[0], bcorners[1], bcorners[2], bcorners[3], resx, resy, 0, false);
	if (pParams) {
		pSoftBody->setTotalMass(pParams->totalMass);
	}

	CPhysicsSoftBody *pPhysBody = new CPhysicsSoftBody;
	pPhysBody->Init(pEnv, pSoftBody, pParams);

	return pPhysBody;
}
#endif