#ifndef SOFTBODYV32_H
#define SOFTBODYV32_H
#ifdef _MSC_VER
#pragma once
#endif

struct softbodynode_t {
	Vector pos, vel;
	float invMass;
};

struct softbodyface_t {
	int nodeIndexes[3]; // Node indexes that can be passed into the node functions
	softbodynode_t nodes[3];
	Vector normal;
};

struct softbodylink_t {
	int nodeIndexes[2]; // Node indexes that can be passed into the node functions
	softbodynode_t nodes[2];
};

struct softbodyparams_t {
	float	totalMass;
};

class IPhysicsSoftBody {
	public:
		virtual ~IPhysicsSoftBody() {}

		virtual void	SetTotalMass(float fMass, bool bFromFaces = false) = 0;
		virtual void	Anchor(int node, IPhysicsObject *pObj) = 0;

		virtual int		GetNodeCount() const = 0;
		virtual int		GetFaceCount() const = 0;
		virtual int		GetLinkCount() const = 0;

		virtual softbodynode_t	GetNode(int i) const = 0;
		virtual softbodyface_t	GetFace(int i) const = 0;
		virtual softbodylink_t	GetLink(int i) const = 0;

		// Get soft body AABB (cannot be implemented in collision interface because soft bodies change shape)
		virtual void	GetAABB(Vector *mins, Vector *maxs) const = 0;
		virtual void	RayTest(Ray_t &ray, trace_t *pTrace) const = 0;

		virtual void	Transform(const matrix3x4_t &mat) = 0;
		virtual void	Transform(const Vector *vec, const QAngle *ang) = 0;
		virtual void	Scale(const Vector &scale) = 0; // Scales a softbody. Do this before it's moved from the origin, or bad things will happen!

		virtual IPhysicsEnvironment32 *GetPhysicsEnvironment() const = 0;
};

#endif // SOFTBODYV32_H