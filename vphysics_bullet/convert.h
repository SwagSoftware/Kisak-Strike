#ifndef CONVERT_H
#define CONVERT_H

#define HL2BULL_FACTOR METERS_PER_INCH
#define HL2BULL_INSQR_PER_METERSQR (1.f / (HL2BULL_FACTOR*HL2BULL_FACTOR))

#define BULL2HL(x) (float)((x) * (1.0f/HL2BULL_FACTOR))
#define HL2BULL(x) (float)((x) * HL2BULL_FACTOR)

#ifdef _MSC_VER
	// Conversion from x to x, possible loss of data
	#pragma warning(disable: 4244)
#endif

// Declarations
inline void ConvertIVPPosToBull(const float *pos, btVector3 &bull);
inline void ConvertPosToBull(const Vector &pos, btVector3 &bull);
inline void ConvertPosToHL(const btVector3 &pos, Vector &hl);
inline void ConvertDirectionToBull(const Vector &dir, btVector3 &bull);
inline void ConvertDirectionToHL(const btVector3 &dir, Vector &hl);

inline void ConvertAABBToBull(const Vector &hlMins, const Vector &hlMaxs, btVector3 &bullMins, btVector3 &bullMaxs);
inline void ConvertAABBToHL(const btVector3 &bullMins, const btVector3 &bullMaxs, Vector &hlMins, Vector &hlMaxs);

inline void ConvertForceImpulseToBull(const Vector &pos, btVector3 &bull);
inline void ConvertForceImpulseToHL(const btVector3 &pos, Vector &hl);
inline btScalar ConvertForceImpulseToBull(float hl);
inline float ConvertForceImpulseToHL(btScalar bull);

inline btScalar ConvertAngleToBull(const float angle);
inline float ConvertAngleToHL(const btScalar &angle);
inline void ConvertRotationToBull(const QAngle &angles, btMatrix3x3 &bull);
inline void ConvertRotationToBull(const QAngle &angles, btQuaternion &bull);
inline void ConvertRotationToHL(const btMatrix3x3 &matrix, QAngle &hl);
inline void ConvertRotationToHL(const btQuaternion &quat, QAngle &hl);
inline void ConvertAngularImpulseToBull(const AngularImpulse &angularimp, btVector3 &bull);
inline void ConvertAngularImpulseToHL(const btVector3 &angularimp, AngularImpulse &hl);
inline void ConvertMatrixToHL(const btTransform &transform, matrix3x4_t &hl);
inline void ConvertMatrixToBull(const matrix3x4_t &hl, btTransform &transform);

inline float ConvertDistanceToBull(float distance);
inline float ConvertDistanceToHL(float distance);
inline float ConvertEnergyToHL(float energy);

/************************************************
* COORDINATE SYSTEMS:
* Bullet vector: Forward, Up, Right
*	+x: forward (east)
*	+y: up
*	+z: right (south)
*
*
*
* HL Vector: Forward, Left, Up
*	+x: forward (east)
*	+y: left (north)
*	+z: up
*
*   (top down)
*
*   left (y)
*    ^
*    |
*   (*)------> forward (x)
*   up (z)
*
************************************************/

// IVP: Forward down left
// IVP Units in meters
inline void ConvertIVPPosToBull(const float *pos, btVector3 &bull) {
	if (!pos) return;

	bull.setX(pos[0]);
	bull.setY(-pos[1]);
	bull.setZ(-pos[2]);
}

inline void ConvertPosToBull(const Vector &pos, btVector3 &bull) {
	bull.setX(HL2BULL(pos.x));
	bull.setY(HL2BULL(pos.z));
	bull.setZ(-HL2BULL(pos.y));
}

inline void ConvertPosToHL(const btVector3 &pos, Vector &hl) {
	hl.x = BULL2HL(pos.x());
	hl.y = -BULL2HL(pos.z());
	hl.z = BULL2HL(pos.y());
}

inline void ConvertAABBToBull(const Vector &hlMins, const Vector &hlMaxs, btVector3 &bullMins, btVector3 &bullMaxs) {
	Assert(hlMins.x <= hlMaxs.x);
	Assert(hlMins.y <= hlMaxs.y);
	Assert(hlMins.z <= hlMaxs.z);

	Vector halfExtents = (hlMaxs - hlMins) / 2;
	btVector3 bullHalfExtents;
	ConvertPosToBull(halfExtents, bullHalfExtents);

	Vector center = hlMins + halfExtents;
	btVector3 bullCenter;
	ConvertPosToBull(center, bullCenter);

	// Half Life AABBs use different corners.
	bullHalfExtents.setZ(-bullHalfExtents.z());

	bullMins = bullCenter - bullHalfExtents;
	bullMaxs = bullCenter + bullHalfExtents;
}

inline void ConvertAABBToHL(const btVector3 &bullMins, const btVector3 &bullMaxs, Vector &hlMins, Vector &hlMaxs) {
	Assert(bullMins.x() <= bullMaxs.x());
	Assert(bullMins.y() <= bullMaxs.y());
	Assert(bullMins.z() <= bullMaxs.z());

	btVector3 halfExtents = (bullMaxs - bullMins) / 2;
	Vector hlHalfExtents;
	ConvertPosToHL(halfExtents, hlHalfExtents);

	btVector3 center = bullMins + halfExtents;
	Vector hlCenter;
	ConvertPosToHL(center, hlCenter);

	// Half Life AABBs use different corners.
	hlHalfExtents.y = -hlHalfExtents.y;

	hlMins = hlCenter - hlHalfExtents;
	hlMaxs = hlCenter + hlHalfExtents;
}

inline void ConvertDirectionToBull(const Vector &dir, btVector3 &bull) {
	bull.setX(dir.x);
	bull.setY(dir.z);
	bull.setZ(-dir.y);
}

inline void ConvertDirectionToHL(const btVector3 &dir, Vector &hl) {
	hl.x = dir.x();
	hl.y = -dir.z();
	hl.z = dir.y();
}

inline void ConvertForceImpulseToBull(const Vector &hl, btVector3 &bull) {
	return ConvertPosToBull(hl, bull);
}

inline void ConvertForceImpulseToHL(const btVector3 &bull, Vector &hl) {
	return ConvertPosToHL(bull, hl);
}

inline btScalar ConvertForceImpulseToBull(float hl) {
	return HL2BULL(hl);
}

inline float ConvertForceImpulseToHL(btScalar bull) {
	return BULL2HL(bull);
}

inline btScalar ConvertAngleToBull(const float angle) {
	return DEG2RAD(angle);
}

inline float ConvertAngleToHL(const btScalar &angle) {
	return RAD2DEG(angle);
}

inline void ConvertRotationToBull(const QAngle &angles, btMatrix3x3 &bull) {
	btQuaternion quat;
	ConvertRotationToBull(angles, quat);
	bull.setRotation(quat);
}

inline void ConvertRotationToBull(const QAngle &angles, btQuaternion &bull) {
	RadianEuler radian(angles);
	Quaternion q(radian);
	bull.setValue(q.x, q.z, -q.y, q.w);
}

inline void ConvertRotationToHL(const btMatrix3x3 &matrix, QAngle &hl) {
	btQuaternion quat;
	matrix.getRotation(quat);
	ConvertRotationToHL(quat, hl);
}

inline void ConvertRotationToHL(const btQuaternion &quat, QAngle &hl) {
	Quaternion q(quat.getX(), -quat.getZ(), quat.getY(), quat.getW());
	RadianEuler radian(q);
	hl = radian.ToQAngle();
}

inline void ConvertAngularImpulseToBull(const AngularImpulse &angularimp, btVector3 &bull) {
	bull.setX(DEG2RAD(angularimp.x));
	bull.setY(DEG2RAD(angularimp.z));
	bull.setZ(-DEG2RAD(angularimp.y));
}

inline void ConvertAngularImpulseToHL(const btVector3 &angularimp, AngularImpulse &hl) {
	hl.x = RAD2DEG(angularimp.x());
	hl.y = -RAD2DEG(angularimp.z());
	hl.z = RAD2DEG(angularimp.y());
}

inline void ConvertMatrixToHL(const btTransform &transform, matrix3x4_t &hl) {
	Vector forward, left, up, pos;

	ConvertDirectionToHL(transform.getBasis().getColumn(0), forward);
	ConvertDirectionToHL(-transform.getBasis().getColumn(2), left);
	ConvertDirectionToHL(transform.getBasis().getColumn(1), up);
	ConvertPosToHL(transform.getOrigin(), pos);

	hl.Init(forward, left, up, pos);
}

inline Vector HLGetMatrixColumn(const matrix3x4_t &hl, int col) {
	Vector ret;
	ret.x = hl[0][col];
	ret.y = hl[1][col];
	ret.z = hl[2][col];

	return ret;
}

inline void ConvertMatrixToBull(const matrix3x4_t &hl, btTransform &transform) {
	Vector forward, left, up, pos;

	forward	= HLGetMatrixColumn(hl, 0);
	left	= HLGetMatrixColumn(hl, 1);
	up		= HLGetMatrixColumn(hl, 2);
	pos		= HLGetMatrixColumn(hl, 3);

	btVector3 bullForward, bullRight, bullUp, origin;
	ConvertDirectionToBull(forward, bullForward);
	ConvertDirectionToBull(-left, bullRight);
	ConvertDirectionToBull(up, bullUp);
	ConvertPosToBull(pos, origin);

	transform.setBasis(btMatrix3x3(bullForward.x(), bullUp.x(), bullRight.x(),
								   bullForward.y(), bullUp.y(), bullRight.y(),
								   bullForward.z(), bullUp.z(), bullRight.z()));
	transform.setOrigin(origin);
}

inline float ConvertDistanceToBull(float distance) {
	return HL2BULL(distance);
}

inline float ConvertDistanceToHL(float distance) {
	return BULL2HL(distance);
}

inline float ConvertEnergyToHL(float energy) {
	return energy * HL2BULL_INSQR_PER_METERSQR;
}

#ifdef _MSC_VER
	#pragma warning (default: 4244)
#endif

#endif
