#include <ctype.h>

#include <tier1/tier1.h>
#include <tier1/utlsymbol.h>
#include <tier0/platform.h>

#include <vphysics_interface.h>
#include <vphysics/collision_set.h> // THIS FILE HAS NO INCLUDE GUARDS!

// NEW INTERFACE HEADERS
#include "vphysics_interfaceV32.h"

#include <cmodel.h>

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletCollision/CollisionShapes/btMaterial.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#if defined(_WIN32)
	#define DEBUG_DRAW 0
#endif

// Thread support is enabled by default
#define BT_THREADSAFE

// Probably shouldn't be using defines for these.
#define SLEEP_LINEAR_THRESHOLD 0.15 // m/s
#define SLEEP_ANGULAR_THRESHOLD 0.1 // rad/s

#define NOT_IMPLEMENTED				DevWarning("VPhysics UNIMPLEMENTED: %s (%s:%u)\n", __FUNCTION__, __FILE__, __LINE__);
#define NOT_IMPLEMENTED_CRITICAL	Error("VPhysics UNIMPLEMENTED: %s (%s:%u)\n", __FUNCTION__, __FILE__, __LINE__);

// NOTE: Not available on non-MSVC builds due to use of __asm.
//#define NOT_IMPLEMENTED_BREAK		{DevWarning("VPhysics UNIMPLEMENTED: %s (%s:%u)\n", __FUNCTION__, __FILE__, __LINE__); __asm int 3;}
