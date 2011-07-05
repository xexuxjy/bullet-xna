#ifndef BULLET_DEBUG_GLOBAL
#define BULLET_DEBUG_GLOBAL

struct btBulletDebug
{
		static const  bool debugRigidBody = true;
		static const bool debugCollisionWorld = false;
		static const bool debugConstraints = true;
		static const bool debugDiscreteDynamicsWorld = false;
		static const bool debugBoxBoxDetector = false;
		static const bool debugIslands = false;
		static const bool debugBVHTriangleMesh = false;
		static const bool debugConvexHull = false;
		static const bool debugConvexShape = false;
		static const bool debugShapeHull = false;
		static const bool debugStridingMesh = false;
		static const bool debugGJK = false;
		static const bool debugGJKDetector = false;
		static const bool debugPersistentManifold = false;
		static const bool debugVoronoiSimplex = false;
		static const bool debugSolver = false;
		static const bool debugBroadphase = false;
		static const bool debugBoxShape = false;
		static const bool debugGimpactShape = true;
		static const bool debugGimpactAlgo = true;
		static const bool debugGimpactBVH = false;
};

#endif