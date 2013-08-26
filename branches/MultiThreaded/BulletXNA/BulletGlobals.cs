/*
 * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

using System;
using System.Collections.Generic;
using System.IO;
using BulletXNA.BulletCollision;
using BulletXNA.LinearMath;
using BulletXNA.BulletDynamics;

namespace BulletXNA
{
    public static class BulletGlobals
    {
        public static bool gDisableDeactivation = false;
        public static int gOverlappingPairs = 0;
        public static float gDeactivationTime = 2f;
        public static int BT_BULLET_VERSION = 276;
        public static int RAND_MAX = int.MaxValue;

        public static float gContactBreakingThreshold = .02f;

        public static Random gRandom = new Random();

        public static IndexedMatrix IdentityMatrix = IndexedMatrix.Identity;

        public static IContactAddedCallback gContactAddedCallback;

        public static IDebugDraw gDebugDraw;

        public static StreamWriter g_streamWriter;

        public static IProfileManager g_profileManager;

        public static void StartProfile(String name)
        {
            if (g_profileManager != null)
            {
                g_profileManager.Start_Profile(name);
            }

        }
        public static void StopProfile()
        {
            if (g_profileManager != null)
            {
                g_profileManager.Stop_Profile();
            }
        }

        public static void ResetProfile()
        {
            if (g_profileManager != null)
            {
                g_profileManager.Reset();
            }
        }


        //public static ManifoldPoint GetManifoldPoint()
        //{
        //    if (m_pointStack.Count == 0)
        //    {
        //        m_pointStack.Push(new ManifoldPoint());
        //    }
        //    ++m_assignedPointCount;
        //    return m_pointStack.Pop();
        //}

        //public static void ReleaseManifoldPoint(ManifoldPoint mp)
        //{
        //    --m_assignedPointCount;
        //    m_pointStack.Push(mp);
        //}

        //private static Stack<ManifoldPoint> m_pointStack = new Stack<ManifoldPoint>(20);
        //private static int m_assignedPointCount = 0;


        public static PooledType<VoronoiSimplexSolver> VoronoiSimplexSolverPool = new PooledType<VoronoiSimplexSolver>();
        public static PooledType<SubSimplexConvexCast> SubSimplexConvexCastPool = new PooledType<SubSimplexConvexCast>();
        public static PooledType<ManifoldPoint> ManifoldPointPool = new PooledType<ManifoldPoint>();
        public static PooledType<CastResult> CastResultPool = new PooledType<CastResult>();
        public static PooledType<SphereShape> SphereShapePool = new PooledType<SphereShape>();
        public static PooledType<DbvtNode> DbvtNodePool = new PooledType<DbvtNode>();
        public static PooledType<SingleRayCallback> SingleRayCallbackPool = new PooledType<SingleRayCallback>();
        public static PooledType<SubSimplexClosestResult> SubSimplexClosestResultPool = new PooledType<SubSimplexClosestResult>();
        public static PooledType<GjkPairDetector> GjkPairDetectorPool = new PooledType<GjkPairDetector>();
        public static PooledType<DbvtTreeCollider> DbvtTreeColliderPool = new PooledType<DbvtTreeCollider>();
        public static PooledType<SingleSweepCallback> SingleSweepCallbackPool = new PooledType<SingleSweepCallback>();
        public static PooledType<BroadphaseRayTester> BroadphaseRayTesterPool = new PooledType<BroadphaseRayTester>();
        public static PooledType<ClosestNotMeConvexResultCallback> ClosestNotMeConvexResultCallbackPool = new PooledType<ClosestNotMeConvexResultCallback>();
        public static PooledType<GjkEpaPenetrationDepthSolver> GjkEpaPenetrationDepthSolverPool = new PooledType<GjkEpaPenetrationDepthSolver>();
        public static PooledType<ContinuousConvexCollision> ContinuousConvexCollisionPool = new PooledType<ContinuousConvexCollision>();
        public static PooledType<DbvtStackDataBlock> DbvtStackDataBlockPool = new PooledType<DbvtStackDataBlock>();

        public static PooledType<BoxBoxCollisionAlgorithm> BoxBoxCollisionAlgorithmPool = new PooledType<BoxBoxCollisionAlgorithm>();
        public static PooledType<CompoundCollisionAlgorithm> CompoundCollisionAlgorithmPool = new PooledType<CompoundCollisionAlgorithm>();
        public static PooledType<ConvexConcaveCollisionAlgorithm> ConvexConcaveCollisionAlgorithmPool = new PooledType<ConvexConcaveCollisionAlgorithm>();
        public static PooledType<ConvexConvexAlgorithm> ConvexConvexAlgorithmPool = new PooledType<ConvexConvexAlgorithm>();
        public static PooledType<ConvexPlaneCollisionAlgorithm> ConvexPlaneAlgorithmPool = new PooledType<ConvexPlaneCollisionAlgorithm>();
        public static PooledType<SphereBoxCollisionAlgorithm> SphereBoxCollisionAlgorithmPool = new PooledType<SphereBoxCollisionAlgorithm>();
        public static PooledType<SphereSphereCollisionAlgorithm> SphereSphereCollisionAlgorithmPool = new PooledType<SphereSphereCollisionAlgorithm>();
        public static PooledType<SphereTriangleCollisionAlgorithm> SphereTriangleCollisionAlgorithmPool = new PooledType<SphereTriangleCollisionAlgorithm>();
        public static PooledType<GImpactCollisionAlgorithm> GImpactCollisionAlgorithmPool = new PooledType<GImpactCollisionAlgorithm>();
        public static PooledType<GjkEpaSolver2MinkowskiDiff> GjkEpaSolver2MinkowskiDiffPool = new PooledType<GjkEpaSolver2MinkowskiDiff>();
        public static PooledType<PersistentManifold> PersistentManifoldPool = new PooledType<PersistentManifold>();
        public static PooledType<ManifoldResult> ManifoldResultPool = new PooledType<ManifoldResult>();
        public static PooledType<GJK> GJKPool = new PooledType<GJK>();
        public static PooledType<GIM_ShapeRetriever> GIM_ShapeRetrieverPool = new PooledType<GIM_ShapeRetriever>();
        public static PooledType<TriangleShape> TriangleShapePool = new PooledType<TriangleShape>();
        public static PooledType<SphereTriangleDetector> SphereTriangleDetectorPool = new PooledType<SphereTriangleDetector>();
        public static PooledType<CompoundLeafCallback> CompoundLeafCallbackPool = new PooledType<CompoundLeafCallback>();
        public static PooledType<GjkConvexCast> GjkConvexCastPool = new PooledType<GjkConvexCast>();
        public static PooledType<LocalTriangleSphereCastCallback> LocalTriangleSphereCastCallbackPool = new PooledType<LocalTriangleSphereCastCallback>();
        public static PooledType<BridgeTriangleRaycastCallback> BridgeTriangleRaycastCallbackPool = new PooledType<BridgeTriangleRaycastCallback>();
        public static PooledType<BridgeTriangleConcaveRaycastCallback> BridgeTriangleConcaveRaycastCallbackPool = new PooledType<BridgeTriangleConcaveRaycastCallback>();
        public static PooledType<BridgeTriangleConvexcastCallback> BridgeTriangleConvexcastCallbackPool = new PooledType<BridgeTriangleConvexcastCallback>();
        public static PooledType<MyNodeOverlapCallback> MyNodeOverlapCallbackPool = new PooledType<MyNodeOverlapCallback>();
        public static PooledType<ClosestRayResultCallback> ClosestRayResultCallbackPool = new PooledType<ClosestRayResultCallback>();
        public static PooledType<DebugDrawcallback> DebugDrawcallbackPool = new PooledType<DebugDrawcallback>();

        public const bool debugRigidBody = true;
        public const bool debugCollisionWorld = false;
		public const bool debugConstraints = false;
		public const bool debugDiscreteDynamicsWorld = true;
		public const bool debugBoxBoxDetector = false;
		public const bool debugIslands = false;
		public const bool debugBVHTriangleMesh = false;
		public const bool debugConvexHull = false;
		public const bool debugConvexShape = false;
		public const bool debugShapeHull = false;
		public const bool debugStridingMesh = false;
		public const bool debugGJK = false;
		public const bool debugGJKDetector = false;
		public const bool debugPersistentManifold = false;
		public const bool debugVoronoiSimplex = false;
        public const bool debugSolver = true;
        public const bool debugBroadphase = false;
        public const bool debugBoxShape = false;
        public const bool debugGimpactShape= false;
        public const bool debugGimpactAlgo = false;
        public const bool debugGimpactBVH = false;
        public const bool debugPairCache = false;
        public const bool debugDispatcher = false;
        public const bool debugManifoldResult = false;


        // tracking
        public static int s_collisionAlgorithmInstanceCount = 0;




    }
}
