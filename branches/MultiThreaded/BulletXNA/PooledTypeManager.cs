using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BulletXNA.LinearMath;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;

namespace BulletXNA
{
    public class PooledTypeManager
    {

        public PooledTypeManager(IDispatcher dispatcher)
        {
            Dispatcher = dispatcher;
            VoronoiSimplexSolverPool = new VoronoiSimplexSolverPoolV2(this);
            SubSimplexConvexCastPool = new PooledType<SubSimplexConvexCast>();
            ManifoldPointPool = new PooledType<ManifoldPoint>();
            CastResultPool = new PooledType<CastResult>();
            SphereShapePool = new PooledType<SphereShape>();
            DbvtNodePool = new PooledType<DbvtNode>();
            SingleRayCallbackPool = new PooledType<SingleRayCallback>();
            SubSimplexClosestResultPool = new PooledType<SubSimplexClosestResult>();
            GjkPairDetectorPool = new PooledType<GjkPairDetector>();
            DbvtTreeColliderPool = new PooledType<DbvtTreeCollider>();
            SingleSweepCallbackPool = new PooledType<SingleSweepCallback>();
            BroadphaseRayTesterPool = new PooledType<BroadphaseRayTester>();
            ClosestNotMeConvexResultCallbackPool = new PooledType<ClosestNotMeConvexResultCallback>();
            GjkEpaPenetrationDepthSolverPool = new PooledType<GjkEpaPenetrationDepthSolver>();
            ContinuousConvexCollisionPool = new PooledType<ContinuousConvexCollision>();
            DbvtStackDataBlockPool = new PooledType<DbvtStackDataBlock>();

            BoxBoxCollisionAlgorithmPool = new PooledType<BoxBoxCollisionAlgorithm>();
            CompoundCollisionAlgorithmPool = new PooledType<CompoundCollisionAlgorithm>();
            ConvexConcaveCollisionAlgorithmPool = new PooledType<ConvexConcaveCollisionAlgorithm>();
            ConvexConvexAlgorithmPool = new PooledType<ConvexConvexAlgorithm>();
            ConvexPlaneAlgorithmPool = new PooledType<ConvexPlaneCollisionAlgorithm>();
            SphereBoxCollisionAlgorithmPool = new PooledType<SphereBoxCollisionAlgorithm>();
            SphereSphereCollisionAlgorithmPool = new PooledType<SphereSphereCollisionAlgorithm>();
            SphereTriangleCollisionAlgorithmPool = new PooledType<SphereTriangleCollisionAlgorithm>();
            GImpactCollisionAlgorithmPool = new PooledType<GImpactCollisionAlgorithm>();
            GjkEpaSolver2MinkowskiDiffPool = new PooledType<GjkEpaSolver2MinkowskiDiff>();
            PersistentManifoldPool = new PooledType<PersistentManifold>();
            ManifoldResultPool = new PooledType<ManifoldResult>();
            GJKPool = new PooledType<GJK>();
            GIM_ShapeRetrieverPool = new PooledType<GIM_ShapeRetriever>();
            TriangleShapePool = new PooledType<TriangleShape>();
            SphereTriangleDetectorPool = new PooledType<SphereTriangleDetector>();
            CompoundLeafCallbackPool = new PooledType<CompoundLeafCallback>();
            GjkConvexCastPool = new PooledType<GjkConvexCast>();
            LocalTriangleSphereCastCallbackPool = new PooledType<LocalTriangleSphereCastCallback>();
            BridgeTriangleRaycastCallbackPool = new PooledType<BridgeTriangleRaycastCallback>();
            BridgeTriangleConcaveRaycastCallbackPool = new PooledType<BridgeTriangleConcaveRaycastCallback>();
            BridgeTriangleConvexcastCallbackPool = new PooledType<BridgeTriangleConvexcastCallback>();
            MyNodeOverlapCallbackPool = new PooledType<MyNodeOverlapCallback>();
            ClosestRayResultCallbackPool = new PooledType<ClosestRayResultCallback>();
            DebugDrawcallbackPool = new PooledType<DebugDrawcallback>();
            EPAPool = new EPAPoolV2(this);
        }

        public IDispatcher Dispatcher
        {
            get;
            set;
        }


        public PooledType<VoronoiSimplexSolver> VoronoiSimplexSolverPool;
        public PooledType<SubSimplexConvexCast> SubSimplexConvexCastPool;
        public PooledType<ManifoldPoint> ManifoldPointPool;
        public PooledType<CastResult> CastResultPool;
        public PooledType<SphereShape> SphereShapePool;
        public PooledType<DbvtNode> DbvtNodePool;
        public PooledType<SingleRayCallback> SingleRayCallbackPool;
        public PooledType<SubSimplexClosestResult> SubSimplexClosestResultPool;
        public PooledType<GjkPairDetector> GjkPairDetectorPool;
        public PooledType<DbvtTreeCollider> DbvtTreeColliderPool;
        public PooledType<SingleSweepCallback> SingleSweepCallbackPool;
        public PooledType<BroadphaseRayTester> BroadphaseRayTesterPool;
        public PooledType<ClosestNotMeConvexResultCallback> ClosestNotMeConvexResultCallbackPool;
        public PooledType<GjkEpaPenetrationDepthSolver> GjkEpaPenetrationDepthSolverPool;
        public PooledType<ContinuousConvexCollision> ContinuousConvexCollisionPool;
        public PooledType<DbvtStackDataBlock> DbvtStackDataBlockPool;

        public PooledType<BoxBoxCollisionAlgorithm> BoxBoxCollisionAlgorithmPool;
        public PooledType<CompoundCollisionAlgorithm> CompoundCollisionAlgorithmPool;
        public PooledType<ConvexConcaveCollisionAlgorithm> ConvexConcaveCollisionAlgorithmPool;
        public PooledType<ConvexConvexAlgorithm> ConvexConvexAlgorithmPool;
        public PooledType<ConvexPlaneCollisionAlgorithm> ConvexPlaneAlgorithmPool;
        public PooledType<SphereBoxCollisionAlgorithm> SphereBoxCollisionAlgorithmPool;
        public PooledType<SphereSphereCollisionAlgorithm> SphereSphereCollisionAlgorithmPool;
        public PooledType<SphereTriangleCollisionAlgorithm> SphereTriangleCollisionAlgorithmPool;
        public PooledType<GImpactCollisionAlgorithm> GImpactCollisionAlgorithmPool;
        public PooledType<GjkEpaSolver2MinkowskiDiff> GjkEpaSolver2MinkowskiDiffPool;
        public PooledType<PersistentManifold> PersistentManifoldPool;
        public PooledType<ManifoldResult> ManifoldResultPool;
        public PooledType<GJK> GJKPool;
        public PooledType<GIM_ShapeRetriever> GIM_ShapeRetrieverPool;
        public PooledType<TriangleShape> TriangleShapePool;
        public PooledType<SphereTriangleDetector> SphereTriangleDetectorPool;
        public PooledType<CompoundLeafCallback> CompoundLeafCallbackPool;
        public PooledType<GjkConvexCast> GjkConvexCastPool;
        public PooledType<LocalTriangleSphereCastCallback> LocalTriangleSphereCastCallbackPool;
        public PooledType<BridgeTriangleRaycastCallback> BridgeTriangleRaycastCallbackPool;
        public PooledType<BridgeTriangleConcaveRaycastCallback> BridgeTriangleConcaveRaycastCallbackPool;
        public PooledType<BridgeTriangleConvexcastCallback> BridgeTriangleConvexcastCallbackPool;
        public PooledType<MyNodeOverlapCallback> MyNodeOverlapCallbackPool;
        public PooledType<ClosestRayResultCallback> ClosestRayResultCallbackPool;
        public PooledType<DebugDrawcallback> DebugDrawcallbackPool;
        public PooledType<EPA> EPAPool;

    }

    public class VoronoiSimplexSolverPoolV2 : PooledType<VoronoiSimplexSolver>
    {
        PooledTypeManager m_manager;
        public VoronoiSimplexSolverPoolV2(PooledTypeManager manager)
        {
            m_manager = manager;
        }

        public override VoronoiSimplexSolver Get()
        {
            VoronoiSimplexSolver result = base.Get();
            result.m_dispatcher = m_manager.Dispatcher;
            return result;
        }
    }
    public class EPAPoolV2 : PooledType<EPA>
    {
        PooledTypeManager m_manager;
        public EPAPoolV2(PooledTypeManager manager)
        {
            m_manager = manager;
        }

        public override EPA Get()
        {
            EPA result = base.Get();
            result.m_dispatcher = m_manager.Dispatcher;
            return result;
        }
    }


}
