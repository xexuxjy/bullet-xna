/*
 * 
 * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
 *
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#define GIMPACT_VS_PLANE_COLLISION

using BulletXNA.LinearMath;
using System;



namespace BulletXNA.BulletCollision
{

    //! Class for accessing the plane equation
    public static class PlaneShape
    {
        public static void GetPlaneEquation(StaticPlaneShape plane,out IndexedVector4 equation)
        {
            equation = new IndexedVector4(plane.GetPlaneNormal(),plane.GetPlaneConstant());

        }


        public static void GetPlaneEquationTransformed(StaticPlaneShape plane,ref IndexedMatrix trans, out IndexedVector4 equation)
        {
            equation = new IndexedVector4();
            IndexedVector3 planeNormal = plane.GetPlaneNormal();

            equation.X = trans._basis.GetRow(0).Dot(ref planeNormal);
            equation.Y = trans._basis.GetRow(1).Dot(ref planeNormal);
            equation.Z = trans._basis.GetRow(2).Dot(ref planeNormal);
            equation.W = trans._origin.Dot(ref planeNormal) + plane.GetPlaneConstant();
        }
    }

    public class GImpactCollisionAlgorithm : ActivatingCollisionAlgorithm
    {
        protected CollisionAlgorithm m_convex_algorithm;
        protected PersistentManifold m_manifoldPtr;
        protected ManifoldResult m_resultOut;
        protected DispatcherInfo m_dispatchInfo;
        protected int m_triface0;
        protected int m_part0;
        protected int m_triface1;
        protected int m_part1;
        GIM_TRIANGLE_CONTACT m_contact_data = new GIM_TRIANGLE_CONTACT();

        public GImpactCollisionAlgorithm() { } // for pool

        //! Creates a new contact point
        protected PersistentManifold NewContactManifold(CollisionObject body0, CollisionObject body1)
        {
            m_manifoldPtr = m_dispatcher.GetNewManifold(body0, body1);
            return m_manifoldPtr;
        }

        protected void DestroyConvexAlgorithm()
        {
            if (m_convex_algorithm != null)
            {
                m_dispatcher.FreeCollisionAlgorithm(m_convex_algorithm);
                m_convex_algorithm = null;
            }
        }

        protected void DestroyContactManifolds()
        {
            if (m_manifoldPtr == null)
            {
                return;
            }
            m_dispatcher.ReleaseManifold(m_manifoldPtr);
            m_manifoldPtr = null;
        }

        protected void ClearCache()
        {
            DestroyContactManifolds();
            DestroyConvexAlgorithm();

            m_triface0 = -1;
            m_part0 = -1;
            m_triface1 = -1;
            m_part1 = -1;
        }

        protected PersistentManifold GetLastManifold()
        {
            return m_manifoldPtr;
        }


        // Call before process collision
        protected void CheckManifold(CollisionObject body0, CollisionObject body1)
        {
            if (GetLastManifold() == null)
            {
                NewContactManifold(body0, body1);
            }

            m_resultOut.SetPersistentManifold(GetLastManifold());
        }

        // Call before process collision
        protected CollisionAlgorithm NewAlgorithm(CollisionObject body0, CollisionObject body1)
        {
            CheckManifold(body0, body1);

            CollisionAlgorithm convex_algorithm = m_dispatcher.FindAlgorithm(
                    body0, body1, GetLastManifold());
            return convex_algorithm;
        }

        // Call before process collision
        protected void CheckConvexAlgorithm(CollisionObject body0, CollisionObject body1)
        {
#if DEBUG        
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
            {
                BulletGlobals.g_streamWriter.WriteLine("GImpactAlgo::CheckConvexAlgo");
            }
#endif
            if (m_convex_algorithm != null)
            {
                return;
            }
            m_convex_algorithm = NewAlgorithm(body0, body1);
        }



        protected void AddContactPoint(CollisionObject body0,
                        CollisionObject body1,
                        IndexedVector3 point,
                        IndexedVector3 normal,
                        float distance)
        {
            AddContactPoint(body0, body1, ref point, ref normal, distance);
        }

        protected void AddContactPoint(CollisionObject body0,
                        CollisionObject body1,
                        ref IndexedVector3 point,
                        ref IndexedVector3 normal,
                        float distance)
        {
#if DEBUG        
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
            {
                BulletGlobals.g_streamWriter.WriteLine("GImpactAlgo::AddContactPoint");
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "point", point);
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "normal", normal);

            }
#endif
            m_resultOut.SetShapeIdentifiersA(m_part0, m_triface0);
            m_resultOut.SetShapeIdentifiersB(m_part1, m_triface1);
            CheckManifold(body0, body1);
            m_resultOut.AddContactPoint(normal, point, distance);

        }

        //! Collision routines
        //!@{

        protected void CollideGjkTriangles(CollisionObject body0,
                      CollisionObject body1,
                      GImpactMeshShapePart shape0,
                      GImpactMeshShapePart shape1,
                      ObjectArray<int> pairs, int pair_count)
        {
            TriangleShapeEx tri0 = new TriangleShapeEx();
            TriangleShapeEx tri1 = new TriangleShapeEx();

            shape0.LockChildShapes();
            shape1.LockChildShapes();

            int pair_pointer = 0;
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
            {
                BulletGlobals.g_streamWriter.WriteLine("GImpactAglo::CollideGjkTriangles [{0}]", pair_count);
            }
#endif
            while (pair_count-- != 0)
            {

                m_triface0 = pairs[pair_pointer];
                m_triface1 = pairs[pair_pointer + 1];
                pair_pointer += 2;



                shape0.GetBulletTriangle(m_triface0, tri0);
                shape1.GetBulletTriangle(m_triface1, tri1);


                //collide two convex shapes
                if (tri0.OverlapTestConservative(tri1))
                {
                    ConvexVsConvexCollision(body0, body1, tri0, tri1);
                }

            }

            shape0.UnlockChildShapes();
            shape1.UnlockChildShapes();

        }

        protected void CollideSatTriangles(CollisionObject body0,
                          CollisionObject body1,
                          GImpactMeshShapePart shape0,
                          GImpactMeshShapePart shape1,
                          PairSet pairs, int pair_count)
        {
            IndexedMatrix orgtrans0 = body0.GetWorldTransform();
            IndexedMatrix orgtrans1 = body1.GetWorldTransform();

            PrimitiveTriangle ptri0 = new PrimitiveTriangle();
            PrimitiveTriangle ptri1 = new PrimitiveTriangle();

#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
            {
                BulletGlobals.g_streamWriter.WriteLine("GImpactAglo::CollideSatTriangles [{0}]", pair_count);
            }
#endif

            shape0.LockChildShapes();
            shape1.LockChildShapes();

            int pair_pointer = 0;

            while (pair_count-- != 0)
            {

                m_triface0 = pairs[pair_pointer].m_index1;
                m_triface1 = pairs[pair_pointer].m_index2;
                pair_pointer += 1;


                shape0.GetPrimitiveTriangle(m_triface0, ptri0);
                shape1.GetPrimitiveTriangle(m_triface1, ptri1);

#if TRI_COLLISION_PROFILING
		BulletGlobal.StartProfile("gim02_tri_time");
#endif

                ptri0.ApplyTransform(ref orgtrans0);
                ptri1.ApplyTransform(ref orgtrans1);


                //build planes
                ptri0.BuildTriPlane();
                ptri1.BuildTriPlane();
                // test conservative

                if (ptri0.OverlapTestConservative(ptri1))
                {
                    if (ptri0.FindTriangleCollisionClipMethod(ptri1, m_contact_data))
                    {

                        int j = m_contact_data.m_point_count;
                        while (j-- != 0)
                        {

                            AddContactPoint(body0, body1,
                                        m_contact_data.m_points[j],
                                        MathUtil.Vector4ToVector3(ref m_contact_data.m_separating_normal),
                                        -m_contact_data.m_penetration_depth);
                        }
                    }
                }

#if TRI_COLLISION_PROFILING
		BulletGlobals.StopProfile();
#endif

            }

            shape0.UnlockChildShapes();
            shape1.UnlockChildShapes();


        }




        protected void ShapeVsShapeCollision(
                          CollisionObject body0,
                          CollisionObject body1,
                          CollisionShape shape0,
                          CollisionShape shape1)
        {
            CollisionShape tmpShape0 = body0.GetCollisionShape();
            CollisionShape tmpShape1 = body1.GetCollisionShape();

            body0.InternalSetTemporaryCollisionShape(shape0);
            body1.InternalSetTemporaryCollisionShape(shape1);
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
            {
                BulletGlobals.g_streamWriter.WriteLine("GImpactAglo::ShapeVsShape");
            }
#endif


            {
                CollisionAlgorithm algor = NewAlgorithm(body0, body1);
                // post :	checkManifold is called

                m_resultOut.SetShapeIdentifiersA(m_part0, m_triface0);
                m_resultOut.SetShapeIdentifiersB(m_part1, m_triface1);

                algor.ProcessCollision(body0, body1, m_dispatchInfo, m_resultOut);

                m_dispatcher.FreeCollisionAlgorithm(algor);
            }

            body0.InternalSetTemporaryCollisionShape(tmpShape0);
            body1.InternalSetTemporaryCollisionShape(tmpShape1);

        }

        protected void ConvexVsConvexCollision(CollisionObject body0,
                          CollisionObject body1,
                          CollisionShape shape0,
                          CollisionShape shape1)
        {
            CollisionShape tmpShape0 = body0.GetCollisionShape();
            CollisionShape tmpShape1 = body1.GetCollisionShape();

            body0.InternalSetTemporaryCollisionShape(shape0);
            body1.InternalSetTemporaryCollisionShape(shape1);
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
            {
                BulletGlobals.g_streamWriter.WriteLine("GImpactAglo::ConvexVsConvex");
            }
#endif

            m_resultOut.SetShapeIdentifiersA(m_part0, m_triface0);
            m_resultOut.SetShapeIdentifiersB(m_part1, m_triface1);

            CheckConvexAlgorithm(body0, body1);
            m_convex_algorithm.ProcessCollision(body0, body1, m_dispatchInfo, m_resultOut);

            body0.InternalSetTemporaryCollisionShape(tmpShape0);
            body1.InternalSetTemporaryCollisionShape(tmpShape1);

        }



        protected void GImpactVsGImpactFindPairs(
                          ref IndexedMatrix trans0,
                          ref IndexedMatrix trans1,
                          GImpactShapeInterface shape0,
                          GImpactShapeInterface shape1, PairSet pairset)
        {
            if (shape0.HasBoxSet() && shape1.HasBoxSet())
            {
                GImpactQuantizedBvh.FindCollision(shape0.GetBoxSet(), ref trans0, shape1.GetBoxSet(), ref trans1, pairset);
            }
            else
            {
                AABB boxshape0 = new AABB();
                AABB boxshape1 = new AABB();
                int i = shape0.GetNumChildShapes();

                while (i-- != 0)
                {
                    shape0.GetChildAabb(i, ref trans0, out boxshape0.m_min, out boxshape0.m_max);

                    int j = shape1.GetNumChildShapes();
                    while (j-- != 0)
                    {
                        shape1.GetChildAabb(i, ref trans1, out boxshape1.m_min, out boxshape1.m_max);

                        if (boxshape1.HasCollision(ref boxshape0))
                        {
                            pairset.PushPair(i, j);
                        }
                    }
                }
            }
#if DEBUG            
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
            {
                BulletGlobals.g_streamWriter.WriteLine("GImpactAglo::GImpactVsGImpactFindPairs [{0}]", pairset.Count);
            }
#endif            
        }

        protected void GImpactVsShapeFindPairs(
                          ref IndexedMatrix trans0,
                          ref IndexedMatrix trans1,
                          GImpactShapeInterface shape0,
                          CollisionShape shape1,
                          ObjectArray<int> collided_primitives)
        {
            AABB boxshape = new AABB();


            if (shape0.HasBoxSet())
            {
                IndexedMatrix trans1to0 = trans0.Inverse();
                //trans1to0 *= trans1;
                trans1to0 = trans1to0 * trans1;
                //trans1to0 = MathUtil.BulletMatrixMultiply(trans1,trans1to0);
                shape1.GetAabb(ref trans1to0, out boxshape.m_min, out boxshape.m_max);
#if DEBUG                
                if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
                {
                    MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "GImpactAglo::GImpactVsShapeFindPairs trans1to0", trans1to0);
                    MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "box min", boxshape.m_min);
                    MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "box max", boxshape.m_max);
                }
#endif                
                shape0.GetBoxSet().BoxQuery(ref boxshape, collided_primitives);
            }
            else
            {
                shape1.GetAabb(ref trans1, out boxshape.m_min, out boxshape.m_max);

                AABB boxshape0 = new AABB();
                int i = shape0.GetNumChildShapes();

                while (i-- != 0)
                {
                    shape0.GetChildAabb(i, ref trans0, out boxshape0.m_min, out boxshape0.m_max);

                    if (boxshape.HasCollision(ref boxshape0))
                    {
                        collided_primitives.Add(i);
                    }
                }

            }


        }


        protected void GImpactTrimeshpartVsPlaneCollision(
                          CollisionObject body0,
                          CollisionObject body1,
                          GImpactMeshShapePart shape0,
                          StaticPlaneShape shape1, bool swapped)
        {
            IndexedMatrix orgtrans0 = body0.GetWorldTransform();
            IndexedMatrix orgtrans1 = body1.GetWorldTransform();

            IndexedVector4 plane;
            PlaneShape.GetPlaneEquationTransformed(shape1,ref orgtrans1, out plane);

            //test box against plane

            AABB tribox = new AABB();
            shape0.GetAabb(ref orgtrans0, out tribox.m_min, out tribox.m_max);
            tribox.IncrementMargin(shape1.GetMargin());

            if (tribox.PlaneClassify(ref plane) != BT_PLANE_INTERSECTION_TYPE.BT_CONST_COLLIDE_PLANE) return;

            shape0.LockChildShapes();

            float margin = shape0.GetMargin() + shape1.GetMargin();

            IndexedVector3 vertex;
            int vi = shape0.GetVertexCount();
            while (vi-- != 0)
            {
                shape0.GetVertex(vi, out vertex);
                vertex = orgtrans0 * vertex;

                float distance = IndexedVector3.Dot(vertex, MathUtil.Vector4ToVector3(ref plane)) - plane.W - margin;

                if (distance < 0.0f)//add contact
                {
                    if (swapped)
                    {
                        AddContactPoint(body1, body0,
                            vertex,
                            MathUtil.Vector4ToVector3(-plane),
                            distance);
                    }
                    else
                    {
                        AddContactPoint(body0, body1,
                            vertex,
                            MathUtil.Vector4ToVector3(ref plane),
                            distance);
                    }
                }
            }

            shape0.UnlockChildShapes();

        }



        public GImpactCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
            : base(ci, body0, body1)
        {
        }

        public override void Initialize(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            base.Initialize(ci, body0, body1);
        }

        public override void Cleanup()
        {
            BulletGlobals.GImpactCollisionAlgorithmPool.Free(this);
        }

        public override void ProcessCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            ClearCache();
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
            {
                BulletGlobals.g_streamWriter.WriteLine("GImpactAglo::ProcessCollision");
            }
#endif

            m_resultOut = resultOut;
            m_dispatchInfo = dispatchInfo;
            GImpactShapeInterface gimpactshape0;
            GImpactShapeInterface gimpactshape1;

            if (body0.GetCollisionShape().GetShapeType() == BroadphaseNativeTypes.GIMPACT_SHAPE_PROXYTYPE)
            {
                gimpactshape0 = body0.GetCollisionShape() as GImpactShapeInterface;

                if (body1.GetCollisionShape().GetShapeType() == BroadphaseNativeTypes.GIMPACT_SHAPE_PROXYTYPE)
                {
                    gimpactshape1 = body1.GetCollisionShape() as GImpactShapeInterface;

                    GImpactVsGImpact(body0, body1, gimpactshape0, gimpactshape1);
                }
                else
                {
                    GImpactVsShape(body0, body1, gimpactshape0, body1.GetCollisionShape(), false);
                }

            }
            else if (body1.GetCollisionShape().GetShapeType() == BroadphaseNativeTypes.GIMPACT_SHAPE_PROXYTYPE)
            {
                gimpactshape1 = body1.GetCollisionShape() as GImpactShapeInterface;

                GImpactVsShape(body1, body0, gimpactshape1, body0.GetCollisionShape(), true);
            }

        }

        public override float CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            return 1.0f;
        }

        public override void GetAllContactManifolds(PersistentManifoldArray manifoldArray)
        {
            if (m_manifoldPtr != null)
            {
                manifoldArray.Add(m_manifoldPtr);
            }
        }


        //struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
        //{
        //    virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* body0,btCollisionObject* body1)
        //    {
        //        void* mem = ci.m_dispatcher1.allocateCollisionAlgorithm(sizeof(btGImpactCollisionAlgorithm));
        //        return new(mem) btGImpactCollisionAlgorithm(ci,body0,body1);
        //    }
        //};

        //! Use this function for register the algorithm externally
        public static void RegisterAlgorithm(CollisionDispatcher dispatcher)
        {
            GImpactCreateFunc s_gimpact_cf = new GImpactCreateFunc();

            int i;

            for (i = 0; i < (int)BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES; i++)
            {
                dispatcher.RegisterCollisionCreateFunc((int)BroadphaseNativeTypes.GIMPACT_SHAPE_PROXYTYPE, i, s_gimpact_cf);
            }

            for (i = 0; i < (int)BroadphaseNativeTypes.MAX_BROADPHASE_COLLISION_TYPES; i++)
            {
                dispatcher.RegisterCollisionCreateFunc(i, (int)BroadphaseNativeTypes.GIMPACT_SHAPE_PROXYTYPE, s_gimpact_cf);
            }

        }

        //! Gets the average time in miliseconds of tree collisions
        public static float GetAverageTreeCollisionTime()
        {
            return 1.0f;
        }

        //! Gets the average time in miliseconds of triangle collisions
        public static float GetAverageTriangleCollisionTime()
        {
            return 1.0f;
        }


        //! Collides two gimpact shapes
        /*!
        \pre shape0 and shape1 couldn't be btGImpactMeshShape objects
        */


        public void GImpactVsGImpact(CollisionObject body0,
                          CollisionObject body1,
                          GImpactShapeInterface shape0,
                          GImpactShapeInterface shape1)
        {
            if (shape0.GetGImpactShapeType() == GIMPACT_SHAPE_TYPE.CONST_GIMPACT_TRIMESH_SHAPE)
            {
                GImpactMeshShape meshshape0 = shape0 as GImpactMeshShape;
                m_part0 = meshshape0.GetMeshPartCount();

                while (m_part0-- != 0)
                {
                    GImpactVsGImpact(body0, body1, meshshape0.GetMeshPart(m_part0), shape1);
                }

                return;
            }

            if (shape1.GetGImpactShapeType() == GIMPACT_SHAPE_TYPE.CONST_GIMPACT_TRIMESH_SHAPE)
            {
                GImpactMeshShape meshshape1 = shape1 as GImpactMeshShape;
                m_part1 = meshshape1.GetMeshPartCount();

                while (m_part1-- != 0)
                {

                    GImpactVsGImpact(body0, body1, shape0, meshshape1.GetMeshPart(m_part1));

                }

                return;
            }


            IndexedMatrix orgtrans0 = body0.GetWorldTransform();
            IndexedMatrix orgtrans1 = body1.GetWorldTransform();

            PairSet pairset = new PairSet();

            GImpactVsGImpactFindPairs(ref orgtrans0, ref orgtrans1, shape0, shape1, pairset);

            if (pairset.Count == 0) return;

#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
            {
                BulletGlobals.g_streamWriter.WriteLine("GImpactAglo::GImpactVsGImpact [{0}]",pairset.Count);
            }
#endif
            if (shape0.GetGImpactShapeType() == GIMPACT_SHAPE_TYPE.CONST_GIMPACT_TRIMESH_SHAPE_PART &&
                shape1.GetGImpactShapeType() == GIMPACT_SHAPE_TYPE.CONST_GIMPACT_TRIMESH_SHAPE_PART)
            {
                GImpactMeshShapePart shapepart0 = shape0 as GImpactMeshShapePart;
                GImpactMeshShapePart shapepart1 = shape1 as GImpactMeshShapePart;
                //specialized function
#if BULLET_TRIANGLE_COLLISION
    CollideGjkTriangles(body0,body1,shapepart0,shapepart1,&pairset[0].m_index1,pairset.size());
#else
                CollideSatTriangles(body0, body1, shapepart0, shapepart1, pairset, pairset.Count);
#endif

                return;
            }

            //general function

            shape0.LockChildShapes();
            shape1.LockChildShapes();

            using(GIM_ShapeRetriever retriever0 = BulletGlobals.GIM_ShapeRetrieverPool.Get())
            using (GIM_ShapeRetriever retriever1 = BulletGlobals.GIM_ShapeRetrieverPool.Get())
            {
                retriever0.Initialize(shape0);
                retriever1.Initialize(shape1);

                bool child_has_transform0 = shape0.ChildrenHasTransform();
                bool child_has_transform1 = shape1.ChildrenHasTransform();

                int i = pairset.Count;
                while (i-- != 0)
                {
                    GIM_PAIR pair = pairset[i];
                    m_triface0 = pair.m_index1;
                    m_triface1 = pair.m_index2;
                    CollisionShape colshape0 = retriever0.GetChildShape(m_triface0);
                    CollisionShape colshape1 = retriever1.GetChildShape(m_triface1);

                    if (child_has_transform0)
                    {
                        body0.SetWorldTransform(orgtrans0 * shape0.GetChildTransform(m_triface0));
                    }

                    if (child_has_transform1)
                    {
                        body1.SetWorldTransform(orgtrans1 * shape1.GetChildTransform(m_triface1));
                    }

                    //collide two convex shapes
                    ConvexVsConvexCollision(body0, body1, colshape0, colshape1);


                    if (child_has_transform0)
                    {
                        body0.SetWorldTransform(ref orgtrans0);
                    }

                    if (child_has_transform1)
                    {
                        body1.SetWorldTransform(ref orgtrans1);
                    }

                }

                shape0.UnlockChildShapes();
                shape1.UnlockChildShapes();
            }
        }

        public void GImpactVsShape(CollisionObject body0,
                          CollisionObject body1,
                          GImpactShapeInterface shape0,
                          CollisionShape shape1, bool swapped)
        {
#if DEBUG        
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
	        {
		        BulletGlobals.g_streamWriter.WriteLine("GImpactAglo::GImpactVsShape");
	        }
#endif

            if (shape0.GetGImpactShapeType() == GIMPACT_SHAPE_TYPE.CONST_GIMPACT_TRIMESH_SHAPE)
            {
                GImpactMeshShape meshshape0 = shape0 as GImpactMeshShape;

                // check this...
                //int& part = swapped ? m_part1 : m_part0;
                //part = meshshape0.GetMeshPartCount();
                int part = meshshape0.GetMeshPartCount();

                while (part-- != 0)
                {

                    GImpactVsShape(body0,
                          body1,
                          meshshape0.GetMeshPart(part),
                          shape1, swapped);

                }
                if (swapped)
                {
                    m_part1 = part;
                }
                else
                {
                    m_part0 = part;
                }
                return;
            }

#if GIMPACT_VS_PLANE_COLLISION
	if(shape0.GetGImpactShapeType() == GIMPACT_SHAPE_TYPE.CONST_GIMPACT_TRIMESH_SHAPE_PART &&
		shape1.GetShapeType() == BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE)
	{
		GImpactMeshShapePart shapepart = shape0 as GImpactMeshShapePart;
		StaticPlaneShape planeshape = shape1 as StaticPlaneShape;
        GImpactTrimeshpartVsPlaneCollision(body0, body1, shapepart, planeshape, swapped);
		return;
	}

#endif



            if (shape1.IsCompound())
            {
                CompoundShape compoundshape = shape1 as CompoundShape;
                GImpactVsCompoundshape(body0, body1, shape0, compoundshape, swapped);
                return;
            }
            else if (shape1.IsConcave())
            {
                ConcaveShape concaveshape = shape1 as ConcaveShape;
                GImpactVsConcave(body0, body1, shape0, concaveshape, swapped);
                return;
            }


            IndexedMatrix orgtrans0 = body0.GetWorldTransform();

            IndexedMatrix orgtrans1 = body1.GetWorldTransform();

            ObjectArray<int> collided_results = new ObjectArray<int>(64);

            GImpactVsShapeFindPairs(ref orgtrans0, ref orgtrans1, shape0, shape1, collided_results);

            if (collided_results.Count == 0) return;


            shape0.LockChildShapes();

            using (GIM_ShapeRetriever retriever0 = BulletGlobals.GIM_ShapeRetrieverPool.Get())
            {
                retriever0.Initialize(shape0);
                bool child_has_transform0 = shape0.ChildrenHasTransform();


                int i = collided_results.Count;
#if DEBUG
                if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
                {
                    BulletGlobals.g_streamWriter.WriteLine("GImpactAglo::GImpactVsShape [{0}]", collided_results.Count);
                }
#endif

                while (i-- != 0)
                {
                    int child_index = collided_results[i];
                    if (swapped)
                        m_triface1 = child_index;
                    else
                        m_triface0 = child_index;

                    CollisionShape colshape0 = retriever0.GetChildShape(child_index);

                    if (child_has_transform0)
                    {
                        body0.SetWorldTransform(orgtrans0 * shape0.GetChildTransform(child_index));
                    }

                    //collide two shapes
                    if (swapped)
                    {
                        ShapeVsShapeCollision(body1, body0, shape1, colshape0);
                    }
                    else
                    {
                        ShapeVsShapeCollision(body0, body1, colshape0, shape1);
                    }

                    //restore transforms
                    if (child_has_transform0)
                    {
                        body0.SetWorldTransform(ref orgtrans0);
                    }

                }

                shape0.UnlockChildShapes();
            }
        }

        public void GImpactVsCompoundshape(CollisionObject body0,
                          CollisionObject body1,
                          GImpactShapeInterface shape0,
                          CompoundShape shape1, bool swapped)
        {
            IndexedMatrix orgtrans1 = body1.GetWorldTransform();

#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactAlgo)
            {
                BulletGlobals.g_streamWriter.WriteLine("GImpactAglo::GImpactVsCompoundshape");
            }
#endif

            int i = shape1.GetNumChildShapes();
            while (i-- != 0)
            {

                CollisionShape colshape1 = shape1.GetChildShape(i);
                IndexedMatrix childtrans1 = orgtrans1 * shape1.GetChildTransform(i);

                body1.SetWorldTransform(ref childtrans1);

                //collide child shape
                GImpactVsShape(body0, body1,
                              shape0, colshape1, swapped);


                //restore transforms
                body1.SetWorldTransform(ref orgtrans1);
            }

        }

        public void GImpactVsConcave(
                          CollisionObject body0,
                          CollisionObject body1,
                          GImpactShapeInterface shape0,
                          ConcaveShape shape1, bool swapped)
        {
            GImpactTriangleCallback tricallback = new GImpactTriangleCallback();
            tricallback.algorithm = this;
            tricallback.body0 = body0;
            tricallback.body1 = body1;
            tricallback.gimpactshape0 = shape0;
            tricallback.swapped = swapped;
            tricallback.margin = shape1.GetMargin();

            //getting the trimesh AABB
            IndexedMatrix gimpactInConcaveSpace;

            gimpactInConcaveSpace = body1.GetWorldTransform().Inverse() * body0.GetWorldTransform();

            IndexedVector3 minAABB, maxAABB;
            shape0.GetAabb(gimpactInConcaveSpace, out minAABB, out maxAABB);

            shape1.ProcessAllTriangles(tricallback, ref minAABB, ref maxAABB);

        }




        /// Accessor/Mutator pairs for Part and triangleID
        public void SetFace0(int value)
        {
            m_triface0 = value;
        }
        public int GetFace0()
        {
            return m_triface0;
        }
        public void SetFace1(int value)
        {
            m_triface1 = value;
        }
        public int GetFace1()
        {
            return m_triface1;
        }
        public void SetPart0(int value)
        {
            m_part0 = value;
        }
        public int GetPart0()
        {
            return m_part0;
        }
        public void SetPart1(int value)
        {
            m_part1 = value;
        }
        public int GetPart1()
        {
            return m_part1;
        }

    }



    class GImpactTriangleCallback : ITriangleCallback
    {
        public GImpactCollisionAlgorithm algorithm;
        public CollisionObject body0;
        public CollisionObject body1;
        public GImpactShapeInterface gimpactshape0;
        public bool swapped;
        public float margin;

        public virtual void ProcessTriangle(IndexedVector3[] triangle, int partId, int triangleIndex)
        {
            TriangleShapeEx tri1 = new TriangleShapeEx(ref triangle[0], ref triangle[1], ref triangle[2]);
            tri1.SetMargin(margin);
            if (swapped)
            {
                algorithm.SetPart0(partId);
                algorithm.SetFace0(triangleIndex);
            }
            else
            {
                algorithm.SetPart1(partId);
                algorithm.SetFace1(triangleIndex);
            }
            algorithm.GImpactVsShape(body0, body1, gimpactshape0, tri1, swapped);
        }

        public void Cleanup()
        {

        }

        public bool graphics()
        {
            return false;
        }
    }



    public class GIM_ShapeRetriever : IDisposable
    {
        public GImpactShapeInterface m_gim_shape;
        public TriangleShapeEx m_trishape = new TriangleShapeEx();
        public TetrahedronShapeEx m_tetrashape = new TetrahedronShapeEx();

        public class ChildShapeRetriever
        {
            public GIM_ShapeRetriever m_parent;
            public virtual CollisionShape GetChildShape(int index)
            {
                return m_parent.m_gim_shape.GetChildShape(index);
            }
            public virtual void Cleanup()
            {

            }
        }

        public class TriangleShapeRetriever : ChildShapeRetriever
        {

            public override CollisionShape GetChildShape(int index)
            {
                m_parent.m_gim_shape.GetBulletTriangle(index, m_parent.m_trishape);
                return m_parent.m_trishape;
            }
            public override void Cleanup()
            {

            }
        }

        public class TetraShapeRetriever : ChildShapeRetriever
        {
            public override CollisionShape GetChildShape(int index)
            {
                m_parent.m_gim_shape.GetBulletTetrahedron(index, m_parent.m_tetrashape);
                return m_parent.m_tetrashape;
            }
        };
        public ChildShapeRetriever m_child_retriever = new ChildShapeRetriever();
        public TriangleShapeRetriever m_tri_retriever = new TriangleShapeRetriever();
        public TetraShapeRetriever m_tetra_retriever = new TetraShapeRetriever();
        public ChildShapeRetriever m_current_retriever;

        public GIM_ShapeRetriever() { } // for pool
        public GIM_ShapeRetriever(GImpactShapeInterface gim_shape)
        {
            m_gim_shape = gim_shape;
            //select retriever
            if (m_gim_shape.NeedsRetrieveTriangles())
            {
                m_current_retriever = m_tri_retriever;
            }
            else if (m_gim_shape.NeedsRetrieveTetrahedrons())
            {
                m_current_retriever = m_tetra_retriever;
            }
            else
            {
                m_current_retriever = m_child_retriever;
            }

            m_current_retriever.m_parent = this;
        }

        public void Initialize(GImpactShapeInterface gim_shape)
        {
            m_gim_shape = gim_shape;
            //select retriever
            if (m_gim_shape.NeedsRetrieveTriangles())
            {
                m_current_retriever = m_tri_retriever;
            }
            else if (m_gim_shape.NeedsRetrieveTetrahedrons())
            {
                m_current_retriever = m_tetra_retriever;
            }
            else
            {
                m_current_retriever = m_child_retriever;
            }

            m_current_retriever.m_parent = this;
        }



        public CollisionShape GetChildShape(int index)
        {
            return m_current_retriever.GetChildShape(index);
        }

        public void Dispose()
        {
            BulletGlobals.GIM_ShapeRetrieverPool.Free(this);
        }

    }

    public class GImpactCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            GImpactCollisionAlgorithm algo = BulletGlobals.GImpactCollisionAlgorithmPool.Get();
            algo.Initialize(ci, body0, body1);
            return algo;
        }
    }


}
