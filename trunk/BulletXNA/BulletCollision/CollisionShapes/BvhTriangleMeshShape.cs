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

//#define DISABLE_BVH
using System;
using System.Diagnostics;
using BulletXNA.LinearMath;


namespace BulletXNA.BulletCollision
{
    public class BvhTriangleMeshShape : TriangleMeshShape
    {
        public BvhTriangleMeshShape()
            : base(null)
        {
            m_bvh = null;
            m_ownsBvh = false;
            m_shapeType = BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE;
            m_triangleInfoMap = null;
        }

        public BvhTriangleMeshShape(StridingMeshInterface meshInterface, bool useQuantizedAabbCompression, bool buildBvh)
            : base(meshInterface)
        {
            m_bvh = null;
            m_ownsBvh = false;
            m_useQuantizedAabbCompression = useQuantizedAabbCompression;
            m_shapeType = BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE;
            //construct bvh from meshInterface
#if !DISABLE_BVH

            //IndexedVector3 bvhAabbMin = IndexedVector3.Zero, bvhAabbMax = IndexedVector3.Zero;
            //if (meshInterface.hasPremadeAabb())
            //{
            //    meshInterface.getPremadeAabb(ref bvhAabbMin, ref bvhAabbMax);
            //}
            //else
            //{
            //    meshInterface.calculateAabbBruteForce(ref bvhAabbMin, ref bvhAabbMax);
            //}
            if (buildBvh)
            {
                BuildOptimizedBvh();
            }
#endif //DISABLE_BVH
        }

        private void BuildOptimizedBvh()
        {
            if (m_ownsBvh)
            {
                m_bvh.Cleanup();
                m_bvh = null;
            }
            ///m_localAabbMin/m_localAabbMax is already re-calculated in btTriangleMeshShape. We could just scale aabb, but this needs some more work
            //void* mem = btAlignedAlloc(sizeof(btOptimizedBvh),16);
            m_bvh = new OptimizedBvh();
            //rebuild the bvh...
            m_bvh.Build(m_meshInterface, m_useQuantizedAabbCompression, ref m_localAabbMin, ref m_localAabbMax);
            m_ownsBvh = true;
        }



        ///optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb
        public BvhTriangleMeshShape(StridingMeshInterface meshInterface, bool useQuantizedAabbCompression, ref IndexedVector3 bvhAabbMin, ref IndexedVector3 bvhAabbMax, bool buildBvh)
            : base(meshInterface)
        {
            m_bvh = null;
            m_ownsBvh = false;
            m_useQuantizedAabbCompression = useQuantizedAabbCompression;
            m_shapeType = BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE;
#if !DISABLE_BVH

            if (buildBvh)
            {
                m_bvh = new OptimizedBvh();

                m_bvh.Build(meshInterface, m_useQuantizedAabbCompression, ref bvhAabbMin, ref bvhAabbMax);
                m_ownsBvh = true;
            }

#endif //DISABLE_BVH

        }

        public bool GetOwnsBvh()
        {
            return m_ownsBvh;
        }

        public void PerformRaycast(ITriangleCallback callback, ref IndexedVector3 raySource, ref IndexedVector3 rayTarget)
        {
            if (m_bvh != null)
            {
                using (MyNodeOverlapCallback myNodeCallback = BulletGlobals.MyNodeOverlapCallbackPool.Get())
                {
                    myNodeCallback.Initialize(callback, m_meshInterface);
                    m_bvh.ReportRayOverlappingNodex(myNodeCallback, ref raySource, ref rayTarget);
                }
            }
        }


        public void PerformConvexCast(ITriangleCallback callback, ref IndexedVector3 boxSource, ref IndexedVector3 boxTarget, ref IndexedVector3 boxMin, ref IndexedVector3 boxMax)
        {
            if (m_bvh != null)
            {
                using (MyNodeOverlapCallback myNodeCallback = BulletGlobals.MyNodeOverlapCallbackPool.Get())
                {
                    myNodeCallback.Initialize(callback, m_meshInterface);
                    m_bvh.ReportBoxCastOverlappingNodex(myNodeCallback, ref boxSource, ref boxTarget, ref boxMin, ref boxMax);
                }
            }
        }

        //public override void processAllTriangles(ITriangleCallback callback, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        public override void ProcessAllTriangles(ITriangleCallback callback, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
#if DISABLE_BVH
	        base.ProcessAllTriangles(callback,ref aabbMin,ref aabbMax);
#else
            if (m_bvh != null)
            {
                using (MyNodeOverlapCallback myNodeCallback = BulletGlobals.MyNodeOverlapCallbackPool.Get())
                {
                    myNodeCallback.Initialize(callback, m_meshInterface);
                    m_bvh.ReportAabbOverlappingNodex(myNodeCallback, ref aabbMin, ref aabbMax);
                }
            }
#endif
        }

        public void RefitTree(IndexedVector3 aabbMin, IndexedVector3 aabbMax)
        {
            RefitTree(ref aabbMin, ref aabbMax);
        }


        public void RefitTree(ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
            if (m_bvh != null)
            {
                m_bvh.Refit(m_meshInterface, ref aabbMin, ref aabbMax);
                RecalcLocalAabb();
            }
        }

        ///for a fast incremental refit of parts of the tree. Note: the entire AABB of the tree will become more conservative, it never shrinks
        public void PartialRefitTree(ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
            if (m_bvh != null)
            {
                m_bvh.RefitPartial(m_meshInterface, ref aabbMin, ref aabbMax);

                MathUtil.VectorMin(ref aabbMin, ref m_localAabbMin);
                MathUtil.VectorMax(ref aabbMax, ref m_localAabbMax);
            }
        }

        //debugging
        public override String GetName()
        {
            return "BVHTRIANGLEMESH";
        }


        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {
            if ((GetLocalScaling() - scaling).LengthSquared() > MathUtil.SIMD_EPSILON)
            {
                base.SetLocalScaling(ref scaling);
                BuildOptimizedBvh();
            }
        }

        public OptimizedBvh GetOptimizedBvh()
        {
            return m_bvh;
        }


        public void SetOptimizedBvh(OptimizedBvh bvh, ref IndexedVector3 scaling)
        {
            Debug.Assert(m_bvh == null);
            Debug.Assert(m_ownsBvh == false);

            m_bvh = bvh;
            m_ownsBvh = false;
            // update the scaling without rebuilding the bvh
            if ((GetLocalScaling() - scaling).LengthSquared() > MathUtil.SIMD_EPSILON)
            {
                base.SetLocalScaling(ref scaling);
            }
        }

        public bool UsesQuantizedAabbCompression()
        {
            return m_useQuantizedAabbCompression;
        }

        public void SetTriangleInfoMap(TriangleInfoMap triangleInfoMap)
        {
            m_triangleInfoMap = triangleInfoMap;
        }

        public TriangleInfoMap GetTriangleInfoMap()
        {
            return m_triangleInfoMap;
        }

        private OptimizedBvh m_bvh;
        private bool m_useQuantizedAabbCompression;
        private bool m_ownsBvh;
        private TriangleInfoMap m_triangleInfoMap;

    }
    public class MyNodeOverlapCallback : INodeOverlapCallback, IDisposable
    {
        public StridingMeshInterface m_meshInterface;
        public ITriangleCallback m_callback;
        IndexedVector3[] m_triangle = new IndexedVector3[3];

        public MyNodeOverlapCallback() { } // for pool
        public MyNodeOverlapCallback(ITriangleCallback callback, StridingMeshInterface meshInterface)
        {
            m_meshInterface = meshInterface;
            m_callback = callback;

        }

        public void Initialize(ITriangleCallback callback, StridingMeshInterface meshInterface)
        {
            m_meshInterface = meshInterface;
            m_callback = callback;
        }

        public virtual void ProcessNode(int nodeSubPart, int nodeTriangleIndex)
        {
            //m_triangle.Clear();            
            Object vertexBase;
            int numVerts;
            PHY_ScalarType type;
            int stride;
            Object indexBase;
            int indexStride;
            int numfaces;
            PHY_ScalarType indicesType;

            m_meshInterface.GetLockedReadOnlyVertexIndexBase(
                out vertexBase,
                out numVerts,
                out type,
                out stride,
                out indexBase,
                out indexStride,
                out numfaces,
                out indicesType,
                nodeSubPart);

            //unsigned int* gfxbase = (unsigned int*)(indexbase+nodeTriangleIndex*indexstride);
            // force index stride to be 1 regardless.
            indexStride = 3;
            int indexIndex = nodeTriangleIndex * indexStride;

            Debug.Assert(indicesType == PHY_ScalarType.PHY_INTEGER || indicesType == PHY_ScalarType.PHY_SHORT);

            IndexedVector3 meshScaling = m_meshInterface.GetScaling();
            int[] indexRaw = ((ObjectArray<int>)indexBase).GetRawArray();


            if (vertexBase is ObjectArray<IndexedVector3>)
            {
                IndexedVector3[] vertexBaseRaw = ((ObjectArray<IndexedVector3>)vertexBase).GetRawArray();
                for (int j = 2; j >= 0; j--)
                {
                    m_triangle[j] = vertexBaseRaw[indexRaw[indexIndex + j]];
                    m_triangle[j] *= meshScaling;
                }
            }
#if XNA
            else if (vertexBase is ObjectArray<Microsoft.Xna.Framework.Vector3>)
            {
                Microsoft.Xna.Framework.Vector3[] vertexBaseRaw = ((ObjectArray<Microsoft.Xna.Framework.Vector3>)vertexBase).GetRawArray();
                for (int j = 2; j >= 0; j--)
                {
                    m_triangle[j] = new IndexedVector3(vertexBaseRaw[indexRaw[indexIndex + j]]);
                    m_triangle[j] *= meshScaling;
                }
            }
#endif
            else if (vertexBase is ObjectArray<float>)
            {
                float[] floats = ((ObjectArray<float>)vertexBase).GetRawArray();
                for (int j = 2; j >= 0; j--)
                {
                    int offset = indexRaw[indexIndex + j] * 3;
                    m_triangle[j] = new IndexedVector3(floats[offset] * meshScaling.X, floats[offset + 1] * meshScaling.Y, floats[offset + 2] * meshScaling.Z);
                }
            }
            else
            {

                Debug.Assert(false, "Unsupported type.");
            }


            //FIXME - Debug here and on quantized Bvh walking
#if DEBUG            
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugBVHTriangleMesh)
            {
                BulletGlobals.g_streamWriter.WriteLine("BVH Triangle");
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, m_triangle[0]);
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, m_triangle[1]);
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, m_triangle[2]);
            }
#endif            
            /* Perform ray vs. triangle collision here */
            m_callback.ProcessTriangle(m_triangle, nodeSubPart, nodeTriangleIndex);
            m_meshInterface.UnLockReadOnlyVertexBase(nodeSubPart);
        }

        public virtual void Cleanup()
        {
        }

        public void Dispose()
        {
            Cleanup();
            BulletGlobals.MyNodeOverlapCallbackPool.Free(this);
        }


    }

}
