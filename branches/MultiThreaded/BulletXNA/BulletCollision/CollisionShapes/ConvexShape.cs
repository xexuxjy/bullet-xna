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
using System.Diagnostics;

using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public abstract class ConvexShape : CollisionShape
    {
        public ConvexShape()
        {
        }

        public override void Cleanup()
        {
            base.Cleanup();
        }

		public IndexedVector3	LocalGetSupportingVertex(IndexedVector3 vec)
		{
			return LocalGetSupportingVertex(ref vec);
		}


        public virtual IndexedVector3 LocalGetSupportingVertex(ref IndexedVector3 vec)
        {
            return IndexedVector3.Zero;
        }

        public virtual IndexedVector3 LocalGetSupportingVertexWithoutMargin(ref IndexedVector3 vec)
        {
            return IndexedVector3.Zero;
        }

        public IndexedVector3 LocalGetSupportVertexWithoutMarginNonVirtual(IndexedVector3 localDir)
        {
            return LocalGetSupportVertexWithoutMarginNonVirtual(ref localDir);
        }

        public IndexedVector3 LocalGetSupportVertexWithoutMarginNonVirtual(ref IndexedVector3 localDir)
        {
            IndexedVector3 result = IndexedVector3.Zero;
#if DEBUG            
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConvexShape)
            {
                BulletGlobals.g_streamWriter.WriteLine("localGetSupportVertexWithoutMarginNonVirtual " + GetName());
            }
#endif
            switch (m_shapeType)
            {
                case BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE:
                    {
                        result = new IndexedVector3();
                        break;
                    }
                case BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE:
                    {
                        BoxShape convexShape = this as BoxShape;
                        IndexedVector3 halfExtents = convexShape.GetImplicitShapeDimensions();

                        result = new IndexedVector3(MathUtil.FSel(localDir.X, halfExtents.X, -halfExtents.X),
                            MathUtil.FSel(localDir.Y, halfExtents.Y, -halfExtents.Y),
                            MathUtil.FSel(localDir.Z, halfExtents.Z, -halfExtents.Z));
#if DEBUG                            
						if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConvexShape)
                        {
                            BulletGlobals.g_streamWriter.WriteLine("localGetSupportVertexWithoutMarginNonVirtual::Box");
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "halfEx", halfExtents);
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "localDir", localDir);
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "result", result);
                        }
#endif
                        break;
                    }
                case BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE:
                    {
                        TriangleShape triangleShape = (TriangleShape)this;
                        IndexedVector3 dir = localDir;
                        IndexedVector3[] vertices = triangleShape.m_vertices1;
                        IndexedVector3 dots = new IndexedVector3(IndexedVector3.Dot(ref dir, ref vertices[0]), IndexedVector3.Dot(ref dir, ref vertices[1]), IndexedVector3.Dot(ref dir, ref vertices[2]));
                        int maxAxis = MathUtil.MaxAxis(ref dots);
                        IndexedVector3 sup = vertices[maxAxis];
#if DEBUG                        
						if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConvexShape)
                        {
                            BulletGlobals.g_streamWriter.WriteLine("localGetSupportVertexWithoutMarginNonVirtual::Triangle");
                            BulletGlobals.g_streamWriter.WriteLine(String.Format("MaxAxis [{0}]", maxAxis));
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "vtx0", vertices[0]);
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "vtx1", vertices[1]);
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "vtx2", vertices[2]);
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "dir", dir);
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "dots", dots);
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "sup", sup);

                        }
#endif
                        
                        
                        result = sup;
                        break;
                    }
                case BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE:
                    {
                        CylinderShape cylShape = (CylinderShape)this;
                        //mapping of halfextents/dimension onto radius/height depends on how cylinder local orientation is (upAxis)

                        IndexedVector3 halfExtents = cylShape.GetImplicitShapeDimensions();
                        IndexedVector3 v = localDir;
                        int cylinderUpAxis = cylShape.GetUpAxis();
                        int XX = 1;
                        int YY = 0;
                        int ZZ = 2;

                        switch (cylinderUpAxis)
                        {
                            case 0:
                                {
                                    XX = 1;
                                    YY = 0;
                                    ZZ = 2;
                                }
                                break;
                            case 1:
                                {
                                    XX = 0;
                                    YY = 1;
                                    ZZ = 2;
                                }
                                break;
                            case 2:
                                {
                                    XX = 0;
                                    YY = 2;
                                    ZZ = 1;

                                }
                                break;
                            default:
                                Debug.Assert(false);
                                break;
                        };

                        float radius = halfExtents[XX];
                        float halfHeight = halfExtents[cylinderUpAxis];

                        IndexedVector3 tmp = new IndexedVector3();
                        float d;
                        float vx = v[XX];
                        float vz = v[ZZ];
                        float s = (float)Math.Sqrt(vx * vx + vz * vz);
                        if (s != 0f)
                        {
                            d = radius / s;
                            tmp[XX] = v[XX] * d;
                            tmp[YY] = v[YY] < 0.0f ? -halfHeight : halfHeight;
                            tmp[ZZ] = v[ZZ] * d;
                            result = tmp;
                        }
                        else
                        {
                            tmp[XX] = radius;
                            tmp[YY] = v[YY] < 0.0f ? -halfHeight : halfHeight;
                            tmp[ZZ] = 0.0f;
                            result = tmp;
                        }
                        break;
                    }
                case BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE:
                    {
                        IndexedVector3 vec0 = localDir;

                        CapsuleShape capsuleShape = this as CapsuleShape;
                        float halfHeight = capsuleShape.GetHalfHeight();
                        int capsuleUpAxis = capsuleShape.GetUpAxis();

                        float radius = capsuleShape.GetRadius();
                        IndexedVector3 supVec = new IndexedVector3();

                        float maxDot = float.MinValue;

                        IndexedVector3 vec = vec0;
                        float lenSqr = vec.LengthSquared();
                        if (lenSqr < 0.0001f)
                        {
                            vec = new IndexedVector3(1, 0, 0);
                        }
                        else
                        {
                            float rlen = (1.0f) / (float)Math.Sqrt(lenSqr);
                            vec *= rlen;

                            //vec = IndexedVector3.Normalize(vec);
                        }
                        IndexedVector3 vtx;
                        float newDot;
                        {
                            IndexedVector3 pos = new IndexedVector3();
                            pos[capsuleUpAxis] = halfHeight;

                            //vtx = pos +vec*(radius);
                            vtx = pos + vec * (radius) - vec * capsuleShape.GetMarginNV();
                            newDot = IndexedVector3.Dot(ref vec, ref vtx);

                            if (newDot > maxDot)
                            {
                                maxDot = newDot;
                                supVec = vtx;
                            }
                        }
                        {
                            IndexedVector3 pos = new IndexedVector3();
                            pos[capsuleUpAxis] =  -halfHeight;

                            //vtx = pos +vec*(radius);
                            vtx = pos + vec * (radius) - vec * capsuleShape.GetMarginNV();
                            newDot = IndexedVector3.Dot(ref vec, ref vtx);

                            if (newDot > maxDot)
                            {
                                maxDot = newDot;
                                supVec = vtx;
                            }
                        }
                        result = supVec;
                        break;
                    }
                case BroadphaseNativeTypes.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
                    {
                        ConvexPointCloudShape convexPointCloudShape = (ConvexPointCloudShape)this;
                        IList<IndexedVector3> points = convexPointCloudShape.GetUnscaledPoints();
                        int numPoints = convexPointCloudShape.GetNumPoints();
                        IndexedVector3 localScaling = convexPointCloudShape.GetLocalScalingNV();
                        result = ConvexHullSupport(ref localDir, points, numPoints, ref localScaling);
                        break;
                    }
                case BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE:
                    {
                        ConvexHullShape convexHullShape = (ConvexHullShape)this;
                        IList<IndexedVector3> points = convexHullShape.GetUnscaledPoints();
                        int numPoints = convexHullShape.GetNumPoints();
                        IndexedVector3 localScaling = convexHullShape.GetLocalScalingNV();
                        result = ConvexHullSupport(ref localDir, points, numPoints, ref localScaling);
                        break;
                    }
                default:
                    result = LocalGetSupportingVertexWithoutMargin(ref localDir);
                    break;
            }

            // should never reach here
            //Debug.Assert(false);
#if DEBUG            
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConvexShape)
            {
                BulletGlobals.g_streamWriter.WriteLine("localGetSupportVertexWithoutMarginNonVirtual");
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "localDir", localDir);
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "result", result);
            }
#endif            
            return result;

        }

        public IndexedVector3 LocalGetSupportVertexNonVirtual(ref IndexedVector3 localDir)
        {
            IndexedVector3 localDirNorm = localDir;
            if (localDirNorm.LengthSquared() < (MathUtil.SIMD_EPSILON * MathUtil.SIMD_EPSILON))
            {
                localDirNorm = new IndexedVector3(-1f);
            }
            localDirNorm = IndexedVector3.Normalize(ref localDirNorm);

            return LocalGetSupportVertexWithoutMarginNonVirtual(ref localDirNorm) + GetMarginNonVirtual() * localDirNorm;
        }

        public float GetMarginNonVirtual()
        {
            switch (m_shapeType)
            {
                case BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE:
                    {
                        SphereShape sphereShape = this as SphereShape;
                        return sphereShape.GetRadius();
                    }
                case BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE:
                    {
                        BoxShape convexShape = this as BoxShape;
                        return convexShape.GetMarginNV();
                    }
                case BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE:
                    {
                        TriangleShape triangleShape = this as TriangleShape;
                        return triangleShape.GetMarginNV();
                    }
                case BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE:
                    {
                        CylinderShape cylShape = this as CylinderShape;
                        return cylShape.GetMarginNV();
                    }
                case BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE:
                    {
                        CapsuleShape capsuleShape = this as CapsuleShape;
                        return capsuleShape.GetMarginNV();
                    }
                case BroadphaseNativeTypes.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
                /* fall through */
                case BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE:
                    {
                        PolyhedralConvexShape convexHullShape = this as PolyhedralConvexShape;
                        return convexHullShape.GetMarginNV();
                    }
                default:
                    return this.GetMargin();
            }

            // should never reach here
            Debug.Assert(false);
            return 0.0f;
        }

        public virtual void GetAabbNonVirtual(ref IndexedMatrix t, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
            switch (m_shapeType)
            {
                case BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE:
                    {
                        SphereShape sphereShape = this as SphereShape;
                        float radius = sphereShape.GetImplicitShapeDimensions().X;// * convexShape->getLocalScaling().getX();
                        float margin = radius + sphereShape.GetMarginNonVirtual();
                        IndexedVector3 center = t._origin;
                        IndexedVector3 extent = new IndexedVector3(margin);
                        aabbMin = center - extent;
                        aabbMax = center + extent;
                    }
                    break;
                case BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE:
                /* fall through */
                case BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE:
                    {
                        BoxShape convexShape = this as BoxShape;
                        float margin = convexShape.GetMarginNonVirtual();
                        IndexedVector3 halfExtents = convexShape.GetImplicitShapeDimensions();
                        halfExtents += new IndexedVector3(margin);

                        IndexedBasisMatrix abs_b = t._basis.Absolute();
                        IndexedVector3 center = t._origin;
                        IndexedVector3 extent = new IndexedVector3(abs_b._el0.Dot(ref halfExtents), abs_b._el1.Dot(ref halfExtents), abs_b._el2.Dot(ref halfExtents));

                        aabbMin = center - extent;
                        aabbMax = center + extent;
                        break;
                    }
                case BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE:
                    {
                        TriangleShape triangleShape = (TriangleShape)this;
                        float margin = triangleShape.GetMarginNonVirtual();
                        for (int i = 0; i < 3; i++)
                        {
                            IndexedVector3 vec = new IndexedVector3();
                            vec[i] = 1f;
                            IndexedVector3 sv = LocalGetSupportVertexWithoutMarginNonVirtual(vec * t._basis);
                            IndexedVector3 tmp = t * sv;
			                aabbMax[i] = tmp[i]+margin;
			                vec[i] = -1.0f;

                            tmp = t * (LocalGetSupportVertexWithoutMarginNonVirtual(vec * t._basis));
                            aabbMin[i] = tmp[i] - margin;
                        }
                    }
                    break;
                case BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE:
                    {
                        CapsuleShape capsuleShape = this as CapsuleShape;
                        float r = capsuleShape.GetRadius();
                        IndexedVector3 halfExtents = new IndexedVector3(r);
                        int m_upAxis = capsuleShape.GetUpAxis();
                        halfExtents[m_upAxis] =  r + capsuleShape.GetHalfHeight();
                        float nvMargin = capsuleShape.GetMarginNonVirtual();
                        halfExtents += new IndexedVector3(nvMargin);

                        IndexedBasisMatrix abs_b  = t._basis.Absolute();
                        IndexedVector3 center = t._origin;
                        IndexedVector3 extent = new IndexedVector3(abs_b._el0.Dot(ref halfExtents), abs_b._el1.Dot(ref halfExtents), abs_b._el2.Dot(ref halfExtents));		

                        aabbMin = center - extent;
                        aabbMax = center + extent;
                    }
                    break;
                case BroadphaseNativeTypes.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
                case BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE:
                    {
                        PolyhedralConvexAabbCachingShape convexHullShape = (PolyhedralConvexAabbCachingShape)this;
                        float margin = convexHullShape.GetMarginNonVirtual();
                        convexHullShape.GetNonvirtualAabb(ref t, out aabbMin, out aabbMax, margin);
                    }
                    break;
                default:
                    GetAabb(ref t, out aabbMin, out aabbMax);
                    break;
            }

            // should never reach here
            Debug.Assert(false);

        }

        public virtual void Project(ref IndexedMatrix trans, ref IndexedVector3 dir, ref float min, ref float max)
        {
            IndexedVector3 localAxis = dir * trans._basis;
            IndexedVector3 vtx1 = trans * LocalGetSupportingVertex(localAxis);
            IndexedVector3 vtx2 = trans * LocalGetSupportingVertex(-localAxis);

            min = vtx1.Dot(dir);
            max = vtx2.Dot(dir);

            if (min > max)
            {
                float tmp = min;
                min = max;
                max = tmp;
            }
        }


        //notice that the vectors should be unit length
		public abstract void BatchedUnitVectorGetSupportingVertexWithoutMargin(IndexedVector3[] vectors, IndexedVector4[] supportVerticesOut, int numVectors);


        ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version

        public abstract void GetAabbSlow(ref IndexedMatrix t, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax);

        public abstract int GetNumPreferredPenetrationDirections();

        public abstract void GetPreferredPenetrationDirection(int index, out IndexedVector3 penetrationVector);

        public static IndexedVector3 ConvexHullSupport(ref IndexedVector3 localDirOrg, IList<IndexedVector3> points, int numPoints, ref IndexedVector3 localScaling)
        {
            IndexedVector3 vec = localDirOrg * localScaling;
			float newDot, maxDot = -MathUtil.BT_LARGE_FLOAT;
            int ptIndex = -1;

            for (int i = 0; i < numPoints; i++)
            {

                newDot = IndexedVector3.Dot(vec,points[i]);
                if (newDot > maxDot)
                {
                    maxDot = newDot;
                    ptIndex = i;
                }
            }
            Debug.Assert(ptIndex >= 0);
            IndexedVector3 supVec = points[ptIndex] * localScaling;
            return supVec;
        }

        public const int MAX_PREFERRED_PENETRATION_DIRECTIONS = 10;
    }
}