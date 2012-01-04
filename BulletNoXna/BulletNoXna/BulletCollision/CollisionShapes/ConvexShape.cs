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

		public Vector3	LocalGetSupportingVertex(Vector3 vec)
		{
			return LocalGetSupportingVertex(ref vec);
		}


        public virtual Vector3 LocalGetSupportingVertex(ref Vector3 vec)
        {
            return Vector3.Zero;
        }

        public virtual Vector3 LocalGetSupportingVertexWithoutMargin(ref Vector3 vec)
        {
            return Vector3.Zero;
        }

        public Vector3 LocalGetSupportVertexWithoutMarginNonVirtual(Vector3 localDir)
        {
            return LocalGetSupportVertexWithoutMarginNonVirtual(ref localDir);
        }

        public Vector3 LocalGetSupportVertexWithoutMarginNonVirtual(ref Vector3 localDir)
        {
            Vector3 result = Vector3.Zero;
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConvexShape)
            {
                BulletGlobals.g_streamWriter.WriteLine("localGetSupportVertexWithoutMarginNonVirtual " + GetName());
            }

            switch (m_shapeType)
            {
                case BroadphaseNativeType.SphereShape:
                    {
                        result = new Vector3();
                        break;
                    }
                case BroadphaseNativeType.BoxShape:
                    {
                        BoxShape convexShape = this as BoxShape;
                        Vector3 halfExtents = convexShape.GetImplicitShapeDimensions();

                        result = new Vector3(MathUtil.FSel(localDir.X, halfExtents.X, -halfExtents.X),
                            MathUtil.FSel(localDir.Y, halfExtents.Y, -halfExtents.Y),
                            MathUtil.FSel(localDir.Z, halfExtents.Z, -halfExtents.Z));
						if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConvexShape)
                        {
                            BulletGlobals.g_streamWriter.WriteLine("localGetSupportVertexWithoutMarginNonVirtual::Box");
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "halfEx", halfExtents);
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "localDir", localDir);
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "result", result);
                        }

                        break;
                    }
                case BroadphaseNativeType.TriangleShape:
                    {
                        TriangleShape triangleShape = (TriangleShape)this;
                        Vector3 dir = localDir;
                        Vector3[] vertices = triangleShape.m_vertices1;
                        Vector3 dots = new Vector3(Vector3.Dot(ref dir, ref vertices[0]), Vector3.Dot(ref dir, ref vertices[1]), Vector3.Dot(ref dir, ref vertices[2]));
                        int maxAxis = MathUtil.MaxAxis(ref dots);
                        Vector3 sup = vertices[maxAxis];
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

                        
                        
                        result = sup;
                        break;
                    }
                case BroadphaseNativeType.CylinderShape:
                    {
                        CylinderShape cylShape = (CylinderShape)this;
                        //mapping of halfextents/dimension onto radius/height depends on how cylinder local orientation is (upAxis)

                        Vector3 halfExtents = cylShape.GetImplicitShapeDimensions();
                        Vector3 v = localDir;
                        int cylinderUpAxis = cylShape.UpAxis;
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

                        Vector3 tmp = new Vector3();
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
                case BroadphaseNativeType.CapsuleShape:
                    {
                        Vector3 vec0 = localDir;

                        CapsuleShape capsuleShape = this as CapsuleShape;
                        float halfHeight = capsuleShape.GetHalfHeight();
                        int capsuleUpAxis = capsuleShape.GetUpAxis();

                        float radius = capsuleShape.Radius;
                        Vector3 supVec = new Vector3();

                        float maxDot = float.MinValue;

                        Vector3 vec = vec0;
                        float lenSqr = vec.LengthSquared();
                        if (lenSqr < 0.0001f)
                        {
                            vec = new Vector3(1, 0, 0);
                        }
                        else
                        {
                            float rlen = (1.0f) / (float)Math.Sqrt(lenSqr);
                            vec *= rlen;

                            //vec = Vector3.Normalize(vec);
                        }
                        Vector3 vtx;
                        float newDot;
                        {
                            Vector3 pos = new Vector3();
                            pos[capsuleUpAxis] = halfHeight;

                            //vtx = pos +vec*(radius);
                            vtx = pos + vec * capsuleShape.GetLocalScalingNV() * (radius) - vec * capsuleShape.GetMarginNV();
                            newDot = Vector3.Dot(ref vec, ref vtx);

                            if (newDot > maxDot)
                            {
                                maxDot = newDot;
                                supVec = vtx;
                            }
                        }
                        {
                            Vector3 pos = new Vector3();
                            pos[capsuleUpAxis] =  -halfHeight;

                            //vtx = pos +vec*(radius);
                            vtx = pos + vec * capsuleShape.GetLocalScalingNV() * (radius) - vec * capsuleShape.GetMarginNV();
                            newDot = Vector3.Dot(ref vec, ref vtx);

                            if (newDot > maxDot)
                            {
                                maxDot = newDot;
                                supVec = vtx;
                            }
                        }
                        result = supVec;
                        break;
                    }
                case BroadphaseNativeType.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
                    {
                        ConvexPointCloudShape convexPointCloudShape = (ConvexPointCloudShape)this;
                        IList<Vector3> points = convexPointCloudShape.GetUnscaledPoints();
                        int numPoints = convexPointCloudShape.GetNumPoints();
                        Vector3 localScaling = convexPointCloudShape.GetLocalScalingNV();
                        result = ConvexHullSupport(ref localDir, points, numPoints, ref localScaling);
                        break;
                    }
                case BroadphaseNativeType.ConvexHullShape:
                    {
                        ConvexHullShape convexHullShape = (ConvexHullShape)this;
                        IList<Vector3> points = convexHullShape.GetUnscaledPoints();
                        int numPoints = convexHullShape.GetNumPoints();
                        Vector3 localScaling = convexHullShape.GetLocalScalingNV();
                        result = ConvexHullSupport(ref localDir, points, numPoints, ref localScaling);
                        break;
                    }
                default:
                    result = LocalGetSupportingVertexWithoutMargin(ref localDir);
                    break;
            }

            // should never reach here
            //Debug.Assert(false);
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConvexShape)
            {
                BulletGlobals.g_streamWriter.WriteLine("localGetSupportVertexWithoutMarginNonVirtual");
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "localDir", localDir);
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "result", result);
            }
            return result;

        }

        public Vector3 LocalGetSupportVertexNonVirtual(ref Vector3 localDir)
        {
            Vector3 localDirNorm = localDir;
            if (localDirNorm.LengthSquared() < (MathUtil.SIMD_EPSILON * MathUtil.SIMD_EPSILON))
            {
                localDirNorm = new Vector3(-1f);
            }
            localDirNorm = Vector3.Normalize(ref localDirNorm);

            return LocalGetSupportVertexWithoutMarginNonVirtual(ref localDirNorm) + GetMarginNonVirtual() * localDirNorm;
        }

        public float GetMarginNonVirtual()
        {
            switch (m_shapeType)
            {
                case BroadphaseNativeType.SphereShape:
                    {
                        SphereShape sphereShape = this as SphereShape;
                        return sphereShape.Radius;
                    }
                case BroadphaseNativeType.BoxShape:
                    {
                        BoxShape convexShape = this as BoxShape;
                        return convexShape.GetMarginNV();
                    }
                case BroadphaseNativeType.TriangleShape:
                    {
                        TriangleShape triangleShape = this as TriangleShape;
                        return triangleShape.GetMarginNV();
                    }
                case BroadphaseNativeType.CylinderShape:
                    {
                        CylinderShape cylShape = this as CylinderShape;
                        return cylShape.GetMarginNV();
                    }
                case BroadphaseNativeType.CapsuleShape:
                    {
                        CapsuleShape capsuleShape = this as CapsuleShape;
                        return capsuleShape.GetMarginNV();
                    }
                case BroadphaseNativeType.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
                /* fall through */
                case BroadphaseNativeType.ConvexHullShape:
                    {
                        PolyhedralConvexShape convexHullShape = this as PolyhedralConvexShape;
                        return convexHullShape.GetMarginNV();
                    }
                default:
                    return this.Margin;
            }

            // should never reach here
            Debug.Assert(false);
            return 0.0f;
        }

        public virtual void GetAabbNonVirtual(ref Matrix t, ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
            switch (m_shapeType)
            {
                case BroadphaseNativeType.SphereShape:
                    {
                        SphereShape sphereShape = this as SphereShape;
                        float radius = sphereShape.GetImplicitShapeDimensions().X;// * convexShape->getLocalScaling().getX();
                        float margin = radius + sphereShape.GetMarginNonVirtual();
                        Vector3 center = t.Translation;
                        Vector3 extent = new Vector3(margin);
                        aabbMin = center - extent;
                        aabbMax = center + extent;
                    }
                    break;
                case BroadphaseNativeType.CylinderShape:
                /* fall through */
                case BroadphaseNativeType.BoxShape:
                    {
                        BoxShape convexShape = this as BoxShape;
                        float margin = convexShape.GetMarginNonVirtual();
                        Vector3 halfExtents = convexShape.GetImplicitShapeDimensions();
                        halfExtents += new Vector3(margin);

                        IndexedBasisMatrix abs_b = t._basis.Absolute();
                        Vector3 center = t.Translation;
                        Vector3 extent = new Vector3(abs_b[0].Dot(ref halfExtents), abs_b[1].Dot(ref halfExtents), abs_b[2].Dot(ref halfExtents));

                        aabbMin = center - extent;
                        aabbMax = center + extent;
                        break;
                    }
                case BroadphaseNativeType.TriangleShape:
                    {
                        TriangleShape triangleShape = (TriangleShape)this;
                        float margin = triangleShape.GetMarginNonVirtual();
                        for (int i = 0; i < 3; i++)
                        {
                            Vector3 vec = new Vector3();
                            vec[i] = 1f;
                            Vector3 sv = LocalGetSupportVertexWithoutMarginNonVirtual(vec * t._basis);
                            Vector3 tmp = t * sv;
			                aabbMax[i] = tmp[i]+margin;
			                vec[i] = -1.0f;

                            tmp = t * (LocalGetSupportVertexWithoutMarginNonVirtual(vec * t._basis));
                            aabbMin[i] = tmp[i] - margin;
                        }
                    }
                    break;
                case BroadphaseNativeType.CapsuleShape:
                    {
                        CapsuleShape capsuleShape = this as CapsuleShape;
                        float r = capsuleShape.Radius;
                        Vector3 halfExtents = new Vector3(r);
                        int m_upAxis = capsuleShape.GetUpAxis();
                        halfExtents[m_upAxis] =  r + capsuleShape.GetHalfHeight();
                        float nvMargin = capsuleShape.GetMarginNonVirtual();
                        halfExtents += new Vector3(nvMargin);

                        IndexedBasisMatrix abs_b  = t._basis.Absolute();
                        Vector3 center = t.Translation;
                        Vector3 extent = new Vector3(abs_b[0].Dot(ref halfExtents), abs_b[1].Dot(ref halfExtents), abs_b[2].Dot(ref halfExtents));		

                        aabbMin = center - extent;
                        aabbMax = center + extent;
                    }
                    break;
                case BroadphaseNativeType.CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE:
                case BroadphaseNativeType.ConvexHullShape:
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

        public virtual void Project(ref Matrix trans, ref Vector3 dir, ref float min, ref float max)
        {
            Vector3 localAxis = dir * trans._basis;
            Vector3 vtx1 = trans * LocalGetSupportingVertex(localAxis);
            Vector3 vtx2 = trans * LocalGetSupportingVertex(-localAxis);

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
		public abstract void BatchedUnitVectorGetSupportingVertexWithoutMargin(Vector3[] vectors, Vector4[] supportVerticesOut, int numVectors);


        ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version

        public abstract void GetAabbSlow(ref Matrix t, out Vector3 aabbMin, out Vector3 aabbMax);

        public abstract int GetNumPreferredPenetrationDirections();

        public abstract void GetPreferredPenetrationDirection(int index, out Vector3 penetrationVector);

        public static Vector3 ConvexHullSupport(ref Vector3 localDirOrg, IList<Vector3> points, int numPoints, ref Vector3 localScaling)
        {
            Vector3 vec = localDirOrg * localScaling;
			float newDot, maxDot = -MathUtil.BT_LARGE_FLOAT;
            int ptIndex = -1;

            for (int i = 0; i < numPoints; i++)
            {

                newDot = Vector3.Dot(vec,points[i]);
                if (newDot > maxDot)
                {
                    maxDot = newDot;
                    ptIndex = i;
                }
            }
            Debug.Assert(ptIndex >= 0);
            Vector3 supVec = points[ptIndex] * localScaling;
            return supVec;
        }

        public const int MAX_PREFERRED_PENETRATION_DIRECTIONS = 10;
    }
}