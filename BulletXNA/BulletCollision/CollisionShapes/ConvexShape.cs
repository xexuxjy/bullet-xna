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
using BulletXNA.BulletCollision.BroadphaseCollision;
using Microsoft.Xna.Framework;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision.CollisionShapes
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

        public Vector3 LocalGetSupportVertexWithoutMarginNonVirtual(ref Vector3 localDir)
        {
            Vector3 result = Vector3.Zero;
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugConvexShape)
            {
                BulletGlobals.g_streamWriter.WriteLine("localGetSupportVertexWithoutMarginNonVirtual " + GetName());
            }

            switch (m_shapeType)
            {
                case BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE:
                    {
                        result = new Vector3();
                        break;
                    }
                case BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE:
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
                case BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE:
                    {
                        TriangleShape triangleShape = (TriangleShape)this;
                        Vector3 dir = localDir;
                        IList<Vector3> vertices = triangleShape.m_vertices1;
                        Vector3 dots = new Vector3(Vector3.Dot(dir, vertices[0]), Vector3.Dot(dir, vertices[1]), Vector3.Dot(dir, vertices[2]));
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
                case BroadphaseNativeTypes.CYLINDER_SHAPE_PROXYTYPE:
                    {
                        CylinderShape cylShape = (CylinderShape)this;
                        //mapping of halfextents/dimension onto radius/height depends on how cylinder local orientation is (upAxis)

                        Vector3 halfExtents = cylShape.GetImplicitShapeDimensions();
                        Vector3 v = localDir;
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

                        float radius = MathUtil.VectorComponent(ref halfExtents, XX);
                        float halfHeight = MathUtil.VectorComponent(ref halfExtents, cylinderUpAxis);

                        Vector3 tmp = new Vector3();
                        float d;
                        float vx = MathUtil.VectorComponent(ref v, XX);
                        float vz = MathUtil.VectorComponent(ref v, ZZ);
                        float s = (float)Math.Sqrt(vx * vx + vz * vz);
                        if (s != 0f)
                        {
                            d = radius / s;
                            MathUtil.VectorComponent(ref tmp, XX, (MathUtil.VectorComponent(ref v, XX) * d));
                            MathUtil.VectorComponent(ref tmp, YY, (MathUtil.VectorComponent(ref v, YY) < 0.0 ? -halfHeight : halfHeight));
                            MathUtil.VectorComponent(ref tmp, ZZ, (MathUtil.VectorComponent(ref v, ZZ) * d));
                            result = tmp;
                        }
                        else
                        {
                            MathUtil.VectorComponent(ref tmp, XX, radius);
                            MathUtil.VectorComponent(ref tmp, YY, (MathUtil.VectorComponent(ref v, YY) < 0.0 ? -halfHeight : halfHeight));
                            MathUtil.VectorComponent(ref tmp, XX, 0.0f);
                            result = tmp;
                        }
                        break;
                    }
                case BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE:
                    {
                        Vector3 vec0 = localDir;

                        CapsuleShape capsuleShape = this as CapsuleShape;
                        float halfHeight = capsuleShape.getHalfHeight();
                        int capsuleUpAxis = capsuleShape.GetUpAxis();

                        float radius = capsuleShape.getRadius();
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
                            MathUtil.VectorComponent(ref pos, capsuleUpAxis, halfHeight);

                            //vtx = pos +vec*(radius);
                            vtx = pos + vec * capsuleShape.GetLocalScalingNV() * (radius) - vec * capsuleShape.GetMarginNV();
                            newDot = Vector3.Dot(vec, vtx);

                            if (newDot > maxDot)
                            {
                                maxDot = newDot;
                                supVec = vtx;
                            }
                        }
                        {
                            Vector3 pos = new Vector3();
                            MathUtil.VectorComponent(ref pos, capsuleUpAxis, -halfHeight);

                            //vtx = pos +vec*(radius);
                            vtx = pos + vec * capsuleShape.GetLocalScalingNV() * (radius) - vec * capsuleShape.GetMarginNV();
                            newDot = Vector3.Dot(vec, vtx);

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
                        IList<Vector3> points = convexPointCloudShape.GetUnscaledPoints();
                        int numPoints = convexPointCloudShape.GetNumPoints();
                        Vector3 localScaling = convexPointCloudShape.GetLocalScalingNV();
                        result = ConvexHullSupport(ref localDir, points, numPoints, ref localScaling);
                        break;
                    }
                case BroadphaseNativeTypes.CONVEX_HULL_SHAPE_PROXYTYPE:
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
                localDirNorm = new Vector3(-1f, -1f, -1f);
            }
            localDirNorm = Vector3.Normalize(localDirNorm);

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

        public virtual void GetAabbNonVirtual(ref Matrix t, ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
            switch (m_shapeType)
            {
                case BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE:
                    {
                        SphereShape sphereShape = this as SphereShape;
                        float radius = sphereShape.GetImplicitShapeDimensions().X;// * convexShape->getLocalScaling().getX();
                        float margin = radius + sphereShape.GetMarginNonVirtual();
                        Vector3 center = t.Translation;
                        Vector3 extent = new Vector3(margin, margin, margin);
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
                        Vector3 halfExtents = convexShape.GetImplicitShapeDimensions();
                        halfExtents += new Vector3(margin, margin, margin);
                        Matrix abs_b;
                        MathUtil.AbsoluteMatrix(ref t, out abs_b);
                        Vector3 center = t.Translation;
                        Vector3 extent = new Vector3(Vector3.Dot(abs_b.Right, halfExtents), Vector3.Dot(abs_b.Up, halfExtents), Vector3.Dot(abs_b.Backward, halfExtents));

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
                            Vector3 vec = new Vector3();
                            MathUtil.VectorComponent(ref vec, i, 1f);

							Vector3 temp = MathUtil.TransposeTransformNormal(vec, t); 
                            Vector3 sv = LocalGetSupportVertexWithoutMarginNonVirtual(ref temp);
                            Vector3 tmp = Vector3.Transform(sv,t);
                            MathUtil.VectorComponent(ref aabbMax, i, (MathUtil.VectorComponent(ref tmp, i) + margin));
                            MathUtil.VectorComponent(ref vec, i, -1f);
							temp = MathUtil.TransposeTransformNormal(vec, t);
                            tmp = Vector3.Transform(LocalGetSupportVertexWithoutMarginNonVirtual(ref temp),t);
                            MathUtil.VectorComponent(ref aabbMin, i, (MathUtil.VectorComponent(ref tmp, i) - margin));
                        }
                    }
                    break;
                case BroadphaseNativeTypes.CAPSULE_SHAPE_PROXYTYPE:
                    {
                        CapsuleShape capsuleShape = this as CapsuleShape;
                        float r = capsuleShape.getRadius();
                        Vector3 halfExtents = new Vector3(r, r, r);
                        int m_upAxis = capsuleShape.GetUpAxis();
                        MathUtil.VectorComponent(ref halfExtents, m_upAxis, r + capsuleShape.getHalfHeight());
                        float nvMargin = capsuleShape.GetMarginNonVirtual();
                        halfExtents += new Vector3(nvMargin);

                        Matrix abs_b;
                        MathUtil.AbsoluteMatrix(ref t, out abs_b);
                        Vector3 center = t.Translation;
                        Vector3 extent = new Vector3(Vector3.Dot(abs_b.Right, halfExtents), Vector3.Dot(abs_b.Up, halfExtents), Vector3.Dot(abs_b.Backward, halfExtents));

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