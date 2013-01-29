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
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    [Flags]
    public enum EFlags
    {
        kF_None = 0,
        kF_FilterBackfaces = 1 << 0,
        kF_KeepUnflippedNormal = 1 << 1,   // Prevents returned face normal getting flipped when a ray hits a back-facing triangle
        kF_Terminator = (int)0xFFFFFFF
    }

    public abstract class TriangleRaycastCallback : ITriangleCallback
    {
        public TriangleRaycastCallback() { } // for pool
        public TriangleRaycastCallback(ref IndexedVector3 from, ref IndexedVector3 to, EFlags flags)
        {
            m_from = from;
            m_to = to;
            m_flags = flags;
            m_hitFraction = 1f;
        }

        public virtual void Initialize(ref IndexedVector3 from, ref IndexedVector3 to, EFlags flags)
        {
            m_from = from;
            m_to = to;
            m_flags = flags;
            m_hitFraction = 1f;

        }

        public virtual bool graphics()
        {
            return false;
        }

        public virtual void ProcessTriangle(IndexedVector3[] triangle, int partId, int triangleIndex)
        {
            IndexedVector3 v10 = triangle[1] - triangle[0];
            IndexedVector3 v20 = triangle[2] - triangle[0];


            IndexedVector3 triangleNormal = v10.Cross(ref v20);

            float dist = IndexedVector3.Dot(ref triangle[0], ref triangleNormal);
            float dist_a = IndexedVector3.Dot(ref triangleNormal, ref m_from);
            dist_a -= dist;
            float dist_b = IndexedVector3.Dot(ref triangleNormal, ref m_to);
            dist_b -= dist;

            if (dist_a * dist_b >= 0f)
            {
                return; // same sign
            }
            //@BP Mod - Backface filtering
            if (((m_flags & EFlags.kF_FilterBackfaces) != 0) && (dist_a > 0f))
            {
                // Backface, skip check
                return;
            }

            float proj_length = dist_a - dist_b;
            float distance = (dist_a) / (proj_length);
            // Now we have the intersection point on the plane, we'll see if it's inside the triangle
            // Add an epsilon as a tolerance for the raycast,
            // in case the ray hits exacly on the edge of the triangle.
            // It must be scaled for the triangle size.

            if (distance < m_hitFraction)
            {
                float edge_tolerance = triangleNormal.LengthSquared();
                edge_tolerance *= -0.0001f;
                IndexedVector3 point;
                point = MathUtil.Interpolate3(ref m_from, ref m_to, distance);
                {
                    IndexedVector3 v0p = triangle[0] - point;
                    IndexedVector3 v1p = triangle[1] - point;

                    IndexedVector3 cp0 = v0p.Cross(ref v1p);

                    if (IndexedVector3.Dot(ref cp0, ref triangleNormal) >= edge_tolerance)
                    {
                        IndexedVector3 v2p = triangle[2] - point;
                        IndexedVector3 cp1 = v1p.Cross(ref v2p);//= IndexedVector3.Cross(v1p,v2p);
                        if (IndexedVector3.Dot(ref cp1, ref triangleNormal) >= edge_tolerance)
                        {
                            IndexedVector3 cp2 = v2p.Cross(ref v0p);
                            if (IndexedVector3.Dot(ref cp2, ref triangleNormal) >= edge_tolerance)
                            {
                                //@BP Mod
                                // Triangle normal isn't normalized
                                triangleNormal.Normalize();

                                //@BP Mod - Allow for unflipped normal when raycasting against backfaces
                                if (((m_flags & EFlags.kF_KeepUnflippedNormal) == 0) && (dist_a <= 0.0f))
                                {
                                    IndexedVector3 negNormal = -triangleNormal;
                                    m_hitFraction = ReportHit(ref negNormal, distance, partId, triangleIndex);
                                }
                                else
                                {
                                    m_hitFraction = ReportHit(ref triangleNormal, distance, partId, triangleIndex);
                                }
                            }
                        }
                    }
                }
            }
        }

        public abstract float ReportHit(ref IndexedVector3 hitNormalLocal, float hitFraction, int partId, int triangleIndex);

        public virtual void Cleanup()
        {
        }

        public IndexedVector3 m_from;
        public IndexedVector3 m_to;
        public EFlags m_flags;
        public float m_hitFraction;

    }

    public abstract class TriangleConvexcastCallback : ITriangleCallback
    {
        public TriangleConvexcastCallback() { } // for pool
        public TriangleConvexcastCallback(ConvexShape convexShape, ref IndexedMatrix convexShapeFrom, ref IndexedMatrix convexShapeTo, ref IndexedMatrix triangleToWorld, float triangleCollisionMargin)
        {
            m_convexShape = convexShape;
            m_convexShapeFrom = convexShapeFrom;
            m_convexShapeTo = convexShapeTo;
            m_triangleToWorld = triangleToWorld;
            m_triangleCollisionMargin = triangleCollisionMargin;
        }

        public virtual void Initialize(ConvexShape convexShape, ref IndexedMatrix convexShapeFrom, ref IndexedMatrix convexShapeTo, ref IndexedMatrix triangleToWorld, float triangleCollisionMargin)
        {
            m_convexShape = convexShape;
            m_convexShapeFrom = convexShapeFrom;
            m_convexShapeTo = convexShapeTo;
            m_triangleToWorld = triangleToWorld;
            m_triangleCollisionMargin = triangleCollisionMargin;
        }

        public virtual bool graphics()
        {
            return false;
        }


        public virtual void ProcessTriangle(IndexedVector3[] triangle, int partId, int triangleIndex)
        {
            TriangleShape triangleShape = new TriangleShape(ref triangle[0], ref triangle[1], ref triangle[2]);
            triangleShape.SetMargin(m_triangleCollisionMargin);

            VoronoiSimplexSolver simplexSolver = BulletGlobals.VoronoiSimplexSolverPool.Get();
            GjkEpaPenetrationDepthSolver gjkEpaPenetrationSolver = new GjkEpaPenetrationDepthSolver();

            //#define  USE_SUBSIMPLEX_CONVEX_CAST 1
            //if you reenable USE_SUBSIMPLEX_CONVEX_CAST see commented ref code below
#if USE_SUBSIMPLEX_CONVEX_CAST
	        SubsimplexConvexCast convexCaster = new SubsimplexConvexCast(m_convexShape, triangleShape, simplexSolver);
#else
            //btGjkConvexCast	convexCaster(m_convexShape,&triangleShape,&simplexSolver);
            ContinuousConvexCollision convexCaster = BulletGlobals.ContinuousConvexCollisionPool.Get();
            convexCaster.Initialize(m_convexShape, triangleShape, simplexSolver, gjkEpaPenetrationSolver);
#endif //#USE_SUBSIMPLEX_CONVEX_CAST

            CastResult castResult = BulletGlobals.CastResultPool.Get();
            castResult.m_fraction = 1f;
            if (convexCaster.CalcTimeOfImpact(ref m_convexShapeFrom, ref m_convexShapeTo, ref m_triangleToWorld, ref m_triangleToWorld, castResult))
            {
                //add hit
                if (castResult.m_normal.LengthSquared() > 0.0001f)
                {
                    if (castResult.m_fraction < m_hitFraction)
                    {
                        /* btContinuousConvexCast's normal is already in world space */
                        /*
                        #ifdef USE_SUBSIMPLEX_CONVEX_CAST
				                        //rotate normal into worldspace
				                        castResult.m_normal = m_convexShapeFrom.getBasis() * castResult.m_normal;
                        #endif //USE_SUBSIMPLEX_CONVEX_CAST
                        */
                        castResult.m_normal.Normalize();

                        ReportHit(ref castResult.m_normal, ref castResult.m_hitPoint, castResult.m_fraction, partId, triangleIndex);
                    }
                }
            }

            BulletGlobals.ContinuousConvexCollisionPool.Free(convexCaster);
            BulletGlobals.VoronoiSimplexSolverPool.Free(simplexSolver);
            castResult.Cleanup();
        }

        public virtual void Cleanup()
        {
        }

        public abstract float ReportHit(ref IndexedVector3 hitNormalLocal, ref IndexedVector3 hitPointLocal, float hitFraction, int partId, int triangleIndex);

        public ConvexShape m_convexShape;
        public IndexedMatrix m_convexShapeFrom;
        public IndexedMatrix m_convexShapeTo;
        public IndexedMatrix m_triangleToWorld;
        public float m_hitFraction;
        public float m_triangleCollisionMargin;
        public float m_allowedPenetration;
    }
}
