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
    public class SphereTriangleDetector : IDiscreteCollisionDetectorInterface,IDisposable
    {
        public SphereTriangleDetector() { } // for pool
        public SphereTriangleDetector(SphereShape sphere, TriangleShape triangle, float contactBreakingThreshold)
        {
            m_sphere = sphere;
            m_triangle = triangle;
            m_contactBreakingThreshold = contactBreakingThreshold;
        }

        public void Initialize(SphereShape sphere, TriangleShape triangle, float contactBreakingThreshold)
        {
            m_sphere = sphere;
            m_triangle = triangle;
            m_contactBreakingThreshold = contactBreakingThreshold;
        }

        public bool Collide(ref Vector3 sphereCenter, out Vector3 point, out Vector3 resultNormal, ref float depth, ref float timeOfImpact, float contactBreakingThreshold)
        {
            Vector3[] vertices = m_triangle.GetVertexPtr(0);

            float radius = m_sphere.Radius;
            float radiusWithThreshold = radius + contactBreakingThreshold;

            Vector3 normal = Vector3.Cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
            normal.Normalize();
            Vector3 p1ToCentre = sphereCenter - vertices[0];
            float distanceFromPlane = Vector3.Dot(ref p1ToCentre, ref normal);

            if (distanceFromPlane < 0f)
            {
                //triangle facing the other way
                distanceFromPlane *= -1f;
                normal *= -1f;
            }

            bool isInsideContactPlane = distanceFromPlane < radiusWithThreshold;


            // Check for contact / intersection
            bool hasContact = false;
            Vector3 contactPoint = Vector3.Zero;
            if (isInsideContactPlane)
            {
                if (FaceContains(ref sphereCenter, vertices, ref normal))
                {
                    // Inside the contact wedge - touches a point on the shell plane
                    hasContact = true;
                    contactPoint = sphereCenter - normal * distanceFromPlane;
                }
                else
                {
                    // Could be inside one of the contact capsules
                    float contactCapsuleRadiusSqr = (radiusWithThreshold) * (radiusWithThreshold);
                    Vector3 nearestOnEdge;
                    for (int i = 0; i < m_triangle.GetNumEdges(); i++)
                    {

                        Vector3 pa;
                        Vector3 pb;

                        m_triangle.GetEdge(i, out pa, out pb);

                        float distanceSqr = SegmentSqrDistance(ref pa, ref pb, ref sphereCenter, out nearestOnEdge);
                        if (distanceSqr < contactCapsuleRadiusSqr)
                        {
                            // Yep, we're inside a capsule
                            hasContact = true;
                            contactPoint = nearestOnEdge;
                        }
                    }
                }
            }

            if (hasContact)
            {
                Vector3 contactToCentre = sphereCenter - contactPoint;
                float distanceSqr = contactToCentre.LengthSquared();
                if (distanceSqr < (radiusWithThreshold) * (radiusWithThreshold))
                {
                    if (distanceSqr > MathUtil.SIMD_EPSILON)
                    {
                        float distance = (float)Math.Sqrt(distanceSqr);
                        resultNormal = contactToCentre;
                        resultNormal.Normalize();
                        point = contactPoint;
                        depth = -(radius - distance);
                    }
                    else
                    {
                        float distance = 0.0f;
                        resultNormal = normal;
                        point = contactPoint;
                        depth = -radius;
                    }
                    return true;
                }
            }
            resultNormal = new Vector3(0, 1, 0);
            point = Vector3.Zero;
            return false;
        }

        private bool PointInTriangle(Vector3[] vertices, ref Vector3 normal, ref Vector3 p)
        {
            Vector3 p1 = vertices[0];
            Vector3 p2 = vertices[1];
            Vector3 p3 = vertices[2];

            Vector3 edge1 = p2 - p1;
            Vector3 edge2 = p3 - p2;
            Vector3 edge3 = p1 - p3;

            Vector3 p1_to_p = p - p1;
            Vector3 p2_to_p = p - p2;
            Vector3 p3_to_p = p - p3;

            Vector3 edge1_normal = Vector3.Cross(ref edge1, ref normal);
            Vector3 edge2_normal = Vector3.Cross(ref edge2, ref normal);
            Vector3 edge3_normal = Vector3.Cross(ref edge3, ref normal);

            float r1, r2, r3;
            r1 = Vector3.Dot(ref edge1_normal, ref p1_to_p);
            r2 = Vector3.Dot(ref edge2_normal, ref p2_to_p);
            r3 = Vector3.Dot(ref edge3_normal, ref p3_to_p);
            if ((r1 > 0 && r2 > 0 && r3 > 0) ||
                 (r1 <= 0 && r2 <= 0 && r3 <= 0))
            {
                return true;
            }
            return false;
        }

        private bool FaceContains(ref Vector3 p, Vector3[] vertices, ref Vector3 normal)
        {
            return PointInTriangle(vertices, ref normal, ref p);

        }


        #region IDiscreteCollisionDetectorInterface Members

        public void GetClosestPoints(ref ClosestPointInput input, IDiscreteCollisionDetectorInterfaceResult output, IDebugDraw debugDraw, bool swapResults)
        {
            Matrix transformA = input.m_transformA;
            Matrix transformB = input.m_transformB;

            Vector3 point, normal;
            float timeOfImpact = 1f;
            float depth = 0f;
            //	output.m_distance = float(1e30);
            //move sphere into triangle space
            Matrix sphereInTr = transformB.InverseTimes(ref transformA);

            Vector3 temp = sphereInTr.Translation;
            if (Collide(ref temp, out point, out normal, ref depth, ref timeOfImpact, m_contactBreakingThreshold))
            {
                if (swapResults)
                {
                    Vector3 normalOnB = transformB._basis * normal;
                    Vector3 normalOnA = -normalOnB;
                    Vector3 pointOnA = transformB * point + normalOnB * depth;
                    output.AddContactPoint(ref normalOnA, ref pointOnA, depth);
                }
                else
                {
                    Vector3 p = transformB._basis * normal;
                    Vector3 p2 = transformB * point;
                    output.AddContactPoint(ref p, ref p2, depth);
                }
            }
        }
        #endregion


        // See also geometrictools.com
        // Basic idea: D = |p - (lo + t0*lv)| where t0 = lv . (p - lo) / lv . lv
        public static float SegmentSqrDistance(ref Vector3 from, ref Vector3 to, ref Vector3 p, out Vector3 nearest)
        {
            Vector3 diff = p - from;
            Vector3 v = to - from;
            float t = Vector3.Dot(ref v, ref diff);

            if (t > 0)
            {
                float dotVV = Vector3.Dot(ref v, ref v);
                if (t < dotVV)
                {
                    t /= dotVV;
                    diff -= t * v;
                }
                else
                {
                    t = 1;
                    diff -= v;
                }
            }
            else
            {
                t = 0;
            }

            nearest = from + t * v;
            return Vector3.Dot(ref diff, ref diff);
        }

        private SphereShape m_sphere;
        private TriangleShape m_triangle;
        private float m_contactBreakingThreshold;
        private const float MAX_OVERLAP = 0f;



        public void Dispose()
        {
            BulletGlobals.SphereTriangleDetectorPool.Free(this);
        }

    }
}
