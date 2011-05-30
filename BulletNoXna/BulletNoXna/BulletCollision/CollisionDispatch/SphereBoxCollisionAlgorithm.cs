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
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public class SphereBoxCollisionAlgorithm : ActivatingCollisionAlgorithm
    {
        public SphereBoxCollisionAlgorithm(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject col0, CollisionObject col1, bool isSwapped)
            : base(ci, col0, col1)
        {
            CollisionObject sphereObj = m_isSwapped ? col1 : col0;
            CollisionObject boxObj = m_isSwapped ? col0 : col1;

            if (m_manifoldPtr == null && m_dispatcher.NeedsCollision(sphereObj, boxObj))
            {
                m_manifoldPtr = m_dispatcher.GetNewManifold(sphereObj, boxObj);
                m_ownManifold = true;
            }
        }

        public override void Cleanup()
        {
            if (m_ownManifold)
            {
                if (m_manifoldPtr != null)
                {
                    m_dispatcher.ReleaseManifold(m_manifoldPtr);
                    m_manifoldPtr = null;
                }
                m_ownManifold = false;
            }
            m_ownManifold = false;
        }

        public override void ProcessCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            //(void)dispatchInfo;
            //(void)resultOut;
            if (m_manifoldPtr == null)
            {
                return;
            }

            CollisionObject sphereObj = m_isSwapped ? body1 : body0;
            CollisionObject boxObj = m_isSwapped ? body0 : body1;

            SphereShape sphere0 = sphereObj.CollisionShape as SphereShape;

            //Vector3 normalOnSurfaceB;
            Vector3 pOnBox = Vector3.Zero, pOnSphere = Vector3.Zero;
            Vector3 sphereCenter = sphereObj.GetWorldTransform().Translation;
            float radius = sphere0.Radius;

            float dist = GetSphereDistance(boxObj, ref pOnBox, ref pOnSphere, ref sphereCenter, radius);
            resultOut.SetPersistentManifold(m_manifoldPtr);

            if (dist < MathUtil.SIMD_EPSILON)
            {
                Vector3 normalOnSurfaceB = (pOnBox - pOnSphere);
                normalOnSurfaceB.Normalize();

                /// report a contact. internally this will be kept persistent, and contact reduction is done

                resultOut.AddContactPoint(ref normalOnSurfaceB, ref pOnBox, dist);
            }

            if (m_ownManifold)
            {
                if (m_manifoldPtr.GetNumContacts() > 0)
                {
                    resultOut.RefreshContactPoints();
                }
            }
        }

        public override float CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            return 1f;
        }

        public override void GetAllContactManifolds(ObjectArray<PersistentManifold> manifoldArray)
        {
            if (m_manifoldPtr != null && m_ownManifold)
            {
                manifoldArray.Add(m_manifoldPtr);
            }
        }

        public float GetSphereDistance(CollisionObject boxObj, ref Vector3 v3PointOnBox, ref Vector3 v3PointOnSphere, ref Vector3 v3SphereCenter, float fRadius)
        {
            float margins;
            Vector3[] bounds = new Vector3[2];
            BoxShape boxShape = boxObj.CollisionShape as BoxShape;

            bounds[0] = -boxShape.GetHalfExtentsWithoutMargin();
            bounds[1] = boxShape.GetHalfExtentsWithoutMargin();

            margins = boxShape.Margin;//also add sphereShape margin?

            Matrix m44T = boxObj.GetWorldTransform();

            Vector3[] boundsVec = new Vector3[2];
            float fPenetration;

            boundsVec[0] = bounds[0];
            boundsVec[1] = bounds[1];

            Vector3 marginsVec = new Vector3(margins);

            // add margins
            bounds[0] += marginsVec;
            bounds[1] -= marginsVec;

            /////////////////////////////////////////////////

            Vector3 tmp, prel, normal, v3P;
            Vector3[] n = new Vector3[6];

            float fSep = 10000000.0f;
            float fSepThis;

            n[0] = new Vector3(-1, 0, 0);
            n[1] = new Vector3(0, -1, 0);
            n[2] = new Vector3(0, 0, -1);
            n[3] = new Vector3(1, 0, 0);
            n[4] = new Vector3(0, 1, 0);
            n[5] = new Vector3(0, 0, 1);

            // convert  point in local space
            prel = MathUtil.InverseTransform(ref m44T, ref v3SphereCenter);

            bool bFound = false;

            v3P = prel;

            for (int i = 0; i < 6; i++)
            {
                int j = i < 3 ? 0 : 1;
                fSepThis = Vector3.Dot((v3P - bounds[j]), n[i]);
                if (fSepThis != 0f)
                {
                    v3P = v3P - n[i] * fSepThis;
                    bFound = true;
                }
            }

            //

            if (bFound)
            {
                bounds[0] = boundsVec[0];
                bounds[1] = boundsVec[1];

                normal = (prel - v3P);
                normal.Normalize();
                v3PointOnBox = v3P + normal * margins;
                v3PointOnSphere = prel - normal * fRadius;

                if ((Vector3.Dot((v3PointOnSphere - v3PointOnBox), normal)) > 0f)
                {
                    return 1f;
                }

                // transform back in world space
                tmp = Vector3.Transform(v3PointOnBox, m44T);
                v3PointOnBox = tmp;
                tmp = Vector3.Transform(v3PointOnSphere, m44T);
                v3PointOnSphere = tmp;
                float fSeps2 = (v3PointOnBox - v3PointOnSphere).LengthSquared();

                //if this fails, fallback into deeper penetration case, below
                if (fSeps2 > MathUtil.SIMD_EPSILON)
                {
                    fSep = (float)-Math.Sqrt(fSeps2);
                    normal = (v3PointOnBox - v3PointOnSphere);
                    normal *= 1f / fSep;
                }

                return fSep;
            }

            //////////////////////////////////////////////////
            // Deep penetration case

            fPenetration = GetSpherePenetration(boxObj, ref v3PointOnBox, ref v3PointOnSphere, ref v3SphereCenter, fRadius, ref bounds[0], ref bounds[1]);

            bounds[0] = boundsVec[0];
            bounds[1] = boundsVec[1];

            if (fPenetration <= 0f)
            {
                return (fPenetration - margins);
            }
            else
            {
                return 1f;
            }
        }

        public float GetSpherePenetration(CollisionObject boxObj, ref Vector3 v3PointOnBox, ref Vector3 v3PointOnSphere, ref Vector3 v3SphereCenter, float fRadius, ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
            Vector3[] bounds = new Vector3[2];

            bounds[0] = aabbMin;
            bounds[1] = aabbMax;

            Vector3 p0, tmp, prel, normal;
            Vector3[] n = new Vector3[6];
            float fSep = -10000000.0f;
            float fSepThis;

            // set p0 and normal to a default value to shup up GCC
            p0 = Vector3.Zero;
            normal = Vector3.Zero;

            n[0] = new Vector3(-1, 0, 0);
            n[1] = new Vector3(0, -1, 0);
            n[2] = new Vector3(0, 0, -1);
            n[3] = new Vector3(1, 0, 0);
            n[4] = new Vector3(0, 1, 0);
            n[5] = new Vector3(0, 0, 1);

            Matrix m44T = boxObj.GetWorldTransform();

            // convert  point in local space

            prel = MathUtil.InverseTransform(ref m44T, ref v3SphereCenter);

            ///////////

            for (int i = 0; i < 6; i++)
            {
                int j = i < 3 ? 0 : 1;
                if ((fSepThis = (Vector3.Dot((prel - bounds[j]), n[i])) - fRadius) > 0f)
                {
                    return 1f;
                }
                if (fSepThis > fSep)
                {
                    p0 = bounds[j];
                    normal = n[i];
                    fSep = fSepThis;
                }
            }

            v3PointOnBox = prel - normal * (Vector3.Dot(normal, (prel - p0)));
            v3PointOnSphere = v3PointOnBox + normal * fSep;

            // transform back in world space
            tmp = Vector3.Transform(v3PointOnBox, m44T);
            v3PointOnBox = tmp;
            tmp = Vector3.Transform(v3PointOnSphere, m44T);
            v3PointOnSphere = tmp;
            normal = (v3PointOnBox - v3PointOnSphere);
            normal.Normalize();

            return fSep;

        }

        private bool m_ownManifold;
        private PersistentManifold m_manifoldPtr;
        private bool m_isSwapped;

    }
}
