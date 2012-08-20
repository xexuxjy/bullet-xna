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
    public class SphereBoxCollisionAlgorithm : ActivatingCollisionAlgorithm
    {
        public SphereBoxCollisionAlgorithm() { } // for pool
        public SphereBoxCollisionAlgorithm(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject col0, CollisionObject col1, bool isSwapped)
            : base(ci, col0, col1)
        {
            m_isSwapped = isSwapped;
            CollisionObject sphereObj = m_isSwapped ? col1 : col0;
            CollisionObject boxObj = m_isSwapped ? col0 : col1;
            
            if (m_manifoldPtr == null && m_dispatcher.NeedsCollision(sphereObj, boxObj))
            {
                m_manifoldPtr = m_dispatcher.GetNewManifold(sphereObj, boxObj);
                m_ownManifold = true;
            }
        }


        public void Initialize(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject col0, CollisionObject col1, bool isSwapped)
        {
            base.Initialize(ci, col0, col1);
            m_isSwapped = isSwapped;
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
            BulletGlobals.SphereBoxCollisionAlgorithmPool.Free(this);
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

            SphereShape sphere0 = sphereObj.GetCollisionShape() as SphereShape;

            //IndexedVector3 normalOnSurfaceB;
            IndexedVector3 pOnBox = IndexedVector3.Zero, pOnSphere = IndexedVector3.Zero;
            IndexedVector3 sphereCenter = sphereObj.GetWorldTransform()._origin;
            float radius = sphere0.GetRadius();

            float dist = GetSphereDistance(boxObj, ref pOnBox, ref pOnSphere, ref sphereCenter, radius);
            resultOut.SetPersistentManifold(m_manifoldPtr);

            if (dist < MathUtil.SIMD_EPSILON)
            {
                IndexedVector3 normalOnSurfaceB = (pOnBox - pOnSphere);
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

        public override void GetAllContactManifolds(PersistentManifoldArray manifoldArray)
        {
            if (m_manifoldPtr != null && m_ownManifold)
            {
                manifoldArray.Add(m_manifoldPtr);
            }
        }

        public float GetSphereDistance(CollisionObject boxObj, ref IndexedVector3 v3PointOnBox, ref IndexedVector3 v3PointOnSphere, ref IndexedVector3 v3SphereCenter, float fRadius)
        {
            float margins;
            BoxShape boxShape = boxObj.GetCollisionShape() as BoxShape;

            m_sphereBoundsBoundsArray[0] = -boxShape.GetHalfExtentsWithoutMargin();
            m_sphereBoundsBoundsArray[1] = boxShape.GetHalfExtentsWithoutMargin();

            margins = boxShape.GetMargin();//also add sphereShape margin?

            IndexedMatrix m44T = boxObj.GetWorldTransform();

            float fPenetration;

            m_sphereBoundsBoundsArray2[0] = m_sphereBoundsBoundsArray[0];
            m_sphereBoundsBoundsArray2[1] = m_sphereBoundsBoundsArray[1];

            IndexedVector3 marginsVec = new IndexedVector3(margins);

            // add margins
            m_sphereBoundsBoundsArray[0] += marginsVec;
            m_sphereBoundsBoundsArray[1] -= marginsVec;

            /////////////////////////////////////////////////

            IndexedVector3 tmp, prel, normal, v3P;

            float fSep = 10000000.0f;
            float fSepThis;

            m_sphereBoundsNArray[0] = new IndexedVector3(-1, 0, 0);
            m_sphereBoundsNArray[1] = new IndexedVector3(0, -1, 0);
            m_sphereBoundsNArray[2] = new IndexedVector3(0, 0, -1);
            m_sphereBoundsNArray[3] = new IndexedVector3(1, 0, 0);
            m_sphereBoundsNArray[4] = new IndexedVector3(0, 1, 0);
            m_sphereBoundsNArray[5] = new IndexedVector3(0, 0, 1);

            // convert  point in local space
            MathUtil.InverseTransform(ref m44T, ref v3SphereCenter, out prel);

            bool bFound = false;

            v3P = prel;

            for (int i = 0; i < 6; i++)
            {
                int j = i < 3 ? 0 : 1;
                fSepThis = IndexedVector3.Dot((v3P - m_sphereBoundsBoundsArray[j]), m_sphereBoundsNArray[i]);
                if (fSepThis != 0f)
                {
                    v3P = v3P - m_sphereBoundsNArray[i] * fSepThis;
                    bFound = true;
                }
            }

            //

            if (bFound)
            {
                m_sphereBoundsBoundsArray[0] = m_sphereBoundsBoundsArray2[0];
                m_sphereBoundsBoundsArray[1] = m_sphereBoundsBoundsArray2[1];

                normal = (prel - v3P);
                normal.Normalize();
                v3PointOnBox = v3P + normal * margins;
                v3PointOnSphere = prel - normal * fRadius;

                if ((IndexedVector3.Dot((v3PointOnSphere - v3PointOnBox), normal)) > 0f)
                {
                    return 1f;
                }

                // transform back in world space
                tmp = m44T * v3PointOnBox;
                v3PointOnBox = tmp;
                tmp = m44T * v3PointOnSphere;
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

            fPenetration = GetSpherePenetration(boxObj, ref v3PointOnBox, ref v3PointOnSphere, ref v3SphereCenter, fRadius, ref m_sphereBoundsBoundsArray[0], ref m_sphereBoundsBoundsArray[1]);

            m_sphereBoundsBoundsArray[0] = m_sphereBoundsBoundsArray2[0];
            m_sphereBoundsBoundsArray[1] = m_sphereBoundsBoundsArray2[1];

            if (fPenetration <= 0f)
            {
                return (fPenetration - margins);
            }
            else
            {
                return 1f;
            }
        }

        public float GetSpherePenetration(CollisionObject boxObj, ref IndexedVector3 v3PointOnBox, ref IndexedVector3 v3PointOnSphere, ref IndexedVector3 v3SphereCenter, float fRadius, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
            m_spherePenetrationBoundsArray[0] = aabbMin;
            m_spherePenetrationBoundsArray[1] = aabbMax;

            IndexedVector3 p0, tmp, prel, normal;
            float fSep = -10000000.0f;
            float fSepThis;

            // set p0 and normal to a default value to shup up GCC
            p0 = IndexedVector3.Zero;
            normal = IndexedVector3.Zero;

            m_spherePenetrationNArray[0] = new IndexedVector3(-1, 0, 0);
            m_spherePenetrationNArray[1] = new IndexedVector3(0, -1, 0);
            m_spherePenetrationNArray[2] = new IndexedVector3(0, 0, -1);
            m_spherePenetrationNArray[3] = new IndexedVector3(1, 0, 0);
            m_spherePenetrationNArray[4] = new IndexedVector3(0, 1, 0);
            m_spherePenetrationNArray[5] = new IndexedVector3(0, 0, 1);

            IndexedMatrix m44T = boxObj.GetWorldTransform();

            // convert  point in local space

            MathUtil.InverseTransform(ref m44T, ref v3SphereCenter, out prel);

            ///////////

            for (int i = 0; i < 6; i++)
            {
                int j = i < 3 ? 0 : 1;
                if ((fSepThis = (IndexedVector3.Dot((prel - m_spherePenetrationBoundsArray[j]), m_spherePenetrationNArray[i])) - fRadius) > 0f)
                {
                    return 1f;
                }
                if (fSepThis > fSep)
                {
                    p0 = m_spherePenetrationBoundsArray[j];
                    normal = m_spherePenetrationNArray[i];
                    fSep = fSepThis;
                }
            }

            v3PointOnBox = prel - normal * (IndexedVector3.Dot(normal, (prel - p0)));
            v3PointOnSphere = v3PointOnBox + normal * fSep;

            // transform back in world space
            tmp = m44T * v3PointOnBox;
            v3PointOnBox = tmp;
            tmp = m44T * v3PointOnSphere;
            v3PointOnSphere = tmp;
            normal = (v3PointOnBox - v3PointOnSphere);
            normal.Normalize();

            return fSep;

        }

        private bool m_ownManifold;
        private PersistentManifold m_manifoldPtr;
        private bool m_isSwapped;
        private IndexedVector3[] m_sphereBoundsBoundsArray = new IndexedVector3[2];
        private IndexedVector3[] m_sphereBoundsBoundsArray2 = new IndexedVector3[2];
        private IndexedVector3[] m_sphereBoundsNArray = new IndexedVector3[6];
        private IndexedVector3[] m_spherePenetrationBoundsArray = new IndexedVector3[2];
        private IndexedVector3[] m_spherePenetrationNArray = new IndexedVector3[6];

    }
}
