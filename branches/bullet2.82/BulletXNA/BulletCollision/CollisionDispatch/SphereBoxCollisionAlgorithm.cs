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
            m_ownManifold = false;
            m_manifoldPtr = mf;
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
            m_ownManifold = false;
            m_manifoldPtr = mf;

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

            IndexedVector3 pOnBox = new IndexedVector3(); ;

            IndexedVector3 normalOnSurfaceB = new IndexedVector3();
            float penetrationDepth = 0f;
            IndexedVector3 sphereCenter = sphereObj.GetWorldTransform()._origin;
            SphereShape sphere0 = sphereObj.GetCollisionShape() as SphereShape;
            float radius = sphere0.GetRadius();
            float maxContactDistance = m_manifoldPtr.GetContactBreakingThreshold();

            resultOut.SetPersistentManifold(m_manifoldPtr);

            if (GetSphereDistance(boxObj, ref pOnBox, ref normalOnSurfaceB, ref penetrationDepth, sphereCenter, radius, maxContactDistance))
            {
                /// report a contact. internally this will be kept persistent, and contact reduction is done
                resultOut.AddContactPoint(normalOnSurfaceB, pOnBox, penetrationDepth);
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

        public bool GetSphereDistance(CollisionObject boxObj, ref IndexedVector3 pointOnBox, ref IndexedVector3 normal, ref float penetrationDepth, IndexedVector3 sphereCenter, float fRadius, float maxContactDistance)
        {
            BoxShape boxShape = boxObj.GetCollisionShape() as BoxShape;
            IndexedVector3 boxHalfExtent = boxShape.GetHalfExtentsWithoutMargin();
            float boxMargin = boxShape.GetMargin();
            penetrationDepth = 1.0f;

            // convert the sphere position to the box's local space
            IndexedMatrix m44T = boxObj.GetWorldTransform();
            IndexedVector3 sphereRelPos = m44T.InvXform(sphereCenter);

            // Determine the closest point to the sphere center in the box
            IndexedVector3 closestPoint = sphereRelPos;
            closestPoint.X = (Math.Min(boxHalfExtent.X, closestPoint.X));
            closestPoint.X = (Math.Max(-boxHalfExtent.X, closestPoint.X));
            closestPoint.Y = (Math.Min(boxHalfExtent.Y, closestPoint.Y));
            closestPoint.Y = (Math.Max(-boxHalfExtent.Y, closestPoint.Y));
            closestPoint.Z = (Math.Min(boxHalfExtent.Z, closestPoint.Z));
            closestPoint.Z = (Math.Max(-boxHalfExtent.Z, closestPoint.Z));

            float intersectionDist = fRadius + boxMargin;
            float contactDist = intersectionDist + maxContactDistance;
            normal = sphereRelPos - closestPoint;

            //if there is no penetration, we are done
            float dist2 = normal.LengthSquared();
            if (dist2 > contactDist * contactDist)
            {
                return false;
            }

            float distance;

            //special case if the sphere center is inside the box
            if (dist2 == 0.0f)
            {
                distance = -GetSpherePenetration(ref boxHalfExtent, ref sphereRelPos, ref closestPoint, ref normal);
            }
            else //compute the penetration details
            {
                distance = normal.Length();
                normal /= distance;
            }

            pointOnBox = closestPoint + normal * boxMargin;
            //	v3PointOnSphere = sphereRelPos - (normal * fRadius);	
            penetrationDepth = distance - intersectionDist;

            // transform back in world space
            IndexedVector3 tmp = m44T * pointOnBox;
            pointOnBox = tmp;
            //	tmp = m44T(v3PointOnSphere);
            //	v3PointOnSphere = tmp;
            tmp = m44T._basis * normal;
            normal = tmp;

            return true;
        }




        public float GetSpherePenetration(ref IndexedVector3 boxHalfExtent, ref IndexedVector3 sphereRelPos, ref IndexedVector3 closestPoint, ref IndexedVector3 normal)
        {
            //project the center of the sphere on the closest face of the box
            float faceDist = boxHalfExtent.X - sphereRelPos.X;
            float minDist = faceDist;
            closestPoint.X = (boxHalfExtent.X);
            normal = new IndexedVector3(1.0f, 0.0f, 0.0f);

            faceDist = boxHalfExtent.X + sphereRelPos.X;
            if (faceDist < minDist)
            {
                minDist = faceDist;
                closestPoint = sphereRelPos;
                closestPoint.X = (-boxHalfExtent.X);
                normal = new IndexedVector3(-1.0f, 0.0f, 0.0f);
            }

            faceDist = boxHalfExtent.Y - sphereRelPos.Y;
            if (faceDist < minDist)
            {
                minDist = faceDist;
                closestPoint = sphereRelPos;
                closestPoint.Y = (boxHalfExtent.Y);
                normal = new IndexedVector3(0.0f, 1.0f, 0.0f);
            }

            faceDist = boxHalfExtent.Y + sphereRelPos.Y;
            if (faceDist < minDist)
            {
                minDist = faceDist;
                closestPoint = sphereRelPos;
                closestPoint.Y = (-boxHalfExtent.Y);
                normal = new IndexedVector3(0.0f, -1.0f, 0.0f);
            }

            faceDist = boxHalfExtent.Z - sphereRelPos.Z;
            if (faceDist < minDist)
            {
                minDist = faceDist;
                closestPoint = sphereRelPos;
                closestPoint.Z = (boxHalfExtent.Z);
                normal = new IndexedVector3(0.0f, 0.0f, 1.0f);
            }

            faceDist = boxHalfExtent.Z + sphereRelPos.Z;
            if (faceDist < minDist)
            {
                minDist = faceDist;
                closestPoint = sphereRelPos;
                closestPoint.Z = (-boxHalfExtent.Z);
                normal = new IndexedVector3(0.0f, 0.0f, -1.0f);
            }

            return minDist;
        }

        private bool m_ownManifold;
        private PersistentManifold m_manifoldPtr;
        private bool m_isSwapped;
    }


    public class SphereBoxCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            SphereBoxCollisionAlgorithm algo = BulletGlobals.SphereBoxCollisionAlgorithmPool.Get();
            algo.Initialize(null, ci, body0, body1, false);
            return algo;
        }
    }

    public class SwappedSphereBoxCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            SphereBoxCollisionAlgorithm algo = BulletGlobals.SphereBoxCollisionAlgorithmPool.Get();
            algo.Initialize(null, ci, body0, body1, true);
            return algo;
        }
    }



}
