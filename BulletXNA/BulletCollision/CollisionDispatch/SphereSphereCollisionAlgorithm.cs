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

using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public class SphereSphereCollisionAlgorithm : ActivatingCollisionAlgorithm
    {
        public SphereSphereCollisionAlgorithm() { } // for pool
        public SphereSphereCollisionAlgorithm(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
            : base(ci, body0, body1)
        {
            m_ownManifold = false;
            m_manifoldPtr = mf;

            // for some reason theres no check in 2.76 or 2.78 for a needs collision, just a check on pointer being null.
            if (m_manifoldPtr == null)
            {
                m_manifoldPtr = m_dispatcher.GetNewManifold(body0, body1);
                m_ownManifold = true;
            }

        }

        public virtual void Initialize(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject colObj0, CollisionObject colObj1)
        {
            base.Initialize(ci, colObj0, colObj1);
            m_ownManifold = false;
            m_manifoldPtr = mf;

            if (m_manifoldPtr == null)
            {
                m_manifoldPtr = m_dispatcher.GetNewManifold(colObj0, colObj1);
                m_ownManifold = true;
            }
        }

        public SphereSphereCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci)
            : base(ci)
        {

        }

        public override void Initialize(CollisionAlgorithmConstructionInfo ci)
        {
            base.Initialize(ci);
        }


        public override void ProcessCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            if (m_manifoldPtr == null)
            {
                return;
            }

            resultOut.SetPersistentManifold(m_manifoldPtr);

            SphereShape sphere0 = body0.GetCollisionShape() as SphereShape;
            SphereShape sphere1 = body1.GetCollisionShape() as SphereShape;

            IndexedVector3 diff = body0.GetWorldTransform()._origin - body1.GetWorldTransform()._origin;
            float len = diff.Length();
            float radius0 = sphere0.GetRadius();
            float radius1 = sphere1.GetRadius();

#if CLEAR_MANIFOLD
	        m_manifoldPtr.clearManifold(); //don't do this, it disables warmstarting
#endif

            ///iff distance positive, don't generate a new contact
            if (len > (radius0 + radius1))
            {
#if !CLEAR_MANIFOLD
                resultOut.RefreshContactPoints();
#endif //CLEAR_MANIFOLD
                return;
            }
            ///distance (negative means penetration)
            float dist = len - (radius0 + radius1);

            IndexedVector3 normalOnSurfaceB = new IndexedVector3(1, 0, 0);
            if (len > MathUtil.SIMD_EPSILON)
            {
                normalOnSurfaceB = diff / len;
            }

            ///point on A (worldspace)
            ///btVector3 pos0 = col0->getWorldTransform().getOrigin() - radius0 * normalOnSurfaceB;
            ///point on B (worldspace)
            IndexedVector3 pos1 = body1.GetWorldTransform()._origin + radius1 * normalOnSurfaceB;

            /// report a contact. internally this will be kept persistent, and contact reduction is done

            resultOut.AddContactPoint(ref normalOnSurfaceB, ref pos1, dist);

#if !CLEAR_MANIFOLD
            resultOut.RefreshContactPoints();
#endif //CLEAR_MANIFOLD


        }

        public override float CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            //not yet
            return 1f;
        }

        public override void GetAllContactManifolds(PersistentManifoldArray manifoldArray)
        {
            if (m_manifoldPtr != null && m_ownManifold)
            {
                manifoldArray.Add(m_manifoldPtr);
            }
        }

        public override void Cleanup()
        {
            if (m_ownManifold)
            {
                if (m_manifoldPtr != null)
                {
                    m_dispatcher.ReleaseManifold(m_manifoldPtr);
                }
            }
            BulletGlobals.SphereSphereCollisionAlgorithmPool.Free(this);
        }

        public bool m_ownManifold;
        public PersistentManifold m_manifoldPtr;

    }

    public class SphereSphereCreateFunc : CollisionAlgorithmCreateFunc
    {
        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            SphereSphereCollisionAlgorithm algo = BulletGlobals.SphereSphereCollisionAlgorithmPool.Get();
            algo.Initialize(null, ci, body0, body1);
            return algo;
        }
    }
}
