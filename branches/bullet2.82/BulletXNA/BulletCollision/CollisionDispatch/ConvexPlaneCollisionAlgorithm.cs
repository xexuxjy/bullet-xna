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
    public class ConvexPlaneCollisionAlgorithm : CollisionAlgorithm
    {

        public ConvexPlaneCollisionAlgorithm() { } // for pool

        public ConvexPlaneCollisionAlgorithm(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject col0, CollisionObject col1, bool isSwapped, int numPerturbationIterations, int minimumPointsPerturbationThreshold)
            : base(ci)
        {
            m_manifoldPtr = mf;
            m_ownManifold = false;
            m_isSwapped = isSwapped;
            m_numPerturbationIterations = numPerturbationIterations;
            m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;

            CollisionObject convexObj = m_isSwapped ? col1 : col0;
            CollisionObject planeObj = m_isSwapped ? col0 : col1;

            if (m_manifoldPtr == null && m_dispatcher.NeedsCollision(convexObj, planeObj))
            {
                m_manifoldPtr = m_dispatcher.GetNewManifold(convexObj, planeObj);
                m_ownManifold = true;
            }

        }

        public void Initialize(PersistentManifold mf, CollisionAlgorithmConstructionInfo ci, CollisionObject col0, CollisionObject col1, bool isSwapped, int numPerturbationIterations, int minimumPointsPerturbationThreshold)
        {
            base.Initialize(ci);
            m_manifoldPtr = mf;
            m_ownManifold = false;
            m_isSwapped = isSwapped;
            m_numPerturbationIterations = numPerturbationIterations;
            m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;

            CollisionObject convexObj = m_isSwapped ? col1 : col0;
            CollisionObject planeObj = m_isSwapped ? col0 : col1;

            if (m_manifoldPtr == null && m_dispatcher.NeedsCollision(convexObj, planeObj))
            {
                m_manifoldPtr = m_dispatcher.GetNewManifold(convexObj, planeObj);
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
            BulletGlobals.ConvexPlaneAlgorithmPool.Free(this);
        }

        public override void ProcessCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            if (m_manifoldPtr == null)
            {
                return;
            }

            CollisionObject convexObj = m_isSwapped ? body1 : body0;
            CollisionObject planeObj = m_isSwapped ? body0 : body1;

            ConvexShape convexShape = convexObj.GetCollisionShape() as ConvexShape;
            StaticPlaneShape planeShape = planeObj.GetCollisionShape() as StaticPlaneShape;

            bool hasCollision = false;
	        IndexedVector3 planeNormal = planeShape.GetPlaneNormal();
	        float planeConstant = planeShape.GetPlaneConstant();
	        IndexedMatrix planeInConvex;
	        planeInConvex= convexObj.GetWorldTransform().Inverse() * planeObj.GetWorldTransform();
            IndexedMatrix convexInPlaneTrans;
	        convexInPlaneTrans= planeObj.GetWorldTransform().Inverse() * convexObj.GetWorldTransform();

	        IndexedVector3 vtx = convexShape.LocalGetSupportingVertex(planeInConvex._basis*-planeNormal);
	        IndexedVector3 vtxInPlane = convexInPlaneTrans * vtx;
	        float distance = (planeNormal.Dot(vtxInPlane) - planeConstant);

	        IndexedVector3 vtxInPlaneProjected = vtxInPlane - distance*planeNormal;
	        IndexedVector3 vtxInPlaneWorld = planeObj.GetWorldTransform() * vtxInPlaneProjected;

	        hasCollision = distance < m_manifoldPtr.GetContactBreakingThreshold();
	        resultOut.SetPersistentManifold(m_manifoldPtr);
	        if (hasCollision)
	        {
		        /// report a contact. internally this will be kept persistent, and contact reduction is done
		        IndexedVector3 normalOnSurfaceB = planeObj.GetWorldTransform()._basis * planeNormal;
		        IndexedVector3 pOnB = vtxInPlaneWorld;
		        resultOut.AddContactPoint(normalOnSurfaceB,pOnB,distance);
	        }
            ////first perform a collision query with the non-perturbated collision objects
            //{
            //    IndexedQuaternion rotq = IndexedQuaternion.Identity;
            //    CollideSingleContact(ref rotq, body0, body1, dispatchInfo, resultOut);
            //}

            if (convexShape.IsPolyhedral() && resultOut.GetPersistentManifold().GetNumContacts() < m_minimumPointsPerturbationThreshold)
            {
                IndexedVector3 v0;
                IndexedVector3 v1;
                TransformUtil.PlaneSpace1(ref planeNormal, out v0, out v1);
                //now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects

                float angleLimit = 0.125f * MathUtil.SIMD_PI;
                float perturbeAngle;
                float radius = convexShape.GetAngularMotionDisc();
                perturbeAngle = BulletGlobals.gContactBreakingThreshold / radius;
                if (perturbeAngle > angleLimit)
                {
                    perturbeAngle = angleLimit;
                }
                IndexedQuaternion perturbeRot = new IndexedQuaternion(v0, perturbeAngle);
                for (int i = 0; i < m_numPerturbationIterations; i++)
                {
                    float iterationAngle = i * (MathUtil.SIMD_2_PI / (float)m_numPerturbationIterations);
                    IndexedQuaternion rotq = new IndexedQuaternion(planeNormal, iterationAngle);
                    rotq = IndexedQuaternion.Inverse(rotq) * perturbeRot *  rotq;
                    CollideSingleContact(ref rotq, body0, body1, dispatchInfo, resultOut);
                }
            }

            if (m_ownManifold)
            {
                if (m_manifoldPtr.GetNumContacts() > 0)
                {
                    resultOut.RefreshContactPoints();
                }
            }
        }

        public virtual void CollideSingleContact(ref IndexedQuaternion perturbeRot, CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            CollisionObject convexObj = m_isSwapped ? body1 : body0;
            CollisionObject planeObj = m_isSwapped ? body0 : body1;

            ConvexShape convexShape = convexObj.GetCollisionShape() as ConvexShape;
            StaticPlaneShape planeShape = planeObj.GetCollisionShape() as StaticPlaneShape;

            bool hasCollision = false;
            IndexedVector3 planeNormal = planeShape.GetPlaneNormal();
            float planeConstant = planeShape.GetPlaneConstant();

            IndexedMatrix convexWorldTransform = convexObj.GetWorldTransform();
            IndexedMatrix convexInPlaneTrans = planeObj.GetWorldTransform().Inverse() * convexWorldTransform;

            //now perturbe the convex-world transform

            convexWorldTransform._basis *= new IndexedBasisMatrix(ref perturbeRot);

            IndexedMatrix planeInConvex = convexWorldTransform.Inverse() * planeObj.GetWorldTransform(); ;

            IndexedVector3 vtx = convexShape.LocalGetSupportingVertex(planeInConvex._basis * -planeNormal);

            IndexedVector3 vtxInPlane = vtxInPlane = convexInPlaneTrans * vtx;
            float distance = (IndexedVector3.Dot(planeNormal, vtxInPlane) - planeConstant);

            IndexedVector3 vtxInPlaneProjected = vtxInPlane - (distance * planeNormal);
            IndexedVector3 vtxInPlaneWorld = planeObj.GetWorldTransform() * vtxInPlaneProjected;

            hasCollision = distance < m_manifoldPtr.GetContactBreakingThreshold();

            resultOut.SetPersistentManifold(m_manifoldPtr);
            if (hasCollision)
            {
                /// report a contact. internally this will be kept persistent, and contact reduction is done
                IndexedVector3 normalOnSurfaceB = planeObj.GetWorldTransform()._basis * planeNormal;
                IndexedVector3 pOnB = vtxInPlaneWorld;
                resultOut.AddContactPoint(ref normalOnSurfaceB, ref pOnB, distance);
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

        private bool m_ownManifold;
        private PersistentManifold m_manifoldPtr;
        private bool m_isSwapped;
        private int m_numPerturbationIterations;
        private int m_minimumPointsPerturbationThreshold;

    }

    public class ConvexPlaneCreateFunc : CollisionAlgorithmCreateFunc
    {
        public int m_numPerturbationIterations;
        public int m_minimumPointsPerturbationThreshold;

        public ConvexPlaneCreateFunc()
        {
            m_numPerturbationIterations = 1;
            m_minimumPointsPerturbationThreshold = 0;
        }

        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            ConvexPlaneCollisionAlgorithm algo = BulletGlobals.ConvexPlaneAlgorithmPool.Get();
            if (!m_swapped)
            {
                algo.Initialize(null, ci, body0, body1, false, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
            }
            else
            {
                algo.Initialize(null, ci, body0, body1, true, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
            }
            return algo;
        }
    };
}
