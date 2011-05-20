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

using System.Collections.Generic;
using BulletXNA.BulletCollision.BroadphaseCollision;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.BulletCollision.NarrowPhaseCollision;
using Microsoft.Xna.Framework;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision.CollisionDispatch
{
    public class ConvexPlaneCollisionAlgorithm : CollisionAlgorithm
    {
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

            //bool hasCollision = false;
            Vector3 planeNormal = planeShape.GetPlaneNormal();
            //float planeConstant = planeShape.getPlaneConstant();

            //first perform a collision query with the non-perturbated collision objects
            {
                Quaternion rotq = Quaternion.Identity;
                CollideSingleContact(ref rotq, body0, body1, dispatchInfo, resultOut);
            }

            if (resultOut.GetPersistentManifold().GetNumContacts() < m_minimumPointsPerturbationThreshold)
            {
                Vector3 v0;
                Vector3 v1;
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
                Quaternion perturbeRot = Quaternion.CreateFromAxisAngle(v0, perturbeAngle);
                for (int i = 0; i < m_numPerturbationIterations; i++)
                {
                    float iterationAngle = i * (MathUtil.SIMD_2_PI / (float)m_numPerturbationIterations);
                    Quaternion rotq = Quaternion.CreateFromAxisAngle(planeNormal, iterationAngle);
                    rotq = MathUtil.QuaternionMultiply(Quaternion.Inverse(rotq), MathUtil.QuaternionMultiply(perturbeRot, rotq));
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

        public virtual void CollideSingleContact(ref Quaternion perturbeRot, CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            CollisionObject convexObj = m_isSwapped ? body1 : body0;
            CollisionObject planeObj = m_isSwapped ? body0 : body1;

            ConvexShape convexShape = convexObj.GetCollisionShape() as ConvexShape;
            StaticPlaneShape planeShape = planeObj.GetCollisionShape() as StaticPlaneShape;

            bool hasCollision = false;
            Vector3 planeNormal = planeShape.GetPlaneNormal();
            float planeConstant = planeShape.GetPlaneConstant();

            Matrix convexWorldTransform = convexObj.GetWorldTransform();
            Matrix convexInPlaneTrans = Matrix.Identity;

            convexInPlaneTrans = MathUtil.BulletMatrixMultiply(Matrix.Invert(planeObj.GetWorldTransform()), convexWorldTransform);

            //now perturbe the convex-world transform

            // MAN - CHECKTHIS
            Matrix rotMatrix = Matrix.CreateFromQuaternion(perturbeRot);
            convexWorldTransform = MathUtil.BulletMatrixMultiplyBasis(ref convexWorldTransform, ref rotMatrix);

            Matrix planeInConvex = Matrix.Identity;
            planeInConvex = MathUtil.BulletMatrixMultiply(Matrix.Invert(convexWorldTransform), planeObj.GetWorldTransform());

            Vector3 tmp = Vector3.TransformNormal(-planeNormal, planeInConvex);
            Vector3 vtx = convexShape.LocalGetSupportingVertex(ref tmp);

            Vector3 vtxInPlane = Vector3.Transform(vtx, convexInPlaneTrans);
            float distance = (Vector3.Dot(planeNormal, vtxInPlane) - planeConstant);

            Vector3 vtxInPlaneProjected = vtxInPlane - (distance * planeNormal);
            Vector3 vtxInPlaneWorld = Vector3.Transform(vtxInPlaneProjected, planeObj.GetWorldTransform());

            hasCollision = distance < m_manifoldPtr.GetContactBreakingThreshold();

            resultOut.SetPersistentManifold(m_manifoldPtr);
            if (hasCollision)
            {
                /// report a contact. internally this will be kept persistent, and contact reduction is done
                Vector3 normalOnSurfaceB = Vector3.TransformNormal(planeNormal, planeObj.GetWorldTransform());
                Vector3 pOnB = vtxInPlaneWorld;
                resultOut.AddContactPoint(ref normalOnSurfaceB, ref pOnB, distance);
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
            m_minimumPointsPerturbationThreshold = 1;
        }

        public override CollisionAlgorithm CreateCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1)
        {
            if (!m_swapped)
            {
                return new ConvexPlaneCollisionAlgorithm(null, ci, body0, body1, false, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
            }
            else
            {
                return new ConvexPlaneCollisionAlgorithm(null, ci, body0, body1, true, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
            }
        }
    };
}
