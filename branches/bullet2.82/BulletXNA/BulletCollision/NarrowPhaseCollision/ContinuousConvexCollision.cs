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
    public class ContinuousConvexCollision : IConvexCast
    {
        public ContinuousConvexCollision() { } // for pool
        public ContinuousConvexCollision(ConvexShape shapeA, ConvexShape shapeB)
        {
            m_convexA = shapeA;
            m_convexB1 = shapeB;
        }

        public ContinuousConvexCollision(ConvexShape shapeA, ConvexShape shapeB, ISimplexSolverInterface simplexSolver, IConvexPenetrationDepthSolver penetrationDepthSolver)
        {
            m_convexA = shapeA;
            m_convexB1 = shapeB;
            m_simplexSolver = simplexSolver;
            m_penetrationDepthSolver = penetrationDepthSolver;

        }

        public ContinuousConvexCollision(ConvexShape shapeA, StaticPlaneShape plane)
        {
            m_convexA = shapeA;
            m_planeShape = plane;
        }

        public virtual void Initialize(ConvexShape shapeA, ConvexShape shapeB)
        {
            m_convexA = shapeA;
            m_convexB1 = shapeB;
            m_simplexSolver = null;
            m_penetrationDepthSolver = null;
        }

        public virtual void Initialize(ConvexShape shapeA, ConvexShape shapeB, ISimplexSolverInterface simplexSolver, IConvexPenetrationDepthSolver penetrationDepthSolver)
        {
            m_convexA = shapeA;
            m_convexB1 = shapeB;
            m_simplexSolver = simplexSolver;
            m_penetrationDepthSolver = penetrationDepthSolver;
        }

        public void Initialize(ConvexShape shapeA, StaticPlaneShape plane)
        {
            m_convexA = shapeA;
            m_planeShape = plane;
            m_simplexSolver = null;
            m_penetrationDepthSolver = null;
        }



        public virtual bool CalcTimeOfImpact(ref IndexedMatrix fromA, ref IndexedMatrix toA, ref IndexedMatrix fromB, ref IndexedMatrix toB, CastResult result)
        {
            /// compute linear and angular velocity for this interval, to interpolate
            IndexedVector3 linVelA, angVelA, linVelB, angVelB;
            TransformUtil.CalculateVelocity(ref fromA, ref toA, 1f, out linVelA, out angVelA);
            TransformUtil.CalculateVelocity(ref fromB, ref toB, 1f, out linVelB, out angVelB);

            float boundingRadiusA = m_convexA.GetAngularMotionDisc();
            float boundingRadiusB = m_convexB1 != null ? m_convexB1.GetAngularMotionDisc() : 0.0f;

            float maxAngularProjectedVelocity = angVelA.Length() * boundingRadiusA + angVelB.Length() * boundingRadiusB;
            IndexedVector3 relLinVel = (linVelB - linVelA);

            float relLinVelocLength = relLinVel.Length();

            if (MathUtil.FuzzyZero(relLinVelocLength + maxAngularProjectedVelocity))
            {
                return false;
            }


            float lambda = 0f;
            IndexedVector3 v = new IndexedVector3(1, 0, 0);

            int maxIter = MAX_ITERATIONS;

            IndexedVector3 n = IndexedVector3.Zero;

            bool hasResult = false;
            IndexedVector3 c;

            float lastLambda = lambda;
            //float epsilon = float(0.001);

            int numIter = 0;
            //first solution, using GJK


            float radius = 0.001f;

            //	result.drawCoordSystem(sphereTr);

            PointCollector pointCollector1 = new PointCollector();

            {
                ComputeClosestPoints(ref fromA, ref fromB, pointCollector1);

                hasResult = pointCollector1.m_hasResult;
                c = pointCollector1.m_pointInWorld;
            }

            if (hasResult)
            {
                float dist = pointCollector1.m_distance + result.m_allowedPenetration;
 
                n = pointCollector1.m_normalOnBInWorld;

                float projectedLinearVelocity = IndexedVector3.Dot(relLinVel, n);
                if ((projectedLinearVelocity + maxAngularProjectedVelocity) <= MathUtil.SIMD_EPSILON)
                {
                    return false;
                }

                //not close enough
                while (dist > radius)
                {
                    if (result.m_debugDrawer != null)
                    {
                        IndexedVector3 colour = new IndexedVector3(1, 1, 1);
                        result.m_debugDrawer.DrawSphere(ref c, 0.2f, ref colour);
                    }
                    float dLambda = 0f;

                    projectedLinearVelocity = IndexedVector3.Dot(relLinVel, n);

                    //don't report time of impact for motion away from the contact normal (or causes minor penetration)
                    if ((projectedLinearVelocity + maxAngularProjectedVelocity) <= MathUtil.SIMD_EPSILON)
                    {
                        return false;
                    }
                    dLambda = dist / (projectedLinearVelocity + maxAngularProjectedVelocity);

                    lambda = lambda + dLambda;

                    if (lambda > 1f || lambda < 0f)
                    {
                        return false;
                    }


                    //todo: next check with relative epsilon
                    if (lambda <= lastLambda)
                    {
                        return false;
                        //n.setValue(0,0,0);
                    }

                    lastLambda = lambda;

                    //interpolate to next lambda
                    IndexedMatrix interpolatedTransA = IndexedMatrix.Identity, interpolatedTransB = IndexedMatrix.Identity, relativeTrans = IndexedMatrix.Identity;

                    TransformUtil.IntegrateTransform(ref fromA, ref linVelA, ref angVelA, lambda, out interpolatedTransA);
                    TransformUtil.IntegrateTransform(ref fromB, ref linVelB, ref angVelB, lambda, out interpolatedTransB);
                    //relativeTrans = interpolatedTransB.inverseTimes(interpolatedTransA);
                    relativeTrans = interpolatedTransB.InverseTimes(ref interpolatedTransA);
                    if (result.m_debugDrawer != null)
                    {
                        result.m_debugDrawer.DrawSphere(interpolatedTransA._origin, 0.2f, new IndexedVector3(1, 0, 0));
                    }
                    result.DebugDraw(lambda);

                    PointCollector pointCollector = new PointCollector();
                    ComputeClosestPoints(ref interpolatedTransA, ref interpolatedTransB, pointCollector);
                    if (pointCollector.m_hasResult)
                    {
                        dist = pointCollector.m_distance + result.m_allowedPenetration;

                        c = pointCollector.m_pointInWorld;
                        n = pointCollector.m_normalOnBInWorld;
                        dist = pointCollector.m_distance;
                    }
                    else
                    {
                        result.ReportFailure(-1, numIter);
                        return false;
                    }
                    numIter++;
                    if (numIter > maxIter)
                    {
                        result.ReportFailure(-2, numIter);
                        return false;
                    }


                }

                result.m_fraction = lambda;
                result.m_normal = n;
                result.m_hitPoint = c;
                return true;
            }

            return false;

        }

        public void ComputeClosestPoints(ref IndexedMatrix transA, ref IndexedMatrix transB, PointCollector pointCollector)
        {
            if (m_convexB1 != null)
            {
                m_simplexSolver.Reset();
                GjkPairDetector gjk = new GjkPairDetector(m_convexA, m_convexB1, m_convexA.GetShapeType(), m_convexB1.GetShapeType(), m_convexA.GetMargin(), m_convexB1.GetMargin(), m_simplexSolver, m_penetrationDepthSolver);
                ClosestPointInput input = ClosestPointInput.Default();
                input.m_transformA = transA;
                input.m_transformB = transB;
                gjk.GetClosestPoints(ref input, pointCollector, null);
            }
            else
            {
                //convex versus plane
                ConvexShape convexShape = m_convexA;
                StaticPlaneShape planeShape = m_planeShape;

                bool hasCollision = false;
                IndexedVector3 planeNormal = planeShape.GetPlaneNormal();
                float planeConstant = planeShape.GetPlaneConstant();

                IndexedMatrix convexWorldTransform = transA;
                IndexedMatrix convexInPlaneTrans = transB.Inverse() * convexWorldTransform;
                IndexedMatrix planeInConvex = convexWorldTransform.Inverse() *  transB;

                IndexedVector3 vtx = convexShape.LocalGetSupportingVertex(planeInConvex._basis * -planeNormal);

                IndexedVector3 vtxInPlane = convexInPlaneTrans * vtx;
                float distance = IndexedVector3.Dot(planeNormal, vtxInPlane) - planeConstant;

                IndexedVector3 vtxInPlaneProjected = vtxInPlane - distance * planeNormal;
                IndexedVector3 vtxInPlaneWorld = transB * vtxInPlaneProjected;
                IndexedVector3 normalOnSurfaceB = transB._basis * planeNormal;

                pointCollector.AddContactPoint(
                    ref normalOnSurfaceB,
                    ref vtxInPlaneWorld,
                    distance);
            }

        }


        private ISimplexSolverInterface m_simplexSolver;
        private IConvexPenetrationDepthSolver m_penetrationDepthSolver;
        private ConvexShape m_convexA;
        private ConvexShape m_convexB1;
        private StaticPlaneShape m_planeShape;
        private static int MAX_ITERATIONS = 64;
    }
}
