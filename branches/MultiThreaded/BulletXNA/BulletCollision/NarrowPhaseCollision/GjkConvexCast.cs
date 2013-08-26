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
using System;

namespace BulletXNA.BulletCollision
{
    public class GjkConvexCast : IConvexCast,IDisposable
    {
        public GjkConvexCast() { } // for pool 
        public GjkConvexCast(ConvexShape convexA, ConvexShape convexB, ISimplexSolverInterface simplexSolver)
        {
            m_convexA = convexA;
            m_convexB = convexB;
            m_simplexSolver = simplexSolver;
        }

        public void Initialize(ConvexShape convexA, ConvexShape convexB, ISimplexSolverInterface simplexSolver)
        {
            m_convexA = convexA;
            m_convexB = convexB;
            m_simplexSolver = simplexSolver;
        }

        public virtual bool CalcTimeOfImpact(IndexedMatrix fromA, IndexedMatrix toA, IndexedMatrix fromB, IndexedMatrix toB, CastResult result)
        {
            return CalcTimeOfImpact(ref fromA, ref toA, ref fromB, ref toB, result);
        }

        public virtual bool CalcTimeOfImpact(ref IndexedMatrix fromA, ref IndexedMatrix toA, ref IndexedMatrix fromB, ref IndexedMatrix toB, CastResult result)
        {
            m_simplexSolver.Reset();

            /// compute linear velocity for this interval, to interpolate
            //assume no rotation/angular velocity, assert here?
            IndexedVector3 linVelA, linVelB;
            linVelA = toA._origin - fromA._origin;
            linVelB = toB._origin - fromB._origin;

            float radius = 0.001f;
            float lambda = 0f;
            IndexedVector3 v = new IndexedVector3(1, 0, 0);

            int maxIter = MAX_ITERATIONS;

            IndexedVector3 n = IndexedVector3.Zero;
            bool hasResult = false;
            IndexedVector3 c;
            IndexedVector3 r = (linVelA - linVelB);

            float lastLambda = lambda;
            //float epsilon = float(0.001);

            int numIter = 0;
            //first solution, using GJK


            IndexedMatrix identityTrans = IndexedMatrix.Identity;

        //	result.drawCoordSystem(sphereTr);

            PointCollector pointCollector = new PointCollector();


            using (GjkPairDetector gjk = BulletGlobals.GjkPairDetectorPool.Get())
            {
                gjk.Initialize(m_convexA, m_convexB, m_simplexSolver, null);//m_penetrationDepthSolver);		
                ClosestPointInput input = ClosestPointInput.Default();

                //we don't use margins during CCD
                //	gjk.setIgnoreMargin(true);

                input.m_transformA = fromA;
                input.m_transformB = fromB;
                gjk.GetClosestPoints(ref input, pointCollector, null, false);

                hasResult = pointCollector.m_hasResult;
                c = pointCollector.m_pointInWorld;

                if (hasResult)
                {
                    float dist = pointCollector.m_distance;
                    n = pointCollector.m_normalOnBInWorld;

                    //not close enough
                    while (dist > radius)
                    {
                        numIter++;
                        if (numIter > maxIter)
                        {
                            return false; //todo: report a failure
                        }
                        float dLambda = 0f;

                        float projectedLinearVelocity = IndexedVector3.Dot(r, n);

                        dLambda = dist / (projectedLinearVelocity);

                        lambda = lambda - dLambda;

                        if (lambda > 1f || lambda < 0f)
                        {
                            return false;
                        }

                        //todo: next check with relative epsilon
                        if (lambda <= lastLambda)
                        {
                            return false;
                            //n.setValue(0,0,0);
                            //break;
                        }
                        lastLambda = lambda;

                        //interpolate to next lambda
                        result.DebugDraw(lambda);
                        input.m_transformA._origin = MathUtil.Interpolate3(fromA._origin, toA._origin, lambda);
                        input.m_transformB._origin = MathUtil.Interpolate3(fromB._origin, toB._origin, lambda);

                        gjk.GetClosestPoints(ref input, pointCollector, null, false);
                        if (pointCollector.m_hasResult)
                        {
                            if (pointCollector.m_distance < 0f)
                            {
                                result.m_fraction = lastLambda;
                                n = pointCollector.m_normalOnBInWorld;
                                result.m_normal = n;
                                result.m_hitPoint = pointCollector.m_pointInWorld;

                                return true;
                            }
                            c = pointCollector.m_pointInWorld;
                            n = pointCollector.m_normalOnBInWorld;
                            dist = pointCollector.m_distance;

                        }
                        else
                        {
                            //??
                            return false;
                        }
                    }

                    //is n normalized?
                    //don't report time of impact for motion away from the contact normal (or causes minor penetration)
                    if (IndexedVector3.Dot(n, r) >= -result.m_allowedPenetration)
                    {
                        return false;
                    }

                    result.m_fraction = lambda;
                    result.m_normal = n;
                    result.m_hitPoint = c;
                    return true;
                }
            }
            return false;
        }

        public void Dispose()
        {
            BulletGlobals.GjkConvexCastPool.Free(this);
        }

        private ISimplexSolverInterface m_simplexSolver;
        private ConvexShape m_convexA;
        private ConvexShape m_convexB;
        public const int MAX_ITERATIONS = 32;
    }
}
