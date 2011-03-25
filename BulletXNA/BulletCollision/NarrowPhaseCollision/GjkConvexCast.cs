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

using Microsoft.Xna.Framework;
using BulletXNA.BulletCollision.CollisionShapes;

namespace BulletXNA.BulletCollision.NarrowPhaseCollision
{
    public class GjkConvexCast : IConvexCast
    {
        public GjkConvexCast(ConvexShape convexA, ConvexShape convexB, ISimplexSolverInterface simplexSolver)
        {
            m_convexA = convexA;
            m_convexB = convexB;
            m_simplexSolver = simplexSolver;
        }

        public virtual bool CalcTimeOfImpact(Matrix fromA, Matrix toA, Matrix fromB, Matrix toB, CastResult result)
        {
            return CalcTimeOfImpact(ref fromA, ref toA, ref fromB, ref toB, result);
        }

        public virtual bool CalcTimeOfImpact(ref Matrix fromA, ref Matrix toA, ref Matrix fromB, ref Matrix toB, CastResult result)
        {
	        m_simplexSolver.Reset();

	        /// compute linear velocity for this interval, to interpolate
	        //assume no rotation/angular velocity, assert here?
	        Vector3 linVelA,linVelB;
	        linVelA = toA.Translation-fromA.Translation;
	        linVelB = toB.Translation-fromB.Translation;

	        float radius = 0.001f;
	        float lambda = 0f;
	        Vector3 v = new Vector3(1,0,0);

	        int maxIter = MAX_ITERATIONS;

	        Vector3 n = Vector3.Zero;
	        bool hasResult = false;
	        Vector3 c;
	        Vector3 r = (linVelA-linVelB);

	        float lastLambda = lambda;
	        //btScalar epsilon = btScalar(0.001);

	        int numIter = 0;
	        //first solution, using GJK


	        Matrix identityTrans = Matrix.Identity;

        //	result.drawCoordSystem(sphereTr);

	        PointCollector	pointCollector = new PointCollector();

        		
	        GjkPairDetector gjk = new GjkPairDetector(m_convexA,m_convexB,m_simplexSolver,null);//m_penetrationDepthSolver);		
	        ClosestPointInput input = new ClosestPointInput();

	        //we don't use margins during CCD
	        //	gjk.setIgnoreMargin(true);

	        input.m_transformA = fromA;
	        input.m_transformB = fromB;
	        gjk.GetClosestPoints(input,pointCollector,null,false);

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

			        float projectedLinearVelocity = Vector3.Dot(r,n);
        			
			        dLambda = dist / (projectedLinearVelocity);

			        lambda = lambda - dLambda;

			        if (lambda > 1f || lambda < 0f)
				        return false;

			        //todo: next check with relative epsilon
			        if (lambda <= lastLambda)
			        {
				        return false;
				        //n.setValue(0,0,0);
                        //break;
			        }
			        lastLambda = lambda;

			        //interpolate to next lambda
			        result.DebugDraw( lambda );
			        input.m_transformA.Translation = MathUtil.Interpolate3(fromA.Translation,toA.Translation,lambda);
                    input.m_transformB.Translation = MathUtil.Interpolate3(fromB.Translation, toB.Translation, lambda);
        			
			        gjk.GetClosestPoints(input,pointCollector,null,false);
			        if (pointCollector.m_hasResult)
			        {
				        if (pointCollector.m_distance < 0f)
				        {
					        result.m_fraction = lastLambda;
					        n = pointCollector.m_normalOnBInWorld;
					        result.m_normal=n;
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
                if (Vector3.Dot(n, r) >= -result.m_allowedPenetration)
                {
                    return false;
                }

		        result.m_fraction = lambda;
		        result.m_normal = n;
		        result.m_hitPoint = c;
		        return true;
	        }

	        return false;
        }

        private ISimplexSolverInterface m_simplexSolver;
	    private ConvexShape	m_convexA;
        private ConvexShape m_convexB;
        public const int MAX_ITERATIONS = 32;
    }
}
