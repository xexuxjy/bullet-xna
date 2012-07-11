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
    public class GjkEpaPenetrationDepthSolver : IConvexPenetrationDepthSolver
    {
        public GjkEpaPenetrationDepthSolver() { } // for pool

        public virtual bool CalcPenDepth(ISimplexSolverInterface simplexSolver, ConvexShape convexA, ConvexShape convexB, ref IndexedMatrix transA, ref IndexedMatrix transB,
                ref IndexedVector3 v, ref IndexedVector3 wWitnessOnA, ref IndexedVector3 wWitnessOnB, IDebugDraw debugDraw)
        {
            //float radialmargin = 0f;

            IndexedVector3 guessVector = (transA._origin - transB._origin);
            GjkEpaSolver2Results results = new GjkEpaSolver2Results();
            if (GjkEpaSolver2.Penetration(convexA, ref transA,
                                        convexB, ref transB,
                                        ref guessVector, ref results))
            {
                //	debugDraw->drawLine(results.witnesses[1],results.witnesses[1]+results.normal,btVector3(255,0,0));
                //resultOut->addContactPoint(results.normal,results.witnesses[1],-results.depth);
                wWitnessOnA = results.witnesses0;
                wWitnessOnB = results.witnesses1;
                v = results.normal;
                return true;
            }
            else
            {
                if (GjkEpaSolver2.Distance(convexA, ref transA, convexB, ref transB, ref guessVector, ref results))
                {
                    wWitnessOnA = results.witnesses0;
                    wWitnessOnB = results.witnesses1;
                    v = results.normal;
                    return false;
                }
            }
            return false;
        }
    }
}

