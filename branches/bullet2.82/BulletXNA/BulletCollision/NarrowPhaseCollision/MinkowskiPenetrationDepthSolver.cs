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

#define USE_BATCHED_SUPPORT
using BulletXNA.LinearMath;


namespace BulletXNA.BulletCollision
{
    public class MinkowskiPenetrationDepthSolver : IConvexPenetrationDepthSolver
    {
        public bool CalcPenDepth(ISimplexSolverInterface simplexSolver, ConvexShape convexA, ConvexShape convexB, ref IndexedMatrix transA, ref IndexedMatrix transB,
                ref IndexedVector3 v, ref IndexedVector3 pa, ref IndexedVector3 pb, IDebugDraw debugDraw)
        {
            bool check2d = convexA.IsConvex2d() && convexB.IsConvex2d();


            float minProj = float.MaxValue;
            IndexedVector3 minNorm = IndexedVector3.Zero;
            IndexedVector3 minA = IndexedVector3.Zero, minB = IndexedVector3.Zero;
            IndexedVector3 seperatingAxisInA, seperatingAxisInB;
            IndexedVector3 pInA, qInB, pWorld, qWorld, w;

#if USE_BATCHED_SUPPORT


            IndexedVector4[] supportVerticesABatch = new IndexedVector4[NUM_UNITSPHERE_POINTS + ConvexShape.MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
            IndexedVector4[] supportVerticesBBatch = new IndexedVector4[NUM_UNITSPHERE_POINTS + ConvexShape.MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
            IndexedVector3[] seperatingAxisInABatch = new IndexedVector3[NUM_UNITSPHERE_POINTS + ConvexShape.MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
            IndexedVector3[] seperatingAxisInBBatch = new IndexedVector3[NUM_UNITSPHERE_POINTS + ConvexShape.MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];


            int numSampleDirections = NUM_UNITSPHERE_POINTS;

            for (int i = 0; i < numSampleDirections; i++)
            {
                IndexedVector3 norm = sPenetrationDirections[i];
                IndexedVector3 negNorm = -norm;

                IndexedBasisMatrix.Multiply(ref seperatingAxisInABatch[i], ref negNorm, ref transA._basis);
                IndexedBasisMatrix.Multiply(ref seperatingAxisInBBatch[i], ref norm, ref transB._basis);
                //seperatingAxisInABatch[i] = (-norm) * transA._basis;
                //seperatingAxisInBBatch[i] = norm * transB._basis;
            }

            {
                int numPDA = convexA.GetNumPreferredPenetrationDirections();
                if (numPDA > 0)
                {
                    for (int i = 0; i < numPDA; i++)
                    {
                        IndexedVector3 norm;
                        convexA.GetPreferredPenetrationDirection(i, out norm);
                        IndexedBasisMatrix.Multiply(ref norm ,ref transA._basis ,ref norm);
                        sPenetrationDirections[numSampleDirections] = norm;
                        IndexedVector3 negNorm = -norm;
                        IndexedBasisMatrix.Multiply(ref seperatingAxisInABatch[numSampleDirections], ref negNorm,ref transA._basis);
                        IndexedBasisMatrix.Multiply(ref seperatingAxisInBBatch[numSampleDirections] ,ref norm ,ref transB._basis);
                        numSampleDirections++;
                    }
                }
            }

            {
                int numPDB = convexB.GetNumPreferredPenetrationDirections();
                if (numPDB > 0)
                {
                    for (int i = 0; i < numPDB; i++)
                    {
                        IndexedVector3 norm;
                        convexB.GetPreferredPenetrationDirection(i, out norm);
                        IndexedBasisMatrix.Multiply(ref norm, ref transB._basis, ref norm);
                        sPenetrationDirections[numSampleDirections] = norm;
                        IndexedVector3 negNorm = -norm;
                        IndexedBasisMatrix.Multiply(ref seperatingAxisInABatch[numSampleDirections],ref negNorm,ref transA._basis);
                        IndexedBasisMatrix.Multiply(ref seperatingAxisInBBatch[numSampleDirections],ref norm ,ref transB._basis);
                        numSampleDirections++;
                    }
                }
            }

            convexA.BatchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInABatch, supportVerticesABatch, numSampleDirections);
            convexB.BatchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInBBatch, supportVerticesBBatch, numSampleDirections);

            for (int i = 0; i < numSampleDirections; i++)
            {
                IndexedVector3 norm = sPenetrationDirections[i];
                if (check2d)
                {
                    // shouldn't this be Y ?
                    norm.Z = 0;
                }
                if (norm.LengthSquared() > 0.01f)
                {
                    seperatingAxisInA = seperatingAxisInABatch[i];
                    seperatingAxisInB = seperatingAxisInBBatch[i];

                    pInA = new IndexedVector3(supportVerticesABatch[i].X, supportVerticesABatch[i].Y, supportVerticesABatch[i].Z);
                    qInB = new IndexedVector3(supportVerticesBBatch[i].X, supportVerticesBBatch[i].Y, supportVerticesBBatch[i].Z);

                    IndexedMatrix.Multiply(out  pWorld, ref transA, ref pInA);
                    IndexedMatrix.Multiply(out  qWorld, ref transB, ref qInB);
                    if (check2d)
                    {
                        // shouldn't this be Y ?
                        pWorld.Z = 0f;
                        qWorld.Z = 0f;
                    }

                    IndexedVector3.Subtract(out w, ref qWorld, ref pWorld);
                    float delta = IndexedVector3.Dot(ref norm, ref w);
                    //find smallest delta
                    if (delta < minProj)
                    {
                        minProj = delta;
                        minNorm = norm;
                        minA = pWorld;
                        minB = qWorld;
                    }
                }
            }
#else
            int numSampleDirections = NUM_UNITSPHERE_POINTS;

	        {
		        int numPDA = convexA.GetNumPreferredPenetrationDirections();
		        if (numPDA > 0)
		        {
			        for (int i=0;i<numPDA;i++)
			        {
				        IndexedVector3 norm;
				        convexA.GetPreferredPenetrationDirection(i, out norm);
				        norm  = IndexedVector3.TransformNormal(norm,transA);
				        sPenetrationDirections[numSampleDirections] = norm;
				        numSampleDirections++;
			        }
		        }
	        }

	        {
		        int numPDB = convexB.GetNumPreferredPenetrationDirections();
		        if (numPDB > 0)
		        {
			        for (int i=0;i<numPDB;i++)
			        {
                        IndexedVector3 norm = IndexedVector3.Zero;
				        convexB.GetPreferredPenetrationDirection(i, out norm);
				        norm  = IndexedVector3.TransformNormal(norm,transB);
				        sPenetrationDirections[numSampleDirections] = norm;
				        numSampleDirections++;
			        }
		        }
	        }

	        for (int i=0;i<numSampleDirections;i++)
	        {
		        IndexedVector3 norm = sPenetrationDirections[i];
		        if (check2d)
		        {
			        norm.Z = 0f;
		        }
                if (norm.LengthSquared() > 0.01f)
                {
                    seperatingAxisInA = IndexedVector3.TransformNormal(-norm, transA);
                    seperatingAxisInB = IndexedVector3.TransformNormal(norm, transB);
                    pInA = convexA.LocalGetSupportVertexWithoutMarginNonVirtual(ref seperatingAxisInA);
                    qInB = convexB.LocalGetSupportVertexWithoutMarginNonVirtual(ref seperatingAxisInB);
                    pWorld = IndexedVector3.Transform(pInA, transA);
                    qWorld = IndexedVector3.Transform(qInB, transB);
                    if (check2d)
                    {
                        pWorld.Z = 0.0f;
                        qWorld.Z = 0.0f;
                    }

                    w = qWorld - pWorld;
                    float delta = IndexedVector3.Dot(norm, w);
                    //find smallest delta
                    if (delta < minProj)
                    {
                        minProj = delta;
                        minNorm = norm;
                        minA = pWorld;
                        minB = qWorld;
                    }
                }
	        }
#endif //USE_BATCHED_SUPPORT

            //add the margins

            minA += minNorm * convexA.GetMarginNonVirtual();
            minB -= minNorm * convexB.GetMarginNonVirtual();
            //no penetration
            if (minProj < 0f)
            {
                return false;
            }

            float extraSeparation = 0.5f;///scale dependent
            minProj += extraSeparation + (convexA.GetMarginNonVirtual() + convexB.GetMarginNonVirtual());

#if DEBUG_DRAW
	        if (debugDraw)
	        {
		        IndexedVector3 color = new IndexedVector3(0,1,0);
		        debugDraw.drawLine(minA,minB,color);
		        color = new IndexedVector3(1,1,1);
		        IndexedVector3 vec = minB-minA;
		        float prj2 = IndexedVector3.Dot(minNorm,vec);
		        debugDraw.drawLine(minA,minA+(minNorm*minProj),color);

	        }
#endif //DEBUG_DRAW



            GjkPairDetector gjkdet = BulletGlobals.GjkPairDetectorPool.Get();
            gjkdet.Initialize(convexA, convexB, simplexSolver, null);

            float offsetDist = minProj;
            IndexedVector3 offset = minNorm * offsetDist;

            ClosestPointInput input = ClosestPointInput.Default();

            IndexedVector3 newOrg = transA._origin + offset;

            IndexedMatrix displacedTrans = transA;
            displacedTrans._origin = newOrg;

            input.m_transformA = displacedTrans;
            input.m_transformB = transB;
            input.m_maximumDistanceSquared = float.MaxValue;

            MinkowskiIntermediateResult res = new MinkowskiIntermediateResult();
            gjkdet.SetCachedSeperatingAxis(-minNorm);

            gjkdet.GetClosestPoints(ref input, res, debugDraw, false);

            float correctedMinNorm = minProj - res.m_depth;

            //the penetration depth is over-estimated, relax it
            float penetration_relaxation = 1f;
            minNorm *= penetration_relaxation;

            if (res.m_hasResult)
            {

                pa = res.m_pointInWorld - minNorm * correctedMinNorm;
                pb = res.m_pointInWorld;
                v = minNorm;

#if DEBUG_DRAW
		        if (debugDraw != null)
		        {
			        IndexedVector3 color = new IndexedVector3(1,0,0);
			        debugDraw.drawLine(pa,pb,color);
		        }
#endif//DEBUG_DRAW


            }

            BulletGlobals.GjkPairDetectorPool.Free(gjkdet);
            return res.m_hasResult;
        }

        private const int NUM_UNITSPHERE_POINTS = 42;
        private readonly static IndexedVector3[] sPenetrationDirections = {
            new IndexedVector3(0.000000f , -0.000000f,-1.000000f),
            new IndexedVector3(0.723608f , -0.525725f,-0.447219f),
            new IndexedVector3(-0.276388f , -0.850649f,-0.447219f),
            new IndexedVector3(-0.894426f , -0.000000f,-0.447216f),
            new IndexedVector3(-0.276388f , 0.850649f,-0.447220f),
            new IndexedVector3(0.723608f , 0.525725f,-0.447219f),
            new IndexedVector3(0.276388f , -0.850649f,0.447220f),
            new IndexedVector3(-0.723608f , -0.525725f,0.447219f),
            new IndexedVector3(-0.723608f , 0.525725f,0.447219f),
            new IndexedVector3(0.276388f , 0.850649f,0.447219f),
            new IndexedVector3(0.894426f , 0.000000f,0.447216f),
            new IndexedVector3(-0.000000f , 0.000000f,1.000000f),
            new IndexedVector3(0.425323f , -0.309011f,-0.850654f),
            new IndexedVector3(-0.162456f , -0.499995f,-0.850654f),
            new IndexedVector3(0.262869f , -0.809012f,-0.525738f),
            new IndexedVector3(0.425323f , 0.309011f,-0.850654f),
            new IndexedVector3(0.850648f , -0.000000f,-0.525736f),
            new IndexedVector3(-0.525730f , -0.000000f,-0.850652f),
            new IndexedVector3(-0.688190f , -0.499997f,-0.525736f),
            new IndexedVector3(-0.162456f , 0.499995f,-0.850654f),
            new IndexedVector3(-0.688190f , 0.499997f,-0.525736f),
            new IndexedVector3(0.262869f , 0.809012f,-0.525738f),
            new IndexedVector3(0.951058f , 0.309013f,0.000000f),
            new IndexedVector3(0.951058f , -0.309013f,0.000000f),
            new IndexedVector3(0.587786f , -0.809017f,0.000000f),
            new IndexedVector3(0.000000f , -1.000000f,0.000000f),
            new IndexedVector3(-0.587786f , -0.809017f,0.000000f),
            new IndexedVector3(-0.951058f , -0.309013f,-0.000000f),
            new IndexedVector3(-0.951058f , 0.309013f,-0.000000f),
            new IndexedVector3(-0.587786f , 0.809017f,-0.000000f),
            new IndexedVector3(-0.000000f , 1.000000f,-0.000000f),
            new IndexedVector3(0.587786f , 0.809017f,-0.000000f),
            new IndexedVector3(0.688190f , -0.499997f,0.525736f),
            new IndexedVector3(-0.262869f , -0.809012f,0.525738f),
            new IndexedVector3(-0.850648f , 0.000000f,0.525736f),
            new IndexedVector3(-0.262869f , 0.809012f,0.525738f),
            new IndexedVector3(0.688190f , 0.499997f,0.525736f),
            new IndexedVector3(0.525730f , 0.000000f,0.850652f),
            new IndexedVector3(0.162456f , -0.499995f,0.850654f),
            new IndexedVector3(-0.425323f , -0.309011f,0.850654f),
            new IndexedVector3(-0.425323f , 0.309011f,0.850654f),
            new IndexedVector3(0.162456f , 0.499995f,0.850654f),
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero,
            IndexedVector3.Zero};
    }

    public class MinkowskiIntermediateResult : IDiscreteCollisionDetectorInterfaceResult
    {
        public MinkowskiIntermediateResult()
        {
            m_hasResult = false;
        }

        public virtual void SetShapeIdentifiersA(int partId0, int index0)
        {
        }

        public virtual void SetShapeIdentifiersB(int partId1, int index1)
        {
        }

        public void AddContactPoint(IndexedVector3 normalOnBInWorld, IndexedVector3 pointInWorld, float depth)
        {
            AddContactPoint(ref normalOnBInWorld, ref pointInWorld, depth);
        }

        public void AddContactPoint(ref IndexedVector3 normalOnBInWorld, ref IndexedVector3 pointInWorld, float depth)
        {
            m_normalOnBInWorld = normalOnBInWorld;
            m_pointInWorld = pointInWorld;
            m_depth = depth;
            m_hasResult = true;
        }

        public IndexedVector3 m_normalOnBInWorld;
        public IndexedVector3 m_pointInWorld;
        public float m_depth;
        public bool m_hasResult;

    }
}
