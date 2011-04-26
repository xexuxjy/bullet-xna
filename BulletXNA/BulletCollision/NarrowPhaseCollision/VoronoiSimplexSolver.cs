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

#define BT_USE_EQUAL_VERTEX_THRESHOLD
#define CATCH_DEGENERATE_TETRAHEDRON

using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision.NarrowPhaseCollision
{

    public class VoronoiSimplexSolver : ISimplexSolverInterface
    {
        public static bool debugVoronoiSimplex = false;

        public VoronoiSimplexSolver()
        {
            m_equalVertexThreshold = VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD;
        }

        public void SetEqualVertexThreshold(float threshold)
        {
            m_equalVertexThreshold = threshold;
        }

        public float GetEqualVertexThreshold()
        {
            return m_equalVertexThreshold;
        }

        public void RemoveVertex(int index)
        {
            Debug.Assert(m_numVertices > 0);
            m_numVertices--;
            m_simplexVectorW[index] = m_simplexVectorW[m_numVertices];
            m_simplexPointsP[index] = m_simplexPointsP[m_numVertices];
            m_simplexPointsQ[index] = m_simplexPointsQ[m_numVertices];
        }

        public void ReduceVertices(BitArray usedVerts)
        {
            if ((NumVertices() >= 4) && (!usedVerts.Get(3)))
                RemoveVertex(3);

            if ((NumVertices() >= 3) && (!usedVerts.Get(2)))
                RemoveVertex(2);

            if ((NumVertices() >= 2) && (!usedVerts.Get(1)))
                RemoveVertex(1);

            if ((NumVertices() >= 1) && (!usedVerts.Get(0)))
                RemoveVertex(0);
        }

        public bool UpdateClosestVectorAndPoints()
        {
            if (m_needsUpdate)
            {
                m_cachedBC.Reset();

                m_needsUpdate = false;

                switch (NumVertices())
                {
                    case 0:
                        m_cachedValidClosest = false;
                        break;
                    case 1:
                        {
                            m_cachedP1 = m_simplexPointsP[0];
                            m_cachedP2 = m_simplexPointsQ[0];
                            m_cachedV = m_cachedP1 - m_cachedP2; //== m_simplexVectorW[0]
                            m_cachedBC.Reset();
                            m_cachedBC.SetBarycentricCoordinates(1, 0, 0, 0);
                            m_cachedValidClosest = m_cachedBC.IsValid();
                            break;
                        };
                    case 2:
                        {
                            //closest point origin from line segment
                            Vector3 from = m_simplexVectorW[0];
                            Vector3 to = m_simplexVectorW[1];
                            Vector3 nearest;

                            Vector3 p = Vector3.Zero;
                            Vector3 diff = p - from;
                            Vector3 v = to - from;
                            float t = Vector3.Dot(v, diff);

                            if (t > 0)
                            {
                                float dotVV = Vector3.Dot(v, v);
                                if (t < dotVV)
                                {
                                    t /= dotVV;
                                    diff -= t * v;
                                    m_cachedBC.m_usedVertices.Set(0, true);
                                    m_cachedBC.m_usedVertices.Set(1, true);
                                }
                                else
                                {
                                    t = 1;
                                    diff -= v;
                                    //reduce to 1 point
                                    m_cachedBC.m_usedVertices.Set(1, true);
                                }
                            }
                            else
                            {
                                t = 0;
                                //reduce to 1 point
                                m_cachedBC.m_usedVertices.Set(0, true);
                            }
                            m_cachedBC.SetBarycentricCoordinates(1 - t, t, 0, 0);
                            nearest = from + t * v;

                            m_cachedP1 = m_simplexPointsP[0] + t * (m_simplexPointsP[1] - m_simplexPointsP[0]);
                            m_cachedP2 = m_simplexPointsQ[0] + t * (m_simplexPointsQ[1] - m_simplexPointsQ[0]);
                            m_cachedV = m_cachedP1 - m_cachedP2;

                            ReduceVertices(m_cachedBC.m_usedVertices);

                            m_cachedValidClosest = m_cachedBC.IsValid();
                            break;
                        }
                    case 3:
                        {
                            //closest point origin from triangle 
                            Vector3 p = Vector3.Zero;

                            Vector3 a = m_simplexVectorW[0];
                            Vector3 b = m_simplexVectorW[1];
                            Vector3 c = m_simplexVectorW[2];

                            ClosestPtPointTriangle(ref p, ref a, ref b, ref c, ref m_cachedBC);
                            m_cachedP1 = m_simplexPointsP[0] * m_cachedBC.m_barycentricCoords.X +
                            m_simplexPointsP[1] * m_cachedBC.m_barycentricCoords.Y +
                            m_simplexPointsP[2] * m_cachedBC.m_barycentricCoords.Z;

                            m_cachedP2 = m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoords.X +
                            m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoords.Y +
                            m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoords.Z;

                            m_cachedV = m_cachedP1 - m_cachedP2;

                            if (BulletGlobals.g_streamWriter != null && debugVoronoiSimplex)
                            {
                                BulletGlobals.g_streamWriter.WriteLine("voronoi update closest points case 3");
                                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "p", p);
                                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "a", a);
                                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "b", b);
                                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "c", c);
                                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "cachedp1", m_cachedP1);
                                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "cachedp2", m_cachedP2);
                                MathUtil.PrintVector4(BulletGlobals.g_streamWriter, "cachedp1", m_cachedBC.m_barycentricCoords);
                            }

                            ReduceVertices(m_cachedBC.m_usedVertices);
                            m_cachedValidClosest = m_cachedBC.IsValid();

                            break;
                        }
                    case 4:
                        {


                            Vector3 p = Vector3.Zero;

                            Vector3 a = m_simplexVectorW[0];
                            Vector3 b = m_simplexVectorW[1];
                            Vector3 c = m_simplexVectorW[2];
                            Vector3 d = m_simplexVectorW[3];

                            bool hasSeperation = ClosestPtPointTetrahedron(ref p, ref a, ref b, ref c, ref d, ref m_cachedBC);

                            if (hasSeperation)
                            {

                                m_cachedP1 = m_simplexPointsP[0] * m_cachedBC.m_barycentricCoords.X +
                                    m_simplexPointsP[1] * m_cachedBC.m_barycentricCoords.Y +
                                    m_simplexPointsP[2] * m_cachedBC.m_barycentricCoords.Z +
                                    m_simplexPointsP[3] * m_cachedBC.m_barycentricCoords.W;

                                m_cachedP2 = m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoords.X +
                                    m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoords.Y +
                                    m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoords.Z +
                                    m_simplexPointsQ[3] * m_cachedBC.m_barycentricCoords.W;

                                m_cachedV = m_cachedP1 - m_cachedP2;
                                ReduceVertices(m_cachedBC.m_usedVertices);
                            }
                            else
                            {
                                //					printf("sub distance got penetration\n");

                                if (m_cachedBC.m_degenerate)
                                {
                                    m_cachedValidClosest = false;
                                }
                                else
                                {
                                    m_cachedValidClosest = true;
                                    //degenerate case == false, penetration = true + zero
                                    m_cachedV = Vector3.Zero;
                                }
                                break;
                            }

                            m_cachedValidClosest = m_cachedBC.IsValid();

                            //closest point origin from tetrahedron
                            break;
                        }
                    default:
                        {
                            m_cachedValidClosest = false;
                            break;
                        }
                };
            }

            if (BulletGlobals.g_streamWriter != null && debugVoronoiSimplex)
            {
                BulletGlobals.g_streamWriter.WriteLine("voronoi update closest points");
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "cachedp1", m_cachedP1);
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "cachedp2", m_cachedP2);
                MathUtil.PrintVector4(BulletGlobals.g_streamWriter, "cachedbarry", m_cachedBC.m_barycentricCoords);
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "cachedV", m_cachedV);

            }



            return m_cachedValidClosest;
        }

        public bool ClosestPtPointTetrahedron(ref Vector3 p, ref Vector3 a, ref Vector3 b, ref Vector3 c, ref Vector3 d, ref SubSimplexClosestResult finalResult)
        {
            SubSimplexClosestResult tempResult = new SubSimplexClosestResult();

            // Start ref assuming point inside all halfspaces, so closest to itself
            finalResult.m_closestPointOnSimplex = p;
            finalResult.m_usedVertices.SetAll(true);

            int pointOutsideABC = PointOutsideOfPlane(ref p, ref a, ref b, ref c, ref d);
            int pointOutsideACD = PointOutsideOfPlane(ref p, ref a, ref c, ref d, ref b);
            int pointOutsideADB = PointOutsideOfPlane(ref p, ref a, ref d, ref b, ref c);
            int pointOutsideBDC = PointOutsideOfPlane(ref p, ref b, ref d, ref c, ref a);

            if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0)
            {
                finalResult.m_degenerate = true;
                return false;
            }

            if (pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0)
            {
                return false;
            }


            float bestSqDist = float.MaxValue;
            // If point outside face abc then compute closest point on abc
            if (pointOutsideABC != 0)
            {
                ClosestPtPointTriangle(ref p, ref a, ref b, ref c, ref tempResult);
                Vector3 q = tempResult.m_closestPointOnSimplex;

                float sqDist = (q - p).LengthSquared();
                // Update best closest point if (squared) distance is less than current best
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    finalResult.m_closestPointOnSimplex = q;
                    //convert result bitmask!
                    finalResult.m_usedVertices.SetAll(false);
                    finalResult.m_usedVertices.Set(0, tempResult.m_usedVertices.Get(0));
                    finalResult.m_usedVertices.Set(1, tempResult.m_usedVertices.Get(1));
                    finalResult.m_usedVertices.Set(2, tempResult.m_usedVertices.Get(2));
                    finalResult.SetBarycentricCoordinates(
                            tempResult.m_barycentricCoords.X,
                            tempResult.m_barycentricCoords.Y,
                            tempResult.m_barycentricCoords.Z,
                            0
                    );

                }
            }


            // Repeat test for face acd
            if (pointOutsideACD != 0)
            {
                ClosestPtPointTriangle(ref p, ref a, ref c, ref d, ref tempResult);
                Vector3 q = tempResult.m_closestPointOnSimplex;

                float sqDist = (q - p).LengthSquared();
                // Update best closest point if (squared) distance is less than current best
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    finalResult.m_closestPointOnSimplex = q;
                    //convert result bitmask!
                    finalResult.m_usedVertices.SetAll(false);
                    finalResult.m_usedVertices.Set(0, tempResult.m_usedVertices.Get(0));
                    finalResult.m_usedVertices.Set(2, tempResult.m_usedVertices.Get(1));
                    finalResult.m_usedVertices.Set(3, tempResult.m_usedVertices.Get(2));
                    finalResult.SetBarycentricCoordinates(
                            tempResult.m_barycentricCoords.X,
                            0,
                            tempResult.m_barycentricCoords.Y,
                            tempResult.m_barycentricCoords.Z);
                }
            }
            // Repeat test for face adb


            if (pointOutsideADB != 0)
            {
                ClosestPtPointTriangle(ref p, ref a, ref d, ref b, ref tempResult);
                Vector3 q = tempResult.m_closestPointOnSimplex;

                float sqDist = (q - p).LengthSquared();
                // Update best closest point if (squared) distance is less than current best
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    finalResult.m_closestPointOnSimplex = q;
                    //convert result bitmask!
                    finalResult.m_usedVertices.SetAll(false);
                    finalResult.m_usedVertices.Set(0, tempResult.m_usedVertices.Get(0));
                    finalResult.m_usedVertices.Set(1, tempResult.m_usedVertices.Get(2));
                    finalResult.m_usedVertices.Set(3, tempResult.m_usedVertices.Get(1));
                    finalResult.SetBarycentricCoordinates(
                            tempResult.m_barycentricCoords.X,
                            tempResult.m_barycentricCoords.Z,
                            0,
                            tempResult.m_barycentricCoords.Y);
                }
            }
            // Repeat test for face bdc


            if (pointOutsideBDC != 0)
            {
                ClosestPtPointTriangle(ref p, ref b, ref d, ref c, ref tempResult);
                Vector3 q = tempResult.m_closestPointOnSimplex;

                float sqDist = (q - p).LengthSquared();
                // Update best closest point if (squared) distance is less than current best
                if (sqDist < bestSqDist)
                {
                    bestSqDist = sqDist;
                    finalResult.m_closestPointOnSimplex = q;
                    //convert result bitmask!
                    finalResult.m_usedVertices.SetAll(false);
                    finalResult.m_usedVertices.Set(1, tempResult.m_usedVertices.Get(0));
                    finalResult.m_usedVertices.Set(2, tempResult.m_usedVertices.Get(2));
                    finalResult.m_usedVertices.Set(3, tempResult.m_usedVertices.Get(1));
                    finalResult.SetBarycentricCoordinates(
                            0,
                            tempResult.m_barycentricCoords.X,
                            tempResult.m_barycentricCoords.Z,
                            tempResult.m_barycentricCoords.Y);
                }

            }

            //help! we ended up full !

            if (finalResult.m_usedVertices.Get(0) &&
                finalResult.m_usedVertices.Get(1) &&
                finalResult.m_usedVertices.Get(2) &&
                finalResult.m_usedVertices.Get(3))
            {
                return true;
            }

            return true;

        }

        public int PointOutsideOfPlane(ref Vector3 p, ref Vector3 a, ref Vector3 b, ref Vector3 c, ref Vector3 d)
        {
            Vector3 normal = Vector3.Cross((b - a), (c - a));

            float signp = Vector3.Dot((p - a), normal); // [AP AB AC]
            float signd = Vector3.Dot((d - a), normal); // [AD AB AC]

#if CATCH_DEGENERATE_TETRAHEDRON
            if (signd * signd < (1e-4f * 1e-4f))
            {
        //		printf("affine dependent/degenerate\n");//
                return -1;
            }
#endif
            // Points on opposite sides if expression signs are opposite
            return (signp * signd) < 0f ?1:0;
        }

        public bool ClosestPtPointTriangle(ref Vector3 p, ref Vector3 a, ref Vector3 b, ref Vector3 c, ref SubSimplexClosestResult result)
        {
            result.m_usedVertices.SetAll(false);

            // Check if P in vertex region outside A
            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3 ap = p - a;
            float d1 = Vector3.Dot(ab, ap);
            float d2 = Vector3.Dot(ac, ap);
            if (d1 <= 0f && d2 <= 0f)
            {
                result.m_closestPointOnSimplex = a;
                result.m_usedVertices.Set(0, true);
                result.SetBarycentricCoordinates(1, 0, 0, 0);
                return true;// a; // barycentric coordinates (1,0,0)
            }

            // Check if P in vertex region outside B
            Vector3 bp = p - b;
            float d3 = Vector3.Dot(ab, bp);
            float d4 = Vector3.Dot(ac, bp);
            if (d3 >= 0f && d4 <= d3)
            {
                result.m_closestPointOnSimplex = b;
                result.m_usedVertices.Set(1, true);
                result.SetBarycentricCoordinates(0, 1, 0, 0);

                return true; // b; // barycentric coordinates (0,1,0)
            }
            // Check if P in edge region of AB, if so return projection of P onto AB
            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float v = d1 / (d1 - d3);
                result.m_closestPointOnSimplex = a + v * ab;
                result.m_usedVertices.Set(0, true);
                result.m_usedVertices.Set(1, true);
                result.SetBarycentricCoordinates(1 - v, v, 0, 0);
                return true;
                //return a + v * ab; // barycentric coordinates (1-v,v,0)
            }

            // Check if P in vertex region outside C
            Vector3 cp = p - c;
            float d5 = Vector3.Dot(ab, cp);
            float d6 = Vector3.Dot(ac, cp);
            if (d6 >= 0f && d5 <= d6)
            {
                result.m_closestPointOnSimplex = c;
                result.m_usedVertices.Set(2, true);
                result.SetBarycentricCoordinates(0, 0, 1, 0);
                return true;//c; // barycentric coordinates (0,0,1)
            }

            // Check if P in edge region of AC, if so return projection of P onto AC
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float w = d2 / (d2 - d6);
                result.m_closestPointOnSimplex = a + w * ac;
                result.m_usedVertices.Set(0, true);
                result.m_usedVertices.Set(2, true);
                result.SetBarycentricCoordinates(1 - w, 0, w, 0);
                return true;
                //return a + w * ac; // barycentric coordinates (1-w,0,w)
            }

            // Check if P in edge region of BC, if so return projection of P onto BC
            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

                result.m_closestPointOnSimplex = b + w * (c - b);
                result.m_usedVertices.Set(1, true);
                result.m_usedVertices.Set(2, true);
                result.SetBarycentricCoordinates(0, 1 - w, w, 0);
                return true;
                // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
            }

            // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
            float denom = 1f / (va + vb + vc);
            float v1 = vb * denom;
            float w1 = vc * denom;

            result.m_closestPointOnSimplex = a + ab * v1 + ac * w1;
            result.m_usedVertices.Set(0, true);
            result.m_usedVertices.Set(1, true);
            result.m_usedVertices.Set(2, true);
            result.SetBarycentricCoordinates(1 - v1 - w1, v1, w1, 0);

            return true;
            //	return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = float(1.0) - v - w

        }

        public void Reset()
        {
            m_cachedValidClosest = false;
            m_numVertices = 0;
            m_needsUpdate = true;
            m_lastW = MathUtil.MAX_VECTOR;
            m_cachedBC.Reset();
        }

        public void AddVertex(ref Vector3 w, ref Vector3 p, ref Vector3 q)
        {
            m_lastW = w;
            m_needsUpdate = true;

            m_simplexVectorW[m_numVertices] = w;
            m_simplexPointsP[m_numVertices] = p;
            m_simplexPointsQ[m_numVertices] = q;

            m_numVertices++;
        }

        public bool Closest(ref Vector3 v)
        {
            if (BulletGlobals.g_streamWriter != null && debugVoronoiSimplex)
            {
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "Voronoi closest pre.", m_cachedV);
            }
            bool succes = UpdateClosestVectorAndPoints();
            v = m_cachedV;
            if (BulletGlobals.g_streamWriter != null && debugVoronoiSimplex)
            {
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "Voronoi closest post.", m_cachedV);
            }

            return succes;
        }

        public float MaxVertex()
        {
            int i, numverts = NumVertices();
            float maxV = 0f;
            for (i = 0; i < numverts; i++)
            {
                float curLen2 = m_simplexVectorW[i].LengthSquared();
                if (maxV < curLen2)
                {
                    maxV = curLen2;
                }
            }
            return maxV;
        }

        public bool FullSimplex()
        {
            return (m_numVertices == 4);
        }

        public int GetSimplex(IList<Vector3> pBuf, IList<Vector3> qBuf, IList<Vector3> yBuf)
        {
            int i;
            for (i = 0; i < NumVertices(); i++)
            {
                yBuf[i] = m_simplexVectorW[i];
                pBuf[i] = m_simplexPointsP[i];
                qBuf[i] = m_simplexPointsQ[i];
            }
            return NumVertices();
        }

        public bool InSimplex(ref Vector3 w)
        {
            bool found = false;
            int i, numverts = NumVertices();
            //float maxV = float(0.);

            //w is in the current (reduced) simplex
            for (i = 0; i < numverts; i++)
            {
#if BT_USE_EQUAL_VERTEX_THRESHOLD
                if ((w - m_simplexVectorW[i]).LengthSquared() <= m_equalVertexThreshold)
#else
                if (m_simplexVectorW[i] == w)
#endif
                {
                    found = true;
                }
            }

            //check in case lastW is already removed
            if (w == m_lastW)
            {
                return true;
            }

            return found;
        }

        public void BackupClosest(ref Vector3 v)
        {
            v = m_cachedV;
        }

        public bool EmptySimplex()
        {
            return (NumVertices() == 0);
        }

        public void ComputePoints(ref Vector3 p1, ref Vector3 p2)
        {
            UpdateClosestVectorAndPoints();
            p1 = m_cachedP1;
            p2 = m_cachedP2;
        }

        public int NumVertices()
        {
            return m_numVertices;
        }

        public int m_numVertices;
        public Vector3[] m_simplexVectorW = new Vector3[VORONOI_SIMPLEX_MAX_VERTS];
        public Vector3[] m_simplexPointsP = new Vector3[VORONOI_SIMPLEX_MAX_VERTS];
        public Vector3[] m_simplexPointsQ = new Vector3[VORONOI_SIMPLEX_MAX_VERTS];

        public Vector3 m_cachedP1;
        public Vector3 m_cachedP2;
        public Vector3 m_cachedV;
        public Vector3 m_lastW;
        public bool m_cachedValidClosest;
        public float m_equalVertexThreshold;


        public SubSimplexClosestResult m_cachedBC = new SubSimplexClosestResult();

        public bool m_needsUpdate;
        public const int VORONOI_SIMPLEX_MAX_VERTS = 5;
        public const float VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD = 0.0001f;

    }

    public class SubSimplexClosestResult
    {
        public Vector3 m_closestPointOnSimplex;
        //MASK for m_usedVertices
        //stores the simplex vertex-usage, using the MASK, 
        // if m_usedVertices & MASK then the related vertex is used
        public BitArray m_usedVertices = new BitArray(4);
        //public float[] m_barycentricCoords = new float[4];
        public Vector4 m_barycentricCoords = new Vector4();
        public bool m_degenerate;

        public void Reset()
        {
            m_degenerate = false;
            SetBarycentricCoordinates(0f, 0f, 0f, 0f);
            m_usedVertices.SetAll(false);
        }
        public bool IsValid()
        {
            bool valid = (m_barycentricCoords.X >= 0f) &&
                (m_barycentricCoords.Y >= 0f) &&
                (m_barycentricCoords.Z >= 0f) &&
                (m_barycentricCoords.W >= 0f);
            return valid;
        }
        public void SetBarycentricCoordinates(float a, float b, float c, float d)
        {
            m_barycentricCoords.X = a;
            m_barycentricCoords.Y = b;
            m_barycentricCoords.Z = c;
            m_barycentricCoords.W = d;
        }
    }
}