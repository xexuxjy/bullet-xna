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
    public static class BoxBoxDetector
    {

        //public BoxBoxDetector(BoxShape box1, BoxShape box2)
        //{
        //    m_box1 = box1;
        //    m_box2 = box2;
        //}


        // Work in progress to copy redo the box detector to remove un-necessary allocations

        public static void GetClosestPoints(BoxShape box1,BoxShape box2, ref ClosestPointInput input, ManifoldResult output, IDebugDraw debugDraw, bool swapResults)
        {
            IndexedMatrix transformA = input.m_transformA;
            IndexedMatrix transformB = input.m_transformB;

#if DEBUG
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugBoxBoxDetector)
            {
                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "BoxBox:GCP:transformA", transformA);
                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "BoxBox:GCP:transformB", transformB);
            }
#endif


            int skip = 0;
            Object contact = null;

            IndexedVector3 normal = new IndexedVector3();
            float depth = 0f;
            int return_code = -1;
            int maxc = 4;

            IndexedVector3 translationA = new IndexedVector3(transformA._origin);
            IndexedVector3 translationB = new IndexedVector3(transformB._origin);

            //IndexedVector3 debugExtents = new IndexedVector3(2f, 2f, 2f);

            IndexedVector3 box1Margin = new IndexedVector3(2f * box1.GetHalfExtentsWithMargin());
            IndexedVector3 box2Margin = new IndexedVector3(2f * box2.GetHalfExtentsWithMargin());

            //IndexedVector3 box1Margin = 2f * debugExtents;
            //IndexedVector3 box2Margin = 2f * debugExtents;

            IndexedBasisMatrix rotateA = transformA._basis.Transpose();
            IndexedBasisMatrix rotateB = transformB._basis.Transpose();

            for (int j = 0; j < 3; j++)
            {
                s_temp1[0 + 4 * j] = transformA._basis[j].X;
                s_temp2[0 + 4 * j] = transformB._basis[j].X;

                s_temp1[1 + 4 * j] = transformA._basis[j].Y;
                s_temp2[1 + 4 * j] = transformB._basis[j].Y;


                s_temp1[2 + 4 * j] = transformA._basis[j].Z;
                s_temp2[2 + 4 * j] = transformB._basis[j].Z;

            }

            //s_temp1[0] = rotateA._Row0.X;
            //s_temp1[1] = rotateA._Row0.Y;
            //s_temp1[2] = rotateA._Row0.Z;

            //s_temp1[4] = rotateA._Row1.X;
            //s_temp1[5] = rotateA._Row1.Y;
            //s_temp1[6] = rotateA._Row1.Z;

            //s_temp1[8] = rotateA._Row2.X;
            //s_temp1[9] = rotateA._Row2.X;
            //s_temp1[10] = rotateA._Row2.X;


            //s_temp2[0] = rotateB._Row0.X;
            //s_temp2[1] = rotateB._Row0.Y;
            //s_temp2[2] = rotateB._Row0.Z;

            //s_temp2[4] = rotateB._Row1.X;
            //s_temp2[5] = rotateB._Row1.Y;
            //s_temp2[6] = rotateB._Row1.Z;

            //s_temp2[8] = rotateB._Row2.X;
            //s_temp2[9] = rotateB._Row2.Y;
            //s_temp2[10] = rotateB._Row2.Z;

            DBoxBox2(ref translationA,
            s_temp1,
            ref box1Margin,
            ref translationB,
            s_temp2,
            ref box2Margin,
            ref normal, ref depth, ref return_code,
            maxc, contact, skip,
            output);

        }

        private static int DBoxBox2(ref IndexedVector3 p1, float[] R1,
        ref IndexedVector3 side1, ref IndexedVector3 p2,
        float[] R2, ref IndexedVector3 side2,
        ref IndexedVector3 normal, ref float depth, ref int return_code,
        int maxc, Object contact, int skip, IDiscreteCollisionDetectorInterfaceResult output)
        {
            //IndexedVector3 centerDifference = IndexedVector3.Zero, ppv = IndexedVector3.Zero;
            float[] normalR = null;
            int normalROffsetResult = 0;


            IndexedVector3 A = side1 * 0.5f;
            IndexedVector3 B = side2 * 0.5f;

            int code;
            bool invert_normal;
            // get vector from centers of box 1 to box 2, relative to box 1
            IndexedVector3 p = p2 - p1;

            IndexedVector3 pp = new IndexedVector3();

            DMULTIPLY1_331(ref pp, R1, ref p);		// get pp = p relative to body 1

            // for all 15 possible separating axes:
            //   * see if the axis separates the boxes. if so, return 0.
            //   * find the depth of the penetration along the separating axis (s2)
            //   * if this is the largest depth so far, record it.
            // the normal vector will be set to the separating axis with the smallest
            // depth. note: normalR is set to point to a column of R1 or R2 if that is
            // the smallest depth normal so far. otherwise normalR is 0 and normalC is
            // set to a vector relative to body 1. invert_normal is 1 if the sign of
            // the normal should be flipped.

            float R11 = DDOT44(R1, 0, R2, 0);
            float R12 = DDOT44(R1, 0, R2, 1);
            float R13 = DDOT44(R1, 0, R2, 2);
            float R21 = DDOT44(R1, 1, R2, 0);
            float R22 = DDOT44(R1, 1, R2, 1);
            float R23 = DDOT44(R1, 1, R2, 2);
            float R31 = DDOT44(R1, 2, R2, 0);
            float R32 = DDOT44(R1, 2, R2, 1);
            float R33 = DDOT44(R1, 2, R2, 2);

            float Q11 = Math.Abs(R11);
            float Q12 = Math.Abs(R12);
            float Q13 = Math.Abs(R13);

            float Q21 = Math.Abs(R21);
            float Q22 = Math.Abs(R22);
            float Q23 = Math.Abs(R23);

            float Q31 = Math.Abs(R31);
            float Q32 = Math.Abs(R32);
            float Q33 = Math.Abs(R33);

            float s = -float.MaxValue;
            invert_normal = false;
            code = 0;

            int normalROffset = 0;
            // separating axis = u1,u2,u3
            if (TST(pp.X, (A.X + B.X * Q11 + B.Y * Q12 + B.Z * Q13), R1, ref normalR, 0, ref normalROffset, 1, ref code, ref s, ref invert_normal)) return 0;
            if (TST(pp.Y, (A.Y + B.X * Q21 + B.Y * Q22 + B.Z * Q23), R1, ref normalR, 1, ref normalROffset, 2, ref code, ref s, ref invert_normal)) return 0;
            if (TST(pp.Z, (A.Z + B.X * Q31 + B.Y * Q32 + B.Z * Q33), R1, ref normalR, 2, ref normalROffset, 3, ref code, ref s, ref invert_normal)) return 0;

            // separating axis = v1,v2,v3
            if (TST(DDOT41(R2, 0, ref p, 0), (A.X * Q11 + A.Y * Q21 + A.Z * Q31 + B.X), R2, ref normalR, 0, ref normalROffset, 4, ref code, ref s, ref invert_normal)) return 0;
            if (TST(DDOT41(R2, 1, ref p, 0), (A.X * Q12 + A.Y * Q22 + A.Z * Q32 + B.Y), R2, ref normalR, 1, ref normalROffset, 5, ref code, ref s, ref invert_normal)) return 0;
            if (TST(DDOT41(R2, 2, ref p, 0), (A.X * Q13 + A.Y * Q23 + A.Z * Q33 + B.Z), R2, ref normalR, 2, ref normalROffset, 6, ref code, ref s, ref invert_normal)) return 0;

            // note: cross product axes need to be scaled when s is computed.
            // normal (n1,n2,n3) is relative to box 1.
            // separating axis = u1 x (v1,v2,v3)
            //private static bool TST2(float expr1,float expr2,ref IndexedVector3 normal, ref IndexedVector3 normalC,int cc,ref int code)

            IndexedVector3 normalC = new IndexedVector3();

            float fudge2 = 1.0e-5f;

            Q11 += fudge2;
            Q12 += fudge2;
            Q13 += fudge2;

            Q21 += fudge2;
            Q22 += fudge2;
            Q23 += fudge2;

            Q31 += fudge2;
            Q32 += fudge2;
            Q33 += fudge2;

            // separating axis = u1 x (v1,v2,v3)
            if (TST2(pp.Z * R21 - pp.Y * R31, (A.Y * Q31 + A.Z * Q21 + B.Y * Q13 + B.Z * Q12), 0, -R31, R21, ref normalC, ref normalR, 7, ref code, ref s, ref invert_normal)) return 0;
            if (TST2(pp.Z * R22 - pp.Y * R32, (A.Y * Q32 + A.Z * Q22 + B.X * Q13 + B.Z * Q11), 0, -R32, R22, ref normalC, ref normalR, 8, ref code, ref s, ref invert_normal)) return 0;
            if (TST2(pp.Z * R23 - pp.Y * R33, (A.Y * Q33 + A.Z * Q23 + B.X * Q12 + B.Y * Q11), 0, -R33, R23, ref normalC, ref normalR, 9, ref code, ref s, ref invert_normal)) return 0;

            // separating axis = u2 x (v1,v2,v3)
            if (TST2(pp.X * R31 - pp.Z * R11, (A.X * Q31 + A.Z * Q11 + B.Y * Q23 + B.Z * Q22), R31, 0, -R11, ref normalC, ref normalR, 10, ref code, ref s, ref invert_normal)) return 0;
            if (TST2(pp.X * R32 - pp.Z * R12, (A.X * Q32 + A.Z * Q12 + B.X * Q23 + B.Z * Q21), R32, 0, -R12, ref normalC, ref normalR, 11, ref code, ref s, ref invert_normal)) return 0;
            if (TST2(pp.X * R33 - pp.Z * R13, (A.X * Q33 + A.Z * Q13 + B.X * Q22 + B.Y * Q21), R33, 0, -R13, ref normalC, ref normalR, 12, ref code, ref s, ref invert_normal)) return 0;

            // separating axis = u3 x (v1,v2,v3)
            if (TST2(pp.Y * R11 - pp.X * R21, (A.X * Q21 + A.Y * Q11 + B.Y * Q33 + B.Z * Q32), -R21, R11, 0, ref normalC, ref normalR, 13, ref code, ref s, ref invert_normal)) return 0;
            if (TST2(pp.Y * R12 - pp.X * R22, (A.X * Q22 + A.Y * Q12 + B.X * Q33 + B.Z * Q31), -R22, R12, 0, ref normalC, ref normalR, 14, ref code, ref s, ref invert_normal)) return 0;
            if (TST2(pp.Y * R13 - pp.X * R23, (A.X * Q23 + A.Y * Q13 + B.X * Q32 + B.Y * Q31), -R23, R13, 0, ref normalC, ref normalR, 15, ref code, ref s, ref invert_normal)) return 0;

            if (code == 0)
            {
                return 0;
            }
            // if we get to this point, the boxes interpenetrate. compute the normal
            // in global coordinates.
            if (normalR != null)
            {
                normal.X = normalR[0 + normalROffset];
                normal.Y = normalR[4 + normalROffset];
                normal.Z = normalR[8 + normalROffset];
            }
            else
            {
                DMULTIPLY0_331(ref normal, R1, ref normalC);
            }
            if (invert_normal)
            {
                normal = -normal;
            }
            depth = -s;

            // compute contact point(s)

            if (code > 6)
            {
                // an edge from box 1 touches an edge from box 2.
                // find a point pa on the intersecting edge of box 1
                IndexedVector3 pa1 = p1;

                for (int j = 0; j < 3; j++)
                {
                    float sign = (DDOT14(ref normal, 0, R1, j) > 0) ? 1.0f : -1.0f;
                    for (int i = 0; i < 3; i++)
                    {
                        pa1[i] += sign * A[j] * R1[i * 4 + j];
                    }
                }

                // find a point pb on the intersecting edge of box 2
                IndexedVector3 pb1 = p2;
                //for (i = 0; i < 3; i++) pb[i] = p2[i];
                for (int j = 0; j < 3; j++)
                {
                    float sign = (DDOT14(ref normal, 0, R2, j) > 0) ? -1.0f : 1.0f;
                    for (int i = 0; i < 3; i++)
                    {
                        pb1[i] += sign * B[j] * R2[i * 4 + j];
                    }
                }


                float alpha, beta;
                IndexedVector3 ua = new IndexedVector3();
                IndexedVector3 ub = new IndexedVector3();
                for (int i = 0; i < 3; i++)
                {
                    ua[i] = R1[((code) - 7) / 3 + i * 4];
                }
                for (int i = 0; i < 3; i++)
                {
                    ub[i] = R2[((code) - 7) % 3 + i * 4];
                }

                DLineClosestApproach(ref pa1, ref ua, ref pb1, ref ub, out alpha, out beta);

                for (int i = 0; i < 3; i++)
                {
                    pa1[i] += ua[i] * alpha;
                }
                for (int i = 0; i < 3; i++)
                {
                    pb1[i] += ub[i] * beta;
                }

                {
                    //contact[0].pos[i] = float(0.5)*(pa[i]+pb[i]);
                    //contact[0].depth = *depth;

#if USE_CENTER_POINT
            pointInWorld = (pa + pb) * 0.5f;
            output.addContactPoint(-normal,pointInWorld,-depth);
#else
                    output.AddContactPoint(-normal, pb1, -depth);
#endif //
                    return_code = code;
                }
                return 1;
            }

            // okay, we have a face-something intersection (because the separating
            // axis is perpendicular to a face). define face 'a' to be the reference
            // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
            // the incident face (the closest face of the other box).


            float[] Ra;
            float[] Rb;
            IndexedVector3 pa;
            IndexedVector3 pb;
            IndexedVector3 Sa;
            IndexedVector3 Sb;

            if (code <= 3)
            {
                Ra = R1;
                Rb = R2;
                pa = p1;
                pb = p2;
                Sa = A;
                Sb = B;
            }
            else
            {
                Ra = R2;
                Rb = R1;
                pa = p2;
                pb = p1;
                Sa = B;
                Sb = A;
            }

            // nr = normal vector of reference face dotted with axes of incident box.
            // anr = absolute values of nr.
            IndexedVector3 normal2;
            IndexedVector3 nr = new IndexedVector3();
            IndexedVector3 anr;


            if (code <= 3)
            {
                normal2 = normal;
            }
            else
            {
                normal2 = -normal;
            }
            DMULTIPLY1_331(ref nr, Rb, ref normal2);

            nr.Abs(out anr);

            // find the largest compontent of anr: this corresponds to the normal
            // for the indident face. the other axis numbers of the indicent face
            // are stored in a1,a2.
            int lanr, a1, a2;
            if (anr.Y > anr.X)
            {
                if (anr.Y > anr.Z)
                {
                    a1 = 0;
                    lanr = 1;
                    a2 = 2;
                }
                else
                {
                    a1 = 0;
                    a2 = 1;
                    lanr = 2;
                }
            }
            else
            {
                if (anr.X > anr.Z)
                {
                    lanr = 0;
                    a1 = 1;
                    a2 = 2;
                }
                else
                {
                    a1 = 0;
                    a2 = 1;
                    lanr = 2;
                }
            }

            // compute center point of incident face, in reference-face coordinates
            IndexedVector3 center;
            if (nr[lanr] < 0)
            {
                center = new IndexedVector3(
                    pb.X - pa.X + Sb[lanr] * Rb[lanr],
                    pb.Y - pa.Y + Sb[lanr] * Rb[lanr + 4],
                    pb.Z - pa.Z + Sb[lanr] * Rb[lanr + 8]
                );
            }
            else
            {
                center = new IndexedVector3(
                    pb.X - pa.X - Sb[lanr] * Rb[lanr],
                    pb.Y - pa.Y - Sb[lanr] * Rb[lanr + 4],
                    pb.Z - pa.Z - Sb[lanr] * Rb[lanr + 8]
                );
            }


            // find the normal and non-normal axis numbers of the reference box
            int codeN, code1, code2;
            if (code <= 3)
            {
                codeN = code - 1;
            }
            else
            {
                codeN = code - 4;
            }
            if (codeN == 0)
            {
                code1 = 1;
                code2 = 2;
            }
            else if (codeN == 1)
            {
                code1 = 0;
                code2 = 2;
            }
            else
            {
                code1 = 0;
                code2 = 1;
            }

            // find the four corners of the incident face, in reference-face coordinates
            float[] quad = s_quad;	// 2D coordinate of incident face (x,y pairs)
            float c1, c2, m11, m12, m21, m22;
            c1 = DDOT14(ref center, 0, Ra, code1);
            c2 = DDOT14(ref center, 0, Ra, code2);

            // optimize this? - we have already computed this data above, but it is not
            // stored in an easy-to-index format. for now it's quicker just to recompute
            // the four dot products.
            m11 = DDOT44(Ra, code1, Rb, a1);
            m12 = DDOT44(Ra, code1, Rb, a2);
            m21 = DDOT44(Ra, code2, Rb, a1);
            m22 = DDOT44(Ra, code2, Rb, a2);
            {
                float k1 = m11 * Sb[a1];
                float k2 = m21 * Sb[a1];
                float k3 = m12 * Sb[a2];
                float k4 = m22 * Sb[a2];
                quad[0] = c1 - k1 - k3;
                quad[1] = c2 - k2 - k4;
                quad[2] = c1 - k1 + k3;
                quad[3] = c2 - k2 + k4;
                quad[4] = c1 + k1 + k3;
                quad[5] = c2 + k2 + k4;
                quad[6] = c1 + k1 - k3;
                quad[7] = c2 + k2 - k4;
            }

            // find the size of the reference face
            //float[] s_rectReferenceFace = new float[2];
            s_rectReferenceFace[0] = Sa[code1];
            s_rectReferenceFace[1] = Sa[code2];

            // intersect the incident and reference faces
            float[] ret = s_ret;
            int n = IntersectRectQuad2(s_rectReferenceFace, quad, ret);
            if (n < 1)
            {
                return 0;		// this should never happen
            }

            // convert the intersection points into reference-face coordinates,
            // and compute the contact position and depth for each point. only keep
            // those points that have a positive (penetrating) depth. delete points in
            // the 'ret' array as necessary so that 'point' and 'ret' correspond.
            float[] point = s_point;		// penetrating contact points
            float[] dep = s_dep;			// depths for those points
            float det1 = 1f / (m11 * m22 - m12 * m21);
            m11 *= det1;
            m12 *= det1;
            m21 *= det1;
            m22 *= det1;
            int cnum = 0;			// number of penetrating contact points found
            for (int j = 0; j < n; j++)
            {
                float k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
                float k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
                for (int i = 0; i < 3; i++)
                {
                    point[cnum * 3 + i] = center[i] + k1 * Rb[i * 4 + a1] + k2 * Rb[i * 4 + a2];
                }
                dep[cnum] = Sa[codeN] - DDOT(ref normal2, 0, point, cnum * 3);
                if (dep[cnum] >= 0)
                {
                    ret[cnum * 2] = ret[j * 2];
                    ret[cnum * 2 + 1] = ret[j * 2 + 1];
                    cnum++;
                }
            }
            if (cnum < 1)
            {
                return 0;	// this should never happen
            }

            // we can't generate more contacts than we actually have
            if (maxc > cnum)
            {
                maxc = cnum;
            }
            if (maxc < 1)
            {
                maxc = 1;
            }

            if (cnum <= maxc)
            {
                if (code < 4)
                {
                    // we have less contacts than we need, so we use them all
                    for (int j = 0; j < cnum; j++)
                    {
                        IndexedVector3 pointInWorldFA = new IndexedVector3(); ;
                        for (int i = 0; i < 3; i++)
                        {
                            pointInWorldFA[i] = point[j * 3 + i] + pa[i];
                        }
#if DEBUG                        
						if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugBoxBoxDetector)
                        {
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "boxbox get closest", pointInWorldFA);
                        }
#endif                        

                        output.AddContactPoint(-normal, pointInWorldFA, -dep[j]);
                    }
                }
                else
                {
                    // we have less contacts than we need, so we use them all
                    for (int j = 0; j < cnum; j++)
                    {
                        IndexedVector3 pointInWorld = new IndexedVector3(); ;
                        for (int i = 0; i < 3; i++)
                        {
                            pointInWorld[i] = point[j * 3 + i] + pa[i];

                        }
#if DEBUG
						if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugBoxBoxDetector)
                        {
                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "boxbox get closest", pointInWorld);
                        }
#endif                        
                        output.AddContactPoint(-normal, pointInWorld, -dep[j]);
                    }
                }
            }
            else
            {
                // we have more contacts than are wanted, some of them must be culled.
                // find the deepest point, it is always the first contact.
                int i1 = 0;
                float maxdepth = dep[0];
                for (int i = 1; i < cnum; i++)
                {
                    if (dep[i] > maxdepth)
                    {
                        maxdepth = dep[i];
                        i1 = i;
                    }
                }

                int[] iret = new int[8];
                CullPoints2(cnum, ret, maxc, i1, iret);

                for (int j = 0; j < maxc; j++)
                {
                    //      dContactGeom *con = CONTACT(contact,skip*j);
                    //    for (i=0; i<3; i++) con->pos[i] = point[iret[j]*3+i] + pa[i];
                    //  con->depth = dep[iret[j]];
                    IndexedVector3 posInWorldFA = new IndexedVector3(); ;
                    for (int i = 0; i < 3; i++)
                    {
                        posInWorldFA[i] = point[iret[j] * 3 + i] + pa[i];
                    }
                    IndexedVector3 pointInWorld = posInWorldFA;

#if DEBUG
					if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugBoxBoxDetector)
                    {
                        MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "boxbox get closest", pointInWorld);
                    }
#endif
                    output.AddContactPoint((-normal), pointInWorld, -dep[iret[j]]);

                }
                cnum = maxc;
            }
            return_code = code;
            return cnum;
        }

        private static void DLineClosestApproach(ref IndexedVector3 pa, ref IndexedVector3 ua,
                       ref IndexedVector3 pb, ref IndexedVector3 ub,
                       out float alpha, out float beta)
        {
            IndexedVector3 p = pb - pa;

            float uaub = ua.Dot(ref ub);
            float q1 = ua.Dot(ref p);
            float q2 = -(ub.Dot(ref p));
            float d = 1 - uaub * uaub;
            if (d <= 0.0001f)
            {
                // @@@ this needs to be made more robust
                alpha = 0f;
                beta = 0f;
            }
            else
            {
                d = 1f / d;
                alpha = (q1 + uaub * q2) * d;
                beta = (uaub * q1 + q2) * d;
            }
        }

        // find all the intersection points between the 2D rectangle with vertices
        // at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
        // (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
        //
        // the intersection points are returned as x,y pairs in the 'ret' array.
        // the number of intersection points is returned by the function (this will
        // be in the range 0 to 8).

        static int IntersectRectQuad2(float[] h, float[] p, float[] ret)
        {
            // q (and r) contain nq (and nr) coordinate points for the current (and
            // chopped) polygons
            int nq = 4, nr = 0;
            float[] buffer = s_quadBuffer;
            float[] q = p;
            float[] r = ret;

            for (int dir = 0; dir <= 1; dir++)
            {
                // direction notation: xy[0] = x axis, xy[1] = y axis
                for (int sign = -1; sign <= 1; sign += 2)
                {
                    // chop q along the line xy[dir] = sign*h[dir]
                    float[] pq = q;
                    float[] pr = r;
                    int pqIndex = 0;
                    int prIndex = 0;
                    nr = 0;
                    for (int i = nq; i > 0; i--)
                    {
                        // go through all points in q and all lines between adjacent points
                        if (sign * pq[pqIndex + dir] < h[dir])
                        {
                            // this point is inside the chopping line
                            pr[prIndex] = pq[pqIndex];
                            pr[prIndex + 1] = pq[pqIndex + 1];
                            prIndex += 2;
                            nr++;
                            if ((nr & 8) != 0)
                            {
                                q = r;
                                goto done;
                            }
                        }
                        float nextQDir = 0f;
                        float nextQ1MinusDir = 0f;
                        if (i > 1)
                        {
                            nextQDir = pq[pqIndex + 2 + dir];
                            nextQ1MinusDir = pq[pqIndex + 2 + (1 - dir)];
                        }
                        else
                        {
                            nextQDir = q[dir];
                            nextQ1MinusDir = q[1 - dir];
                        }

                        if ((sign * pq[pqIndex + dir] < h[dir]) ^ (sign * nextQDir < h[dir]))
                        {
                            // this line crosses the chopping line
                            pr[prIndex + (1 - dir)] = pq[pqIndex + (1 - dir)] + (nextQ1MinusDir - pq[pqIndex + (1 - dir)]) /
                            (nextQDir - pq[pqIndex + dir]) * (sign * h[dir] - pq[pqIndex + dir]);
                            pr[prIndex + dir] = sign * h[dir];
                            prIndex += 2;
                            nr++;
                            if ((nr & 8) != 0)
                            {
                                q = r;
                                goto done;
                            }
                        }
                        pqIndex += 2;
                    }
                    q = r;
                    r = (q == ret) ? buffer : ret;
                    nq = nr;
                }
            }
        done:
            // data already in ret
            if (q != ret)
            {
                for (int i = 0; i < nr * 2; ++i)
                {
                    ret[i] = q[i];
                }
            }
            return nr;
        }

        // given n points in the plane (array p, of size 2*n), generate m points that
        // best represent the whole set. the definition of 'best' here is not
        // predetermined - the idea is to select points that give good box-box
        // collision detection behavior. the chosen point indexes are returned in the
        // array iret (of size m). 'i0' is always the first entry in the array.
        // n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
        // in the range [0..n-1].

        private static void CullPoints2(int n, float[] p, int m, int i0, int[] iret)
        {
            // compute the centroid of the polygon in cx,cy
            int iretIndex = 0;
            int i, j;
            float a, cx, cy, q;
            if (n == 1)
            {
                cx = p[0];
                cy = p[1];
            }
            else if (n == 2)
            {
                cx = 0.5f * (p[0] + p[2]);
                cy = 0.5f * (p[1] + p[3]);
            }
            else
            {
                a = 0;
                cx = 0;
                cy = 0;
                for (i = 0; i < (n - 1); i++)
                {
                    q = p[i * 2] * p[i * 2 + 3] - p[i * 2 + 2] * p[i * 2 + 1];
                    a += q;
                    cx += q * (p[i * 2] + p[i * 2 + 2]);
                    cy += q * (p[i * 2 + 1] + p[i * 2 + 3]);
                }
                q = p[n * 2 - 2] * p[1] - p[0] * p[n * 2 - 1];
                if (Math.Abs(a + q) > MathUtil.SIMD_EPSILON)
                {
                    a = 1f / (3.0f * (a + q));
                }
                else
                {
                    a = 1e30f;
                }
                cx = a * (cx + q * (p[n * 2 - 2] + p[0]));
                cy = a * (cy + q * (p[n * 2 - 1] + p[1]));
            }

            // compute the angle of each point w.r.t. the centroid
            float[] A = s_A;
            for (i = 0; i < n; i++)
            {
                A[i] = (float)Math.Atan2(p[i * 2 + 1] - cy, p[i * 2] - cx);
            }

            // search for points that have angles closest to A[i0] + i*(2*pi/m).
            for (i = 0; i < n; i++)
            {
                s_availablePoints[i] = 1;
            }
            s_availablePoints[i0] = 0;
            iret[0] = i0;
            iretIndex++;
            for (j = 1; j < m; j++)
            {
                a = j * (MathUtil.SIMD_2_PI / m) + A[i0];
                if (a > MathUtil.SIMD_PI)
                {
                    a -= MathUtil.SIMD_2_PI;
                }
                float maxdiff = float.MaxValue, diff;

                iret[iretIndex] = i0;			// iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0

                for (i = 0; i < n; i++)
                {
                    if (s_availablePoints[i] != 0)
                    {
                        diff = Math.Abs(A[i] - a);
                        if (diff > MathUtil.SIMD_PI)
                        {
                            diff = MathUtil.SIMD_2_PI - diff;
                        }
                        if (diff < maxdiff)
                        {
                            maxdiff = diff;
                            iret[iretIndex] = i;
                        }
                    }
                }
                //#if defined(DEBUG) || defined (_DEBUG)
                //    btAssert (*iret != i0);	// ensure iret got set
                //#endif
                s_availablePoints[iret[iretIndex]] = 0;
                iretIndex++;
            }
        }

        private static bool TST(float expr1, float expr2, float[] norm, ref float[] normalR, int offset, ref int normalROffset, int cc, ref int code, ref float s, ref bool invert_normal)
        {
            float s2 = Math.Abs(expr1) - expr2;
            if (s2 > 0)
            {
                return true;
            }
            if (s2 > s)
            {
                s = s2;
                normalR = norm;
                normalROffset = offset;
                invert_normal = expr1 < 0;
                code = cc;
            }
            return false;
        }

        // note: cross product axes need to be scaled when s is computed.
        // normal (n1,n2,n3) is relative to box 1.
        private static bool TST2(float expr1, float expr2, float n1, float n2, float n3, ref IndexedVector3 normalC, ref float[] normalR, int cc, ref int code, ref float s, ref bool invert_normal)
        {
            float s2 = Math.Abs(expr1) - (expr2);
            if (s2 > MathUtil.SIMD_EPSILON)
            {
                return true;
            }
            float l = (float)Math.Sqrt((n1 * n1) + (n2 * n2) + (n3 * n3));
            if (l > MathUtil.SIMD_EPSILON)
            {
                s2 /= l;
                if (s2 * fudge_factor > s)
                {
                    s = s2;
                    normalR = null;
                    normalC.X = (n1) / l; normalC.Y = (n2) / l; normalC.Z = (n3) / l;
                    invert_normal = ((expr1) < 0);
                    invert_normal = ((expr1) < 0);
                    code = (cc);
                }
            }
            return false;
        }

        //#define dDOTpq(a,b,p,q) ((a)[0]*(b)[0] + (a)[p]*(b)[q] + (a)[2*(p)]*(b)[2*(q)])

        private static float DDOT(ref IndexedVector3 a, int aOffset, ref IndexedVector3 b, int bOffset)
        {
            //return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
            //return DDOTpq(ref a, ref b, aOffset, bOffset, 1, 1);
            return a[aOffset] * b[bOffset] + a[1 + aOffset] * b[1 + bOffset] + a[2 + aOffset] * b[2 + bOffset];
        }

        private static float DDOT(ref IndexedVector3 a, int aOffset, float[] b, int bOffset)
        {
            //return DDOTpq(ref a, b, aOffset, bOffset, 1, 1);
            return (a[aOffset] * b[bOffset] + a[1 + aOffset] * b[1 + bOffset] + a[2 + aOffset] * b[2 + bOffset]);
        }

        private static float DDOT(float[] a, int aOffset, ref IndexedVector3 b, int bOffset)
        {
            //return DDOTpq(a, ref b, aOffset, bOffset, 1, 1);
            return (a[aOffset] * b[bOffset] + a[1 + aOffset] * b[1 + bOffset] + a[2 + aOffset] * b[2 + bOffset]);
        }


        private static float DDOT44(float[] a, int aOffset, float[] b, int bOffset)
        {
            //return DDOTpq(a, b, aOffset, bOffset, 4, 4);
            return (a[aOffset] * b[bOffset] + a[4 + aOffset] * b[4 + bOffset] + a[8 + aOffset] * b[8 + bOffset]);
        }

        private static float DDOT41(float[] a, int aOffset, float[] b, int bOffset)
        {
            //return DDOTpq(a, b, aOffset, bOffset, 4, 1);
            return (a[aOffset] * b[bOffset] + a[4 + aOffset] * b[1 + bOffset] + a[8 + aOffset] * b[2 + bOffset]);
        }
        
        private static float DDOT41(float[] a, int aOffset, ref IndexedVector3 b, int bOffset)
        {
            //return DDOTpq(a, ref b, aOffset, bOffset, 4, 1);
            return (a[aOffset] * b[bOffset] + a[4 + aOffset] * b[1 + bOffset] + a[8 + aOffset] * b[2 + bOffset]);
        }

        private static float DDOT14(float[] a, int aOffset, float[] b, int bOffset)
        {
            //return DDOTpq(a, b, aOffset, bOffset, 1, 4);
            return (a[aOffset] * b[bOffset] + a[1 + aOffset] * b[4 + bOffset] + a[2 + aOffset] * b[8 + bOffset]);
        }

        private static float DDOT14(ref IndexedVector3 a, int aOffset, float[] b, int bOffset)
        {
            //return DDOTpq(ref a, b, aOffset, bOffset, 1, 4);
            return (a[aOffset] * b[bOffset] + a[1 + aOffset] * b[4 + bOffset] + a[2 + aOffset] * b[8 + bOffset]);
        }


        public static void DMULTIPLY1_331(ref IndexedVector3 A, float[] B, ref IndexedVector3 C)
        {

            A.X = DDOT41(B, 0, ref C, 0);
            A.Y = DDOT41(B, 1, ref C, 0);
            A.Z = DDOT41(B, 2, ref C, 0);
        }


        public static void DMULTIPLY0_331(ref IndexedVector3 A, float[] B, ref IndexedVector3 C)
        {
            A.X = DDOT(B, 0, ref C, 0);
            A.Y = DDOT(B, 4, ref C, 0);
            A.Z = DDOT(B, 8, ref C, 0);
        }

        //public static void DMULTIPLY0_331(float[] A, float[] B, float[] C)
        //{
        //    A[0] = DDOT(B, 0, C, 0);
        //    A[1] = DDOT(B, 4, C, 0);
        //    A[2] = DDOT(B, 8, C, 0);
        //}

        //private BoxShape m_box1;
        //private BoxShape m_box2;
        private static float fudge_factor = 1.05f;

        private static float[] s_buffer = new float[12];
        private static float[] s_quadBuffer = new float[16];

        private static float[] s_temp1 = new float[12];
        private static float[] s_temp2 = new float[12];
        private static float[] s_quad = new float[8];
        private static float[] s_ret = new float[16];
        private static float[] s_point = new float[3 * 8];		// penetrating contact points
        private static float[] s_dep = new float[8];			// depths for those points
        private static float[] s_A = new float[8];
        private static float[] s_rectReferenceFace = new float[2];
        private static int[] s_availablePoints = new int[8];

    }

}
