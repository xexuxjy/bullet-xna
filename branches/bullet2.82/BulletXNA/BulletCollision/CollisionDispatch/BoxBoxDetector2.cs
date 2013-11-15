///*
// * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
// *
// * Bullet Continuous Collision Detection and Physics Library
// * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
// *
// * This software is provided 'as-is', without any express or implied warranty.
// * In no event will the authors be held liable for any damages arising from
// * the use of this software.
// * 
// * Permission is granted to anyone to use this software for any purpose, 
// * including commercial applications, and to alter it and redistribute it
// * freely, subject to the following restrictions:
// * 
// * 1. The origin of this software must not be misrepresented; you must not
// *    claim that you wrote the original software. If you use this software
// *    in a product, an acknowledgment in the product documentation would be
// *    appreciated but is not required.
// * 2. Altered source versions must be plainly marked as such, and must not be
// *    misrepresented as being the original software.
// * 3. This notice may not be removed or altered from any source distribution.
// */

//using System;
//
//using BulletXNA.BulletCollision.CollisionShapes;
//using BulletXNA.BulletCollision.NarrowPhaseCollision;
//using BulletXNA.LinearMath;
//using System.Collections.Generic;

//namespace BulletXNA.BulletCollision.CollisionDispatch
//{
//    public class BoxBoxDetector
//    {

//        public BoxBoxDetector(BoxShape box1, BoxShape box2)
//        {
//            m_box1 = box1;
//            m_box2 = box2;
//        }

//        public virtual void Cleanup()
//        {
//        }

//        public virtual void GetClosestPoints(ClosestPointInput input, ManifoldResult output, IDebugDraw debugDraw, bool swapResults)
//        {
//            IndexedMatrix transformA = input.m_transformA;
//            IndexedMatrix transformB = input.m_transformB;

//            if (BulletGlobals.g_streamWriter != null && debugBoxBox)
//            {
//                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "BoxBox:GCP:transformA", transformA);
//                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "BoxBox:GCP:transformB", transformB);
//            }



//            int skip = 0;
//            Object contact = null;
//            IndexedMatrix rotateA = IndexedMatrix.Identity;
//            rotateA.Backward = transformA.Backward;
//            rotateA.Right = transformA.Right;
//            rotateA.Up = transformA.Up;

//            IndexedMatrix rotateB = IndexedMatrix.Identity;
//            rotateB.Backward = transformB.Backward;
//            rotateB.Right = transformB.Right;
//            rotateB.Up = transformB.Up;

//            float[] normal = new float[3];
//            float depth = 0f;
//            int return_code = -1;
//            int maxc = 4;

//            IndexedVector3 translationA = transformA._origin;
//            IndexedVector3 translationB = transformB._origin;

//            IndexedVector3 debugExtents = new IndexedVector3(2f, 2f, 2f);

//            IndexedVector3 box1Margin = 2f * m_box1.GetHalfExtentsWithMargin();
//            IndexedVector3 box2Margin = 2f * m_box2.GetHalfExtentsWithMargin();

//            //IndexedVector3 box1Margin = 2f * debugExtents;
//            //IndexedVector3 box2Margin = 2f * debugExtents;
//            rotateA = IndexedMatrix.Transpose(rotateA);
//            rotateB = IndexedMatrix.Transpose(rotateB);

//            float[] temp1 = new float[12];
//            float[] temp2 = new float[12];

//            temp1[0] = rotateA.M11;
//            temp1[1] = rotateA.M12;
//            temp1[2] = rotateA.M13;

//            temp1[4] = rotateA.M21;
//            temp1[5] = rotateA.M22;
//            temp1[6] = rotateA.M23;

//            temp1[8] = rotateA.M31;
//            temp1[9] = rotateA.M32;
//            temp1[10] = rotateA.M33;


//            temp2[0] = rotateB.M11;
//            temp2[1] = rotateB.M12;
//            temp2[2] = rotateB.M13;

//            temp2[4] = rotateB.M21;
//            temp2[5] = rotateB.M22;
//            temp2[6] = rotateB.M23;

//            temp2[8] = rotateB.M31;
//            temp2[9] = rotateB.M32;
//            temp2[10] = rotateB.M33;

//            DBoxBox2(MathUtil.FloatFromVector3(ref translationA),
//            temp1,
//            ref box1Margin,
//            MathUtil.FloatFromVector3(ref translationB),
//            temp2,
//            ref box2Margin,
//            normal, ref depth, ref return_code,
//            maxc, contact, skip,
//            output);

//        }

//        private static int DBoxBox2(float[] p1, float[] R1,
//        ref IndexedVector3 side1, float[] p2,
//        float[] R2, ref IndexedVector3 side2,
//        float[] normal, ref float depth, ref int return_code,
//        int maxc, Object contact, int skip, IDiscreteCollisionDetectorInterfaceResult output)
//        {
//            IndexedVector3 centerDifference = IndexedVector3.Zero, ppv = IndexedVector3.Zero;
//            float[] normalR = null;
//            int normalROffsetResult = 0;


//            float[] A = MathUtil.FloatFromVector3(side1 * 0.5f);
//            float[] B = MathUtil.FloatFromVector3(side2 * 0.5f);

//            int code;
//            bool invert_normal;
//            // get vector from centers of box 1 to box 2, relative to box 1
//            float[] p = new float[3];
//            p[0] = p2[0] - p1[0];
//            p[1] = p2[1] - p1[1];
//            p[2] = p2[2] - p1[2];

//            float[] pp = new float[3];

//            DMULTIPLY1_331(pp, R1, p);		// get pp = p relative to body 1

//            // for all 15 possible separating axes:
//            //   * see if the axis separates the boxes. if so, return 0.
//            //   * find the depth of the penetration along the separating axis (s2)
//            //   * if this is the largest depth so far, record it.
//            // the normal vector will be set to the separating axis with the smallest
//            // depth. note: normalR is set to point to a column of R1 or R2 if that is
//            // the smallest depth normal so far. otherwise normalR is 0 and normalC is
//            // set to a vector relative to body 1. invert_normal is 1 if the sign of
//            // the normal should be flipped.

//            float R11 = DDOT44(R1, 0, R2, 0);
//            float R12 = DDOT44(R1, 0, R2, 1);
//            float R13 = DDOT44(R1, 0, R2, 2);
//            float R21 = DDOT44(R1, 1, R2, 0);
//            float R22 = DDOT44(R1, 1, R2, 1);
//            float R23 = DDOT44(R1, 1, R2, 2);
//            float R31 = DDOT44(R1, 2, R2, 0);
//            float R32 = DDOT44(R1, 2, R2, 1);
//            float R33 = DDOT44(R1, 2, R2, 2);

//            float Q11 = Math.Abs(R11);
//            float Q12 = Math.Abs(R12);
//            float Q13 = Math.Abs(R13);

//            float Q21 = Math.Abs(R21);
//            float Q22 = Math.Abs(R22);
//            float Q23 = Math.Abs(R23);

//            float Q31 = Math.Abs(R31);
//            float Q32 = Math.Abs(R32);
//            float Q33 = Math.Abs(R33);

//            float s = -float.MaxValue;
//            invert_normal = false;
//            code = 0;

//            //IndexedMatrix m1 = IndexedMatrix.Identity;
//            //IndexedMatrix m2 = IndexedMatrix.Identity;
//            //MathUtil.matrixFromFloat(ref m1, R1);
//            //MathUtil.matrixFromFloat(ref m2, R2); 

//            int normalROffset = 0;
//            // separating axis = u1,u2,u3
//            if (TST(pp[0], (A[0] + B[0] * Q11 + B[1] * Q12 + B[2] * Q13), R1, ref normalR, 0, ref normalROffset, 1, ref code, ref s, ref invert_normal)) return 0;
//            if (TST(pp[1], (A[1] + B[0] * Q21 + B[1] * Q22 + B[2] * Q23), R1, ref normalR, 1, ref normalROffset, 2, ref code, ref s, ref invert_normal)) return 0;
//            if (TST(pp[2], (A[2] + B[0] * Q31 + B[1] * Q32 + B[2] * Q33), R1, ref normalR, 2, ref normalROffset, 3, ref code, ref s, ref invert_normal)) return 0;

//            // separating axis = v1,v2,v3
//            if (TST(DDOT41(R2, 0, p, 0), (A[0] * Q11 + A[1] * Q21 + A[2] * Q31 + B[0]), R2, ref normalR, 0, ref normalROffset, 4, ref code, ref s, ref invert_normal)) return 0;
//            if (TST(DDOT41(R2, 1, p, 0), (A[0] * Q12 + A[1] * Q22 + A[2] * Q32 + B[1]), R2, ref normalR, 1, ref normalROffset, 5, ref code, ref s, ref invert_normal)) return 0;
//            if (TST(DDOT41(R2, 2, p, 0), (A[0] * Q13 + A[1] * Q23 + A[2] * Q33 + B[2]), R2, ref normalR, 2, ref normalROffset, 6, ref code, ref s, ref invert_normal)) return 0;

//            // note: cross product axes need to be scaled when s is computed.
//            // normal (n1,n2,n3) is relative to box 1.
//            // separating axis = u1 x (v1,v2,v3)
//            //private static bool TST2(float expr1,float expr2,ref IndexedVector3 normal, ref IndexedVector3 normalC,int cc,ref int code)

//            float[] normalC = new float[3];

//            // separating axis = u1 x (v1,v2,v3)
//            if (TST2(pp[2] * R21 - pp[1] * R31, (A[1] * Q31 + A[2] * Q21 + B[1] * Q13 + B[2] * Q12), 0, -R31, R21, normalC, ref normalR, 7, ref code, ref s, ref invert_normal)) return 0;
//            if (TST2(pp[2] * R22 - pp[1] * R32, (A[1] * Q32 + A[2] * Q22 + B[0] * Q13 + B[2] * Q11), 0, -R32, R22, normalC, ref normalR, 8, ref code, ref s, ref invert_normal)) return 0;
//            if (TST2(pp[2] * R23 - pp[1] * R33, (A[1] * Q33 + A[2] * Q23 + B[0] * Q12 + B[1] * Q11), 0, -R33, R23, normalC, ref normalR, 9, ref code, ref s, ref invert_normal)) return 0;

//            // separating axis = u2 x (v1,v2,v3)
//            if (TST2(pp[0] * R31 - pp[2] * R11, (A[0] * Q31 + A[2] * Q11 + B[1] * Q23 + B[2] * Q22), R31, 0, -R11, normalC, ref normalR, 10, ref code, ref s, ref invert_normal)) return 0;
//            if (TST2(pp[0] * R32 - pp[2] * R12, (A[0] * Q32 + A[2] * Q12 + B[0] * Q23 + B[2] * Q21), R32, 0, -R12, normalC, ref normalR, 11, ref code, ref s, ref invert_normal)) return 0;
//            if (TST2(pp[0] * R33 - pp[2] * R13, (A[0] * Q33 + A[2] * Q13 + B[0] * Q22 + B[1] * Q21), R33, 0, -R13, normalC, ref normalR, 12, ref code, ref s, ref invert_normal)) return 0;

//            // separating axis = u3 x (v1,v2,v3)
//            if (TST2(pp[1] * R11 - pp[0] * R21, (A[0] * Q21 + A[1] * Q11 + B[1] * Q33 + B[2] * Q32), -R21, R11, 0, normalC, ref normalR, 13, ref code, ref s, ref invert_normal)) return 0;
//            if (TST2(pp[1] * R12 - pp[0] * R22, (A[0] * Q22 + A[1] * Q12 + B[0] * Q33 + B[2] * Q31), -R22, R12, 0, normalC, ref normalR, 14, ref code, ref s, ref invert_normal)) return 0;
//            if (TST2(pp[1] * R13 - pp[0] * R23, (A[0] * Q23 + A[1] * Q13 + B[0] * Q32 + B[1] * Q31), -R23, R13, 0, normalC, ref normalR, 15, ref code, ref s, ref invert_normal)) return 0;

//            if (code == 0)
//            {
//                return 0;
//            }
//            // if we get to this point, the boxes interpenetrate. compute the normal
//            // in global coordinates.
//            if (normalR != null)
//            {
//                normal[0] = normalR[0 + normalROffset];
//                normal[1] = normalR[4 + normalROffset];
//                normal[2] = normalR[8 + normalROffset];
//            }
//            else
//            {
//                DMULTIPLY0_331(normal, R1, normalC);
//            }
//            if (invert_normal)
//            {
//                normal[0] = -normal[0];
//                normal[1] = -normal[1];
//                normal[2] = -normal[2];
//            }
//            depth = -s;

//            // compute contact point(s)

//            if (code > 6)
//            {
//                // an edge from box 1 touches an edge from box 2.
//                // find a point pa on the intersecting edge of box 1
//                float[] pa1 = new float[3];
//                pa1[0] = p1[0];
//                pa1[1] = p1[1];
//                pa1[2] = p1[2];

//                for (int j = 0; j < 3; j++)
//                {
//                    float sign = (DDOT14(normal, 0, R1, j) > 0) ? 1.0f : -1.0f;
//                    for (int i = 0; i < 3; i++)
//                    {
//                        pa1[i] += sign * A[j] * R1[i * 4 + j];
//                    }
//                }

//                // find a point pb on the intersecting edge of box 2
//                float[] pb1 = new float[3];
//                pb1[0] = p2[0];
//                pb1[1] = p2[1];
//                pb1[2] = p2[2];
//                //for (i = 0; i < 3; i++) pb[i] = p2[i];
//                for (int j = 0; j < 3; j++)
//                {
//                    float sign = (DDOT14(normal, 0, R2, j) > 0) ? -1.0f : 1.0f;
//                    for (int i = 0; i < 3; i++)
//                    {
//                        pb1[i] += sign * B[j] * R2[i * 4 + j];
//                    }
//                }


//                float alpha = 0f, beta = 0f;
//                float[] ua = MathUtil.FloatFromVector3(IndexedVector3.Zero);
//                float[] ub = MathUtil.FloatFromVector3(IndexedVector3.Zero);
//                for (int i = 0; i < 3; i++)
//                {
//                    ua[i] = R1[((code) - 7) / 3 + i * 4];
//                }
//                for (int i = 0; i < 3; i++)
//                {
//                    ub[i] = R2[((code) - 7) % 3 + i * 4];
//                }

//                DLineClosestApproach(pa1, ua, pb1, ub, ref alpha, ref beta);

//                for (int i = 0; i < 3; i++)
//                {
//                    pa1[i] += ua[i] * alpha;
//                }
//                for (int i = 0; i < 3; i++)
//                {
//                    pb1[i] += ub[i] * beta;
//                }

//                {
//                    //contact[0].pos[i] = float(0.5)*(pa[i]+pb[i]);
//                    //contact[0].depth = *depth;
//                    IndexedVector3 pointInWorld = new IndexedVector3();

//#if USE_CENTER_POINT
//            pointInWorld = (pa + pb) * 0.5f;
//            output.addContactPoint(-normal,pointInWorld,-depth);
//#else
//                    IndexedVector3 pbv = IndexedVector3.Zero;
//                    MathUtil.Vector3FromFloat(ref pbv, pb1);
//                    output.AddContactPoint(-new IndexedVector3(normal[0], normal[1], normal[2]), pbv, -depth);
//#endif //
//                    return_code = code;
//                }
//                return 1;
//            }

//            // okay, we have a face-something intersection (because the separating
//            // axis is perpendicular to a face). define face 'a' to be the reference
//            // face (i.e. the normal vector is perpendicular to this) and face 'b' to be
//            // the incident face (the closest face of the other box).


//            float[] Ra;
//            float[] Rb;
//            float[] pa;
//            float[] pb;
//            float[] Sa;
//            float[] Sb;

//            if (code <= 3)
//            {
//                Ra = R1;
//                Rb = R2;
//                pa = p1;
//                pb = p2;
//                Sa = A;
//                Sb = B;
//            }
//            else
//            {
//                Ra = R2;
//                Rb = R1;
//                pa = p2;
//                pb = p1;
//                Sa = B;
//                Sb = A;
//            }

//            // nr = normal vector of reference face dotted with axes of incident box.
//            // anr = absolute values of nr.
//            float[] normal2 = new float[3];
//            float[] nr = new float[3];
//            float[] anr = new float[3];


//            if (code <= 3)
//            {
//                normal2[0] = normal[0];
//                normal2[1] = normal[1];
//                normal2[2] = normal[2];
//            }
//            else
//            {
//                normal2[0] = -normal[0];
//                normal2[1] = -normal[1];
//                normal2[2] = -normal[2];
//            }
//            DMULTIPLY1_331(nr, Rb, normal2);

//            anr[0] = Math.Abs(nr[0]);
//            anr[1] = Math.Abs(nr[1]);
//            anr[2] = Math.Abs(nr[2]);

//            // find the largest compontent of anr: this corresponds to the normal
//            // for the indident face. the other axis numbers of the indicent face
//            // are stored in a1,a2.
//            int lanr, a1, a2;
//            if (anr[1] > anr[0])
//            {
//                if (anr[1] > anr[2])
//                {
//                    a1 = 0;
//                    lanr = 1;
//                    a2 = 2;
//                }
//                else
//                {
//                    a1 = 0;
//                    a2 = 1;
//                    lanr = 2;
//                }
//            }
//            else
//            {
//                if (anr[0] > anr[2])
//                {
//                    lanr = 0;
//                    a1 = 1;
//                    a2 = 2;
//                }
//                else
//                {
//                    a1 = 0;
//                    a2 = 1;
//                    lanr = 2;
//                }
//            }

//            // compute center point of incident face, in reference-face coordinates
//            float[] center = new float[3];
//            if (nr[lanr] < 0)
//            {
//                for (int i = 0; i < 3; i++)
//                {
//                    center[i] = pb[i] - pa[i] + Sb[lanr] * Rb[i * 4 + lanr];
//                }
//            }
//            else
//            {
//                for (int i = 0; i < 3; i++)
//                {
//                    center[i] = pb[i] - pa[i] - Sb[lanr] * Rb[i * 4 + lanr];
//                }
//            }


//            // find the normal and non-normal axis numbers of the reference box
//            int codeN, code1, code2;
//            if (code <= 3)
//            {
//                codeN = code - 1;
//            }
//            else
//            {
//                codeN = code - 4;
//            }
//            if (codeN == 0)
//            {
//                code1 = 1;
//                code2 = 2;
//            }
//            else if (codeN == 1)
//            {
//                code1 = 0;
//                code2 = 2;
//            }
//            else
//            {
//                code1 = 0;
//                code2 = 1;
//            }

//            // find the four corners of the incident face, in reference-face coordinates
//            float[] quad = new float[8];	// 2D coordinate of incident face (x,y pairs)
//            float c1, c2, m11, m12, m21, m22;
//            c1 = DDOT14(center, 0, Ra, code1);
//            c2 = DDOT14(center, 0, Ra, code2);

//            // optimize this? - we have already computed this data above, but it is not
//            // stored in an easy-to-index format. for now it's quicker just to recompute
//            // the four dot products.
//            m11 = DDOT44(Ra, code1, Rb, a1);
//            m12 = DDOT44(Ra, code1, Rb, a2);
//            m21 = DDOT44(Ra, code2, Rb, a1);
//            m22 = DDOT44(Ra, code2, Rb, a2);
//            {
//                float k1 = m11 * Sb[a1];
//                float k2 = m21 * Sb[a1];
//                float k3 = m12 * Sb[a2];
//                float k4 = m22 * Sb[a2];
//                quad[0] = c1 - k1 - k3;
//                quad[1] = c2 - k2 - k4;
//                quad[2] = c1 - k1 + k3;
//                quad[3] = c2 - k2 + k4;
//                quad[4] = c1 + k1 + k3;
//                quad[5] = c2 + k2 + k4;
//                quad[6] = c1 + k1 - k3;
//                quad[7] = c2 + k2 - k4;
//            }

//            // find the size of the reference face
//            float[] rect = new float[2];
//            rect[0] = Sa[code1];
//            rect[1] = Sa[code2];

//            // intersect the incident and reference faces
//            float[] ret = new float[16];
//            int n = IntersectRectQuad2(rect, quad, ret);
//            if (n < 1)
//            {
//                return 0;		// this should never happen
//            }

//            // convert the intersection points into reference-face coordinates,
//            // and compute the contact position and depth for each point. only keep
//            // those points that have a positive (penetrating) depth. delete points in
//            // the 'ret' array as necessary so that 'point' and 'ret' correspond.
//            float[] point = new float[3 * 8];		// penetrating contact points
//            float[] dep = new float[8];			// depths for those points
//            float det1 = 1f / (m11 * m22 - m12 * m21);
//            m11 *= det1;
//            m12 *= det1;
//            m21 *= det1;
//            m22 *= det1;
//            int cnum = 0;			// number of penetrating contact points found
//            for (int j = 0; j < n; j++)
//            {
//                float k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
//                float k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
//                for (int i = 0; i < 3; i++)
//                {
//                    point[cnum * 3 + i] = center[i] + k1 * Rb[i * 4 + a1] + k2 * Rb[i * 4 + a2];
//                }
//                dep[cnum] = Sa[codeN] - DDOT(normal2, 0, point, cnum * 3);
//                if (dep[cnum] >= 0)
//                {
//                    ret[cnum * 2] = ret[j * 2];
//                    ret[cnum * 2 + 1] = ret[j * 2 + 1];
//                    cnum++;
//                }
//            }
//            if (cnum < 1)
//            {
//                return 0;	// this should never happen
//            }

//            // we can't generate more contacts than we actually have
//            if (maxc > cnum)
//            {
//                maxc = cnum;
//            }
//            if (maxc < 1)
//            {
//                maxc = 1;
//            }

//            if (cnum <= maxc)
//            {
//                if (code < 4)
//                {
//                    // we have less contacts than we need, so we use them all
//                    for (int j = 0; j < cnum; j++)
//                    {
//                        float[] pointInWorldFA = new float[3];
//                        for (int i = 0; i < 3; i++)
//                        {
//                            pointInWorldFA[i] = point[j * 3 + i] + pa[i];
//                        }
//                        IndexedVector3 pointInWorld = IndexedVector3.Zero;
//                        MathUtil.Vector3FromFloat(ref pointInWorld, pointInWorldFA);
//                        if (BulletGlobals.g_streamWriter != null && debugBoxBox)
//                        {
//                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "boxbox get closest", pointInWorld);
//                        }

//                        output.AddContactPoint(-new IndexedVector3(normal[0], normal[1], normal[2]), pointInWorld, -dep[j]);
//                    }
//                }
//                else
//                {
//                    // we have less contacts than we need, so we use them all
//                    for (int j = 0; j < cnum; j++)
//                    {
//                        IndexedVector3 pointInWorld = IndexedVector3.Zero;
//                        pointInWorld.X = point[j * 3 + 0] + pa[0] - normal[0] * dep[j];
//                        pointInWorld.Y = point[j * 3 + 1] + pa[1] - normal[1] * dep[j];
//                        pointInWorld.Z = point[j * 3 + 2] + pa[2] - normal[2] * dep[j];

//                        if (BulletGlobals.g_streamWriter != null && debugBoxBox)
//                        {
//                            MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "boxbox get closest", pointInWorld);
//                        }
//                        output.AddContactPoint(-new IndexedVector3(normal[0], normal[1], normal[2]), pointInWorld, -dep[j]);
//                    }
//                }
//            }
//            else
//            {
//                // we have more contacts than are wanted, some of them must be culled.
//                // find the deepest point, it is always the first contact.
//                int i1 = 0;
//                float maxdepth = dep[0];
//                for (int i = 1; i < cnum; i++)
//                {
//                    if (dep[i] > maxdepth)
//                    {
//                        maxdepth = dep[i];
//                        i1 = i;
//                    }
//                }

//                int[] iret = new int[8];
//                CullPoints2(cnum, ret, maxc, i1, iret);

//                for (int j = 0; j < maxc; j++)
//                {
//                    //      dContactGeom *con = CONTACT(contact,skip*j);
//                    //    for (i=0; i<3; i++) con->pos[i] = point[iret[j]*3+i] + pa[i];
//                    //  con->depth = dep[iret[j]];
//                    float[] posInWorldFA = new float[3];
//                    for (int i = 0; i < 3; i++)
//                    {
//                        posInWorldFA[i] = point[iret[j] * 3 + i] + pa[i];
//                    }
//                    IndexedVector3 posInWorld = IndexedVector3.Zero;
//                    MathUtil.Vector3FromFloat(ref posInWorld, posInWorldFA);

//                    if (BulletGlobals.g_streamWriter != null && debugBoxBox)
//                    {
//                        MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "boxbox get closest", posInWorld);
//                    }


//                    output.AddContactPoint(-new IndexedVector3(normal[0], normal[1], normal[2]), posInWorld, -dep[iret[j]]);
//                }
//                cnum = maxc;
//            }
//            return_code = code;
//            return cnum;
//        }

//        private static void DLineClosestApproach(float[] pa, float[] ua,
//                       float[] pb, float[] ub,
//                       ref float alpha, ref float beta)
//        {

//            float[] p = new float[3];
//            p[0] = pb[0] - pa[0];
//            p[1] = pb[1] - pa[1];
//            p[2] = pb[2] - pa[2];

//            float uaub = DDOT(ua, 0, ub, 0);
//            float q1 = DDOT(ua, 0, p, 0);
//            float q2 = -DDOT(ub, 0, p, 0);
//            float d = 1 - uaub * uaub;
//            if (d <= 0.0001f)
//            {
//                // @@@ this needs to be made more robust
//                alpha = 0f;
//                beta = 0f;
//            }
//            else
//            {
//                d = 1f / d;
//                alpha = (q1 + uaub * q2) * d;
//                beta = (uaub * q1 + q2) * d;
//            }
//        }

//        // find all the intersection points between the 2D rectangle with vertices
//        // at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
//        // (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
//        //
//        // the intersection points are returned as x,y pairs in the 'ret' array.
//        // the number of intersection points is returned by the function (this will
//        // be in the range 0 to 8).

//        static int IntersectRectQuad2(float[] h, float[] p, float[] ret)
//        {
//            // q (and r) contain nq (and nr) coordinate points for the current (and
//            // chopped) polygons
//            int nq = 4, nr = 0;
//            float[] buffer = new float[16];
//            float[] q = p;
//            float[] r = ret;

//            for (int dir = 0; dir <= 1; dir++)
//            {
//                // direction notation: xy[0] = x axis, xy[1] = y axis
//                for (int sign = -1; sign <= 1; sign += 2)
//                {
//                    // chop q along the line xy[dir] = sign*h[dir]
//                    float[] pq = q;
//                    float[] pr = r;
//                    int pqIndex = 0;
//                    int prIndex = 0;
//                    nr = 0;
//                    for (int i = nq; i > 0; i--)
//                    {
//                        // go through all points in q and all lines between adjacent points
//                        if (sign * pq[pqIndex + dir] < h[dir])
//                        {
//                            // this point is inside the chopping line
//                            pr[prIndex] = pq[pqIndex];
//                            pr[prIndex + 1] = pq[pqIndex + 1];
//                            prIndex += 2;
//                            nr++;
//                            if ((nr & 8) != 0)
//                            {
//                                q = r;
//                                goto done;
//                            }
//                        }
//                        float nextQDir = 0f;
//                        float nextQ1MinusDir = 0f;
//                        if (i > 1)
//                        {
//                            nextQDir = pq[pqIndex + 2 + dir];
//                            nextQ1MinusDir = pq[pqIndex + 2 + (1 - dir)];
//                        }
//                        else
//                        {
//                            nextQDir = q[dir];
//                            nextQ1MinusDir = q[1 - dir];
//                        }

//                        if ((sign * pq[pqIndex + dir] < h[dir]) ^ (sign * nextQDir < h[dir]))
//                        {
//                            // this line crosses the chopping line
//                            pr[prIndex + (1 - dir)] = pq[pqIndex + (1 - dir)] + (nextQ1MinusDir - pq[pqIndex + (1 - dir)]) /
//                            (nextQDir - pq[pqIndex + dir]) * (sign * h[dir] - pq[pqIndex + dir]);
//                            pr[prIndex + dir] = sign * h[dir];
//                            prIndex += 2;
//                            nr++;
//                            if ((nr & 8) != 0)
//                            {
//                                q = r;
//                                goto done;
//                            }
//                        }
//                        pqIndex += 2;
//                    }
//                    q = r;
//                    r = (q == ret) ? buffer : ret;
//                    nq = nr;
//                }
//            }
//        done:
//            // data already in ret
//            if (q != ret)
//            {
//                for (int i = 0; i < nr * 2; ++i)
//                {
//                    ret[i] = q[i];
//                }
//            }
//            return nr;
//        }

//        // given n points in the plane (array p, of size 2*n), generate m points that
//        // best represent the whole set. the definition of 'best' here is not
//        // predetermined - the idea is to select points that give good box-box
//        // collision detection behavior. the chosen point indexes are returned in the
//        // array iret (of size m). 'i0' is always the first entry in the array.
//        // n must be in the range [1..8]. m must be in the range [1..n]. i0 must be
//        // in the range [0..n-1].

//        private static void CullPoints2(int n, float[] p, int m, int i0, int[] iret)
//        {
//            // compute the centroid of the polygon in cx,cy
//            int iretIndex = 0;
//            int i, j;
//            float a, cx, cy, q;
//            if (n == 1)
//            {
//                cx = p[0];
//                cy = p[1];
//            }
//            else if (n == 2)
//            {
//                cx = 0.5f * (p[0] + p[2]);
//                cy = 0.5f * (p[1] + p[3]);
//            }
//            else
//            {
//                a = 0;
//                cx = 0;
//                cy = 0;
//                for (i = 0; i < (n - 1); i++)
//                {
//                    q = p[i * 2] * p[i * 2 + 3] - p[i * 2 + 2] * p[i * 2 + 1];
//                    a += q;
//                    cx += q * (p[i * 2] + p[i * 2 + 2]);
//                    cy += q * (p[i * 2 + 1] + p[i * 2 + 3]);
//                }
//                q = p[n * 2 - 2] * p[1] - p[0] * p[n * 2 - 1];
//                if (Math.Abs(a + q) > MathUtil.SIMD_EPSILON)
//                {
//                    a = 1f / (3.0f * (a + q));
//                }
//                else
//                {
//                    a = 1e30f;
//                }
//                cx = a * (cx + q * (p[n * 2 - 2] + p[0]));
//                cy = a * (cy + q * (p[n * 2 - 1] + p[1]));
//            }

//            // compute the angle of each point w.r.t. the centroid
//            float[] A = new float[8];
//            for (i = 0; i < n; i++)
//            {
//                A[i] = (float)Math.Atan2(p[i * 2 + 1] - cy, p[i * 2] - cx);
//            }

//            // search for points that have angles closest to A[i0] + i*(2*pi/m).
//            int[] avail = new int[8];
//            for (i = 0; i < n; i++)
//            {
//                avail[i] = 1;
//            }
//            avail[i0] = 0;
//            iret[0] = i0;
//            iretIndex++;
//            for (j = 1; j < m; j++)
//            {
//                a = j * (MathUtil.SIMD_2_PI / m) + A[i0];
//                if (a > MathUtil.SIMD_PI)
//                {
//                    a -= MathUtil.SIMD_2_PI;
//                }
//                float maxdiff = float.MaxValue, diff;

//                iret[iretIndex] = i0;			// iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0

//                for (i = 0; i < n; i++)
//                {
//                    if (avail[i] != 0)
//                    {
//                        diff = Math.Abs(A[i] - a);
//                        if (diff > MathUtil.SIMD_PI)
//                        {
//                            diff = MathUtil.SIMD_2_PI - diff;
//                        }
//                        if (diff < maxdiff)
//                        {
//                            maxdiff = diff;
//                            iret[iretIndex] = i;
//                        }
//                    }
//                }
//                //#if defined(DEBUG) || defined (_DEBUG)
//                //    btAssert (*iret != i0);	// ensure iret got set
//                //#endif
//                avail[iret[iretIndex]] = 0;
//                iretIndex++;
//            }
//        }

//        private static bool TST(float expr1, float expr2, float[] norm, ref float[] normalR, int offset, ref int normalROffset, int cc, ref int code, ref float s, ref bool invert_normal)
//        {
//            float s2 = Math.Abs(expr1) - expr2;
//            if (s2 > 0)
//            {
//                return true;
//            }
//            if (s2 > s)
//            {
//                s = s2;
//                normalR = norm;
//                normalROffset = offset;
//                invert_normal = expr1 < 0;
//                code = cc;
//            }
//            return false;
//        }

//        // note: cross product axes need to be scaled when s is computed.
//        // normal (n1,n2,n3) is relative to box 1.
//        private static bool TST2(float expr1, float expr2, float n1, float n2, float n3, float[] normalC, ref float[] normalR, int cc, ref int code, ref float s, ref bool invert_normal)
//        {
//            float s2 = Math.Abs(expr1) - (expr2);
//            if (s2 > MathUtil.SIMD_EPSILON)
//            {
//                return true;
//            }
//            float l = (float)Math.Sqrt((n1 * n1) + (n2 * n2) + (n3 * n3));
//            if (l > MathUtil.SIMD_EPSILON)
//            {
//                s2 /= l;
//                if (s2 * fudge_factor > s)
//                {
//                    s = s2;
//                    normalR = null;
//                    normalC[0] = (n1) / l; normalC[1] = (n2) / l; normalC[2] = (n3) / l;
//                    invert_normal = ((expr1) < 0);
//                    invert_normal = ((expr1) < 0);
//                    code = (cc);
//                }
//            }
//            return false;
//        }

//        private static float DDOT(float[] a, int aOffset, float[] b, int bOffset) { return DDOTpq(a, b, aOffset, bOffset, 1, 1); }
//        private static float DDOT44(float[] a, int aOffset, float[] b, int bOffset) { return DDOTpq(a, b, aOffset, bOffset, 4, 4); }
//        private static float DDOT41(float[] a, int aOffset, float[] b, int bOffset) { return DDOTpq(a, b, aOffset, bOffset, 4, 1); }
//        private static float DDOT14(float[] a, int aOffset, float[] b, int bOffset) { return DDOTpq(a, b, aOffset, bOffset, 1, 4); }

//        private static float DDOTpq(float[] a, float[] b, int aOffset, int bOffset, int p, int q)
//        {
//            //return (a[0] * b[0] + (a)[p] * (b)[q] + (a)[2 * (p)] * (b)[2 * (q)]); 
//            return (a[aOffset] * b[bOffset] + a[p + aOffset] * b[q + bOffset] + a[(2 * p) + aOffset] * b[(2 * q) + bOffset]);
//        }

//        public static void DMULTIPLY1_331(float[] A, float[] B, float[] C)
//        {
//            A[0] = DDOT41(B, 0, C, 0);
//            A[1] = DDOT41(B, 1, C, 0);
//            A[2] = DDOT41(B, 2, C, 0);
//        }

//        public static void DMULTIPLY0_331(float[] A, float[] B, float[] C)
//        {
//            A[0] = DDOT(B, 0, C, 0);
//            A[1] = DDOT(B, 4, C, 0);
//            A[2] = DDOT(B, 8, C, 0);
//        }

//        private BoxShape m_box1;
//        private BoxShape m_box2;
//        private static float fudge_factor = 1.05f;
//        private static bool debugBoxBox = false;

//    }
//}
