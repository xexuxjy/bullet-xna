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

namespace BulletXNA.LinearMath
{
    public abstract class DebugDraw : IDebugDraw
    {
        public abstract void DrawLine(Vector3 from, Vector3 to, Vector3 color);
        public abstract void DrawLine(ref Vector3 from, ref Vector3 to, ref Vector3 color);
        public abstract void Draw3dText(ref Vector3 location, String textString);
        public abstract void DrawContactPoint(Vector3 pointOnB, Vector3 normalOnB, float distance, int lifeTime, Vector3 color);
        public abstract void DrawContactPoint(ref Vector3 pointOnB, ref Vector3 normalOnB, float distance, int lifeTime, ref Vector3 color);
        public abstract void ReportErrorWarning(String warningString);
        public abstract DebugDrawModes DebugMode { get; set; }

        public virtual void DrawLine(ref Vector3 from, ref Vector3 to, ref Vector3 fromColor, ref Vector3 toColor)
        {
            DrawLine(ref from, ref to, ref fromColor);
        }

        public virtual void DrawBox(ref Vector3 bbMin, ref Vector3 bbMax, ref Vector3 color)
        {
            DrawLine(bbMin, new Vector3(bbMax.X, bbMin.Y, bbMin.Z), color);
            DrawLine(new Vector3(bbMax.X, bbMin.Y, bbMin.Z), new Vector3(bbMax.X, bbMax.Y, bbMin.Z), color);
            DrawLine(new Vector3(bbMax.X, bbMax.Y, bbMin.Z), new Vector3(bbMin.X, bbMax.Y, bbMin.Z), color);
            DrawLine(new Vector3(bbMin.X, bbMax.Y, bbMin.Z), bbMin, color);
            DrawLine(bbMin, new Vector3(bbMin.X, bbMin.Y, bbMax.Z), color);
            DrawLine(new Vector3(bbMax.X, bbMin.Y, bbMin.Z), new Vector3(bbMax.X, bbMin.Y, bbMax.Z), color);
            DrawLine(new Vector3(bbMax.X, bbMax.Y, bbMin.Z), bbMax, color);
            DrawLine(new Vector3(bbMin.X, bbMax.Y, bbMin.Z), new Vector3(bbMin.X, bbMax.Y, bbMax.Z), color);
            DrawLine(new Vector3(bbMin.X, bbMin.Y, bbMax.Z), new Vector3(bbMax.X, bbMin.Y, bbMax.Z), color);
            DrawLine(new Vector3(bbMax.X, bbMin.Y, bbMax.Z), bbMax, color);
            DrawLine(bbMax, new Vector3(bbMin.X, bbMax.Y, bbMax.Z), color);
            DrawLine(new Vector3(bbMin.X, bbMax.Y, bbMax.Z), new Vector3(bbMin.X, bbMin.Y, bbMax.Z), color);
        }

        public virtual void DrawBox(ref Vector3 bbMin, ref Vector3 bbMax, ref Matrix trans, ref Vector3 color)
        {
            DrawLine(trans * bbMin, trans * new Vector3(bbMax.X, bbMin.Y, bbMin.Z), color);
            DrawLine(trans * new Vector3(bbMax.X, bbMin.Y, bbMin.Z), trans * new Vector3(bbMax.X, bbMax.Y, bbMin.Z), color);
            DrawLine(trans * new Vector3(bbMax.X, bbMax.Y, bbMin.Z), trans * new Vector3(bbMin.X, bbMax.Y, bbMin.Z), color);
            DrawLine(trans * new Vector3(bbMin.X, bbMax.Y, bbMin.Z), trans * bbMin, color);
            DrawLine(trans * bbMin, trans * new Vector3(bbMin.X, bbMin.Y, bbMax.Z), color);
            DrawLine(trans * new Vector3(bbMax.X, bbMin.Y, bbMin.Z), trans * new Vector3(bbMax.X, bbMin.Y, bbMax.Z), color);
            DrawLine(trans * new Vector3(bbMax.X, bbMax.Y, bbMin.Z), trans * bbMax, color);
            DrawLine(trans * new Vector3(bbMin.X, bbMax.Y, bbMin.Z), trans * new Vector3(bbMin.X, bbMax.Y, bbMax.Z), color);
            DrawLine(trans * new Vector3(bbMin.X, bbMin.Y, bbMax.Z), trans * new Vector3(bbMax.X, bbMin.Y, bbMax.Z), color);
            DrawLine(trans * new Vector3(bbMax.X, bbMin.Y, bbMax.Z), trans * bbMax, color);
            DrawLine(trans * bbMax, trans * new Vector3(bbMin.X, bbMax.Y, bbMax.Z), color);
            DrawLine(trans * new Vector3(bbMin.X, bbMax.Y, bbMax.Z), trans * new Vector3(bbMin.X, bbMin.Y, bbMax.Z), color);
        }

        public virtual void DrawSphere(float radius, ref Matrix transform, ref Vector3 color)
        {
            Vector3 start = transform.Translation;

            Vector3 xoffs = transform._basis * new Vector3(radius, 0, 0);
            Vector3 yoffs = transform._basis * new Vector3(0, radius, 0);
            Vector3 zoffs = transform._basis * new Vector3(0, 0, radius);

            // XY 
            DrawLine(start - xoffs, start + yoffs, color);
            DrawLine(start + yoffs, start + xoffs, color);
            DrawLine(start + xoffs, start - yoffs, color);
            DrawLine(start - yoffs, start - xoffs, color);

            // XZ
            DrawLine(start - xoffs, start + zoffs, color);
            DrawLine(start + zoffs, start + xoffs, color);
            DrawLine(start + xoffs, start - zoffs, color);
            DrawLine(start - zoffs, start - xoffs, color);

            // YZ
            DrawLine(start - yoffs, start + zoffs, color);
            DrawLine(start + zoffs, start + yoffs, color);
            DrawLine(start + yoffs, start - zoffs, color);
            DrawLine(start - zoffs, start - yoffs, color);
        }

        public virtual void DrawSphere(Vector3 p, float radius, Vector3 color)
        {
            Matrix tr = Matrix.CreateTranslation(p);
            DrawSphere(radius, ref tr, ref color);
        }

        public virtual void DrawSphere(ref Vector3 p, float radius, ref Vector3 color)
        {
            Matrix tr = Matrix.CreateTranslation(p);
            DrawSphere(radius, ref tr, ref color);
        }

        public virtual void DrawTriangle(ref Vector3 v0, ref Vector3 v1, ref Vector3 v2, ref Vector3 n0, ref Vector3 n1, ref Vector3 n2, ref Vector3 color, float alpha)
        {
            DrawTriangle(ref v0, ref v1, ref v2, ref color, alpha);
        }

        public virtual void DrawTriangle(ref Vector3 v0, ref Vector3 v1, ref Vector3 v2, ref Vector3 color, float alpha)
        {
            DrawLine(ref v0, ref v1, ref color);
            DrawLine(ref v1, ref v2, ref color);
            DrawLine(ref v2, ref v0, ref color);
        }

        public virtual void DrawAabb(Vector3 from, Vector3 to, Vector3 color)
        {
            DrawAabb(ref from, ref to, ref color);
        }

        public virtual void DrawAabb(ref Vector3 from, ref Vector3 to, ref Vector3 color)
        {
            Vector3 halfExtents = (to - from) * 0.5f;
            Vector3 center = (to + from) * 0.5f;
            int i, j;

            Vector3 edgecoord = new Vector3(1.0f, 1.0f, 1.0f), pa, pb;
            for (i = 0; i < 4; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    pa = new Vector3(edgecoord[0] * halfExtents[0], edgecoord[1] * halfExtents[1],
                           edgecoord[2] * halfExtents[2]);
                    pa += center;

                    int othercoord = j % 3;
                    edgecoord[othercoord] *= -1.0f;
                    pb = new Vector3(edgecoord[0] * halfExtents[0], edgecoord[1] * halfExtents[1],
                            edgecoord[2] * halfExtents[2]);
                    pb += center;

                    DrawLine(pa, pb, color);
                }
                edgecoord = new Vector3(-1.0f, -1.0f, -1.0f);
                if (i < 3)
                {
                    edgecoord[i] *= -1.0f;
                }
            }
        }

        public virtual void DrawTransform(ref Matrix transform, float orthoLen)
        {
            Vector3 start = transform.Translation;
            Vector3 temp = start + transform._basis * new Vector3(orthoLen, 0, 0);
            Vector3 colour = new Vector3(0.7f, 0, 0);
            DrawLine(ref start, ref temp, ref colour);
            temp = start + transform._basis * new Vector3(0, orthoLen, 0);
            colour = new Vector3(0, 0.7f, 0);
            DrawLine(ref start, ref temp, ref colour);
            temp = start + transform._basis * new Vector3(0, 0, orthoLen);
            colour = new Vector3(0, 0, 0.7f);
            DrawLine(ref start, ref temp, ref colour);
        }

        public virtual void DrawArc(ref Vector3 center, ref Vector3 normal, ref Vector3 axis, float radiusA, float radiusB, float minAngle, float maxAngle,
            ref Vector3 color, bool drawSect)
        {
            DrawArc(ref center, ref normal, ref axis, radiusA, radiusB, minAngle, maxAngle, ref color, drawSect, 10f);
        }

        public virtual void DrawArc(ref Vector3 center, ref Vector3 normal, ref Vector3 axis, float radiusA, float radiusB, float minAngle, float maxAngle,
            ref Vector3 color, bool drawSect, float stepDegrees)
        {
            Vector3 vx = axis;
            Vector3 vy = Vector3.Cross(normal, axis);
            float step = stepDegrees * MathUtil.SIMD_RADS_PER_DEG;
            int nSteps = (int)((maxAngle - minAngle) / step);
            if (nSteps == 0)
            {
                nSteps = 1;
            }
            Vector3 prev = center + radiusA * vx * (float)Math.Cos(minAngle) + radiusB * vy * (float)Math.Sin(minAngle);
            if (drawSect)
            {
                DrawLine(ref center, ref prev, ref color);
            }
            for (int i = 1; i <= nSteps; i++)
            {
                float angle = minAngle + (maxAngle - minAngle) * i / nSteps;
                Vector3 next = center + radiusA * vx * (float)Math.Cos(angle) + radiusB * vy * (float)Math.Sin(angle);
                DrawLine(ref prev, ref next, ref color);
                prev = next;
            }
            if (drawSect)
            {
                DrawLine(ref center, ref prev, ref color);
            }
        }

        public virtual void DrawSpherePatch(ref Vector3 center, ref Vector3 up, ref Vector3 axis, float radius,
            float minTh, float maxTh, float minPs, float maxPs, ref Vector3 color)
        {
            DrawSpherePatch(ref center, ref up, ref axis, radius, minTh, maxTh, minPs, maxPs, ref color, 10.0f);
        }

        public virtual void DrawSpherePatch(ref Vector3 center, ref Vector3 up, ref Vector3 axis, float radius,
            float minTh, float maxTh, float minPs, float maxPs, ref Vector3 color, float stepDegrees)
        {
            Vector3[] vA;
            Vector3[] vB;
            Vector3[] pvA, pvB, pT;
            Vector3 npole = center + up * radius;
            Vector3 spole = center - up * radius;
            Vector3 arcStart = Vector3.Zero;
            float step = stepDegrees * MathUtil.SIMD_RADS_PER_DEG;
            Vector3 kv = up;
            Vector3 iv = axis;

            Vector3 jv = Vector3.Cross(kv, iv);
            bool drawN = false;
            bool drawS = false;
            if (minTh <= -MathUtil.SIMD_HALF_PI)
            {
                minTh = -MathUtil.SIMD_HALF_PI + step;
                drawN = true;
            }
            if (maxTh >= MathUtil.SIMD_HALF_PI)
            {
                maxTh = MathUtil.SIMD_HALF_PI - step;
                drawS = true;
            }
            if (minTh > maxTh)
            {
                minTh = -MathUtil.SIMD_HALF_PI + step;
                maxTh = MathUtil.SIMD_HALF_PI - step;
                drawN = drawS = true;
            }
            int n_hor = (int)((maxTh - minTh) / step) + 1;
            if (n_hor < 2) n_hor = 2;
            float step_h = (maxTh - minTh) / (n_hor - 1);
            bool isClosed = false;
            if (minPs > maxPs)
            {
                minPs = -MathUtil.SIMD_PI + step;
                maxPs = MathUtil.SIMD_PI;
                isClosed = true;
            }
            else if ((maxPs - minPs) >= MathUtil.SIMD_PI * 2f)
            {
                isClosed = true;
            }
            else
            {
                isClosed = false;
            }
            int n_vert = (int)((maxPs - minPs) / step) + 1;
            if (n_vert < 2) n_vert = 2;

            vA = new Vector3[n_vert];
            vB = new Vector3[n_vert];
            pvA = vA; pvB = vB;

            float step_v = (maxPs - minPs) / (float)(n_vert - 1);
            for (int i = 0; i < n_hor; i++)
            {
                float th = minTh + i * step_h;
                float sth = radius * (float)Math.Sin(th);
                float cth = radius * (float)Math.Cos(th);
                for (int j = 0; j < n_vert; j++)
                {
                    float psi = minPs + (float)j * step_v;
                    float sps = (float)Math.Sin(psi);
                    float cps = (float)Math.Cos(psi);
                    pvB[j] = center + cth * cps * iv + cth * sps * jv + sth * kv;
                    if (i != 0)
                    {
                        DrawLine(pvA[j], pvB[j], color);
                    }
                    else if (drawS)
                    {
                        DrawLine(spole, pvB[j], color);
                    }
                    if (j != 0)
                    {
                        DrawLine(pvB[j - 1], pvB[j], color);
                    }
                    else
                    {
                        arcStart = pvB[j];
                    }
                    if ((i == (n_hor - 1)) && drawN)
                    {
                        DrawLine(npole, pvB[j], color);
                    }
                    if (isClosed)
                    {
                        if (j == (n_vert - 1))
                        {
                            DrawLine(arcStart, pvB[j], color);
                        }
                    }
                    else
                    {
                        if (((i == 0) || (i == (n_hor - 1))) && ((j == 0) || (j == (n_vert - 1))))
                        {
                            DrawLine(center, pvB[j], color);
                        }
                    }
                }
                pT = pvA; pvA = pvB; pvB = pT;
            }
        }

        public virtual void DrawCapsule(float radius, float halfHeight, int upAxis, ref Matrix transform, ref Vector3 color)
        {
            Vector3 capStart = Vector3.Zero; ;
            capStart[upAxis] = -halfHeight;

            Vector3 capEnd = Vector3.Zero;
            capEnd[upAxis] = halfHeight;

            // Draw the ends
            {
                Matrix childTransform = transform;
                childTransform.Translation = transform * capStart;
                DrawSphere(radius, ref childTransform, ref color);
            }

            {
                Matrix childTransform = transform;
                childTransform.Translation = transform * capEnd;
                DrawSphere(radius, ref childTransform, ref color);
            }

            // Draw some additional lines
            Vector3 start = transform.Translation;

            capStart[(upAxis + 1) % 3] = radius;
            capEnd[(upAxis + 1) % 3] = radius;

            DrawLine(start + transform._basis * capStart, start + transform._basis * capEnd, color);

            capStart[(upAxis + 1) % 3] = -radius;
            capEnd[(upAxis + 1) % 3] = -radius;
            DrawLine(start + transform._basis * capStart, start + transform._basis * capEnd, color);


            capStart[(upAxis + 2) % 3] = radius;
            capEnd[(upAxis + 2) % 3] = radius;
            DrawLine(start + transform._basis * capStart, start + transform._basis * capEnd, color);


            capStart[(upAxis + 2) % 3] = -radius;
            capEnd[(upAxis + 2) % 3] = -radius;
            DrawLine(start + transform._basis * capStart, start + transform._basis * capEnd, color);
        }

        public virtual void DrawCylinder(float radius, float halfHeight, int upAxis, ref Matrix transform, ref Vector3 color)
        {
            Vector3 start = transform.Translation;
            Vector3 offsetHeight = Vector3.Zero;
            offsetHeight[upAxis] = halfHeight;
            Vector3 offsetRadius = Vector3.Zero;
            offsetRadius[(upAxis + 1) % 3] = radius;
            DrawLine(start + transform._basis * offsetHeight + offsetRadius, start + transform._basis * -offsetHeight + offsetRadius, color);
            DrawLine(start + transform._basis * offsetHeight - offsetRadius, start + transform._basis * -offsetHeight - offsetRadius, color);
        }

        public virtual void DrawCone(float radius, float height, int upAxis, ref Matrix transform, ref Vector3 color)
        {
            Vector3 start = transform.Translation;

            Vector3 offsetHeight = Vector3.Zero;
            offsetHeight[upAxis] = height * 0.5f;
            Vector3 offsetRadius = Vector3.Zero;
            offsetRadius[(upAxis + 1) % 3] = radius;

            Vector3 offset2Radius = Vector3.Zero;
            offsetRadius[(upAxis + 2) % 3] = radius;

            DrawLine(start + transform._basis * offsetHeight, start + transform._basis * -offsetHeight + offsetRadius, color);
            DrawLine(start + transform._basis * offsetHeight, start + transform._basis * -offsetHeight - offsetRadius, color);
            DrawLine(start + transform._basis * offsetHeight, start + transform._basis * -offsetHeight + offset2Radius, color);
            DrawLine(start + transform._basis * offsetHeight, start + transform._basis * -offsetHeight - offset2Radius, color);
        }

        public virtual void DrawPlane(ref Vector3 planeNormal, float planeConst, ref Matrix transform, ref Vector3 color)
        {
            Vector3 planeOrigin = planeNormal * planeConst;
            Vector3 vec0, vec1;
            TransformUtil.PlaneSpace1(ref planeNormal, out vec0, out vec1);
            float vecLen = 100f;
            Vector3 pt0 = planeOrigin + vec0 * vecLen;
            Vector3 pt1 = planeOrigin - vec0 * vecLen;
            Vector3 pt2 = planeOrigin + vec1 * vecLen;
            Vector3 pt3 = planeOrigin - vec1 * vecLen;
            DrawLine(transform * pt0, transform * pt1, color);
            DrawLine(transform * pt2, transform * pt3, color);
        }
    }
}
