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
    public interface IDebugDraw
    {
        void DrawLine(IndexedVector3 from, IndexedVector3 to, IndexedVector3 color);
        void DrawLine(ref IndexedVector3 from, ref IndexedVector3 to, ref IndexedVector3 fromColor);
        void DrawLine(ref IndexedVector3 from, ref IndexedVector3 to, ref IndexedVector3 fromColor, ref IndexedVector3 toColor);
        void DrawBox(ref IndexedVector3 bbMin, ref IndexedVector3 bbMax, ref IndexedVector3 color);
        void DrawBox(ref IndexedVector3 bbMin, ref IndexedVector3 bbMax, ref IndexedMatrix trans, ref IndexedVector3 color);
        void DrawSphere(IndexedVector3 p, float radius, IndexedVector3 color);
        void DrawSphere(ref IndexedVector3 p, float radius, ref IndexedVector3 color);
        void DrawSphere(float radius, ref IndexedMatrix transform, ref IndexedVector3 color);
        void DrawTriangle(ref IndexedVector3 v0, ref IndexedVector3 v1, ref IndexedVector3 v2, ref IndexedVector3 n0, ref IndexedVector3 n1, ref IndexedVector3 n2, ref IndexedVector3 color, float alpha);
        void DrawTriangle(ref IndexedVector3 v0, ref IndexedVector3 v1, ref IndexedVector3 v2, ref IndexedVector3 color, float alpha);
        void DrawContactPoint(IndexedVector3 pointOnB, IndexedVector3 normalOnB, float distance, int lifeTime, IndexedVector3 color);
        void DrawContactPoint(ref IndexedVector3 pointOnB, ref IndexedVector3 normalOnB, float distance, int lifeTime, ref IndexedVector3 color);
        void ReportErrorWarning(String warningString);
        void Draw3dText(ref IndexedVector3 location, String textString);

        void SetDebugMode(DebugDrawModes debugMode);
        DebugDrawModes GetDebugMode();

        void DrawAabb(IndexedVector3 from, IndexedVector3 to, IndexedVector3 color);
        void DrawAabb(ref IndexedVector3 from, ref IndexedVector3 to, ref IndexedVector3 color);
        void DrawTransform(ref IndexedMatrix transform, float orthoLen);
        void DrawArc(ref IndexedVector3 center, ref IndexedVector3 normal, ref IndexedVector3 axis, float radiusA, float radiusB, float minAngle, float maxAngle,
            ref IndexedVector3 color, bool drawSect);
        void DrawArc(ref IndexedVector3 center, ref IndexedVector3 normal, ref IndexedVector3 axis, float radiusA, float radiusB, float minAngle, float maxAngle,
            ref IndexedVector3 color, bool drawSect, float stepDegrees);
        void DrawSpherePatch(ref IndexedVector3 center, ref IndexedVector3 up, ref IndexedVector3 axis, float radius,
            float minTh, float maxTh, float minPs, float maxPs, ref IndexedVector3 color);
        void DrawSpherePatch(ref IndexedVector3 center, ref IndexedVector3 up, ref IndexedVector3 axis, float radius,
            float minTh, float maxTh, float minPs, float maxPs, ref IndexedVector3 color, float stepDegrees);
        void DrawCapsule(float radius, float halfHeight, int upAxis, ref IndexedMatrix transform, ref IndexedVector3 color);
        void DrawCylinder(float radius, float halfHeight, int upAxis, ref IndexedMatrix transform, ref IndexedVector3 color);
        void DrawCone(float radius, float height, int upAxis, ref IndexedMatrix transform, ref IndexedVector3 color);
        void DrawPlane(ref IndexedVector3 planeNormal, float planeConst, ref IndexedMatrix transform, ref IndexedVector3 color);
    }



    /*-------------------------------------------------------------------------------------*/

    [Flags]
    public enum DebugDrawModes
    {
        DBG_NoDebug = 0,
        DBG_DrawWireframe = 1,
        DBG_DrawAabb = 2,
        DBG_DrawFeaturesText = 4,
        DBG_DrawContactPoints = 8,
        DBG_NoDeactivation = 16,
        DBG_NoHelpText = 32,
        DBG_DrawText = 64,
        DBG_ProfileTimings = 128,
        DBG_EnableSatComparison = 256,
        DBG_DisableBulletLCP = 512,
        DBG_EnableCCD = 1024,
        DBG_DrawConstraints = (1 << 11),
        DBG_DrawConstraintLimits = (1 << 12),
        DBG_DrawFastWireframe = (1 << 13),
        DBG_DrawNormals = (1 << 14),
        ALL = DBG_DrawWireframe|DBG_DrawAabb|DBG_DrawFeaturesText|DBG_DrawContactPoints|DBG_DrawText|DBG_DrawConstraints|DBG_DrawConstraintLimits|DBG_DrawNormals,
        DBG_MAX_DEBUG_DRAW_MODE
    }

    /*-------------------------------------------------------------------------------------*/

}
