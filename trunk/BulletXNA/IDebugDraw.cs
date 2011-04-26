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
using Microsoft.Xna.Framework;

namespace BulletXNA.LinearMath
{
    public interface IDebugDraw
    {
        void DrawLine(Vector3 from, Vector3 to, Vector3 fromColor);
        void DrawLine(ref Vector3 from,ref Vector3 to, ref Vector3 fromColor);
	    void DrawLine(ref Vector3 from,ref Vector3 to, ref Vector3 fromColor, ref Vector3 toColor);
        //void drawBox(ref Vector3 bbMin, ref Vector3 bbMax, ref Vector3 color);
        void DrawBox(ref Vector3 bbMin, ref Vector3 bbMax, ref Matrix trans, ref Vector3 color);
        void DrawBox(ref Vector3 bbMin, ref Vector3 bbMax, ref Matrix transform,ref Vector3 color, float alpha);
        void DrawSphere(Vector3 p, float radius, Vector3 color);
        void DrawSphere(ref Vector3 p, float radius, ref Vector3 color);
	    void DrawTriangle(ref Vector3 v0,ref Vector3 v1,ref Vector3 v2,ref Vector3 n0,ref Vector3 n1,ref Vector3 n2,ref Vector3 color, float alpha);
	    void DrawTriangle(ref Vector3 v0,ref Vector3 v1,ref Vector3 v2,ref Vector3 color, float alpha);
        void DrawContactPoint(Vector3 pointOnB, Vector3 normalOnB, float distance, int lifeTime, Vector3 color);
	    void DrawContactPoint(ref Vector3 pointOnB,ref Vector3 normalOnB,float distance,int lifeTime,ref Vector3 color);
	    void ReportErrorWarning(String warningString);
	    void Draw3dText(ref Vector3 location,String textString);
        void SetDebugMode(DebugDrawModes debugMode);

        DebugDrawModes GetDebugMode();

        void DrawAabb(Vector3 from, Vector3 to, Vector3 color);

        void DrawAabb(ref Vector3 from, ref Vector3 to, ref Vector3 color);

        void DrawTransform(ref Matrix transform, float orthoLen);

        void DrawArc(ref Vector3 center, ref Vector3 normal, ref Vector3 axis, float radiusA, float radiusB, float minAngle, float maxAngle,
            ref Vector3 color, bool drawSect);
        //{
        //    drawArc(ref center,ref normal,ref axis,radiusA,radiusB,minAngle,maxAngle,ref color,drawSect,10f);
        //}

        void DrawArc(ref Vector3 center, ref Vector3 normal, ref Vector3 axis, float radiusA, float radiusB, float minAngle, float maxAngle, 
		    ref Vector3 color, bool drawSect, float stepDegrees);

        void DrawSpherePatch(ref Vector3 center, ref Vector3 up, ref Vector3 axis, float radius,
            float minTh, float maxTh, float minPs, float maxPs, ref Vector3 color);
        //{
        //    drawSpherePatch(ref center,ref up,ref axis,radius,minTh,maxTh,minPs,maxPs,ref color,10.0f);
        //}


        void DrawSpherePatch(ref Vector3 center, ref Vector3 up, ref Vector3 axis, float radius, 
		    float minTh, float maxTh, float minPs, float maxPs, ref Vector3 color, float stepDegrees);
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
        DBG_MAX_DEBUG_DRAW_MODE
    }

    /*-------------------------------------------------------------------------------------*/

}
