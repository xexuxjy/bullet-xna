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

using System.Collections.Generic;

namespace BulletXNA.BulletCollision
{

    ///The btTriangleInfo structure stores information to adjust collision normals to avoid collisions against internal edges
    ///it can be generated using 
    public class TriangleInfo
    {
        public TriangleInfo()
        {
            m_edgeV0V1Angle = MathUtil.SIMD_2_PI;
            m_edgeV1V2Angle = MathUtil.SIMD_2_PI;
            m_edgeV2V0Angle = MathUtil.SIMD_2_PI;
            m_flags = 0;
        }

        public int m_flags;

        public float m_edgeV0V1Angle;
        public float m_edgeV1V2Angle;
        public float m_edgeV2V0Angle;

    }


    public class TriangleInfoMap : Dictionary<int, TriangleInfo>
    {

        ///for btTriangleInfo m_flags
        public const int TRI_INFO_V0V1_CONVEX = 1;
        public const int TRI_INFO_V1V2_CONVEX = 2;
        public const int TRI_INFO_V2V0_CONVEX = 4;

        public const int TRI_INFO_V0V1_SWAP_NORMALB = 8;
        public const int TRI_INFO_V1V2_SWAP_NORMALB = 16;
        public const int TRI_INFO_V2V0_SWAP_NORMALB = 32;


        ///The btTriangleInfoMap stores edge angle information for some triangles. You can compute this information yourself or using btGenerateInternalEdgeInfo.
        public float m_convexEpsilon;///used to determine if an edge or contact normal is convex, using the dot product
        public float m_planarEpsilon; ///used to determine if a triangle edge is planar with zero angle
        public float m_equalVertexThreshold; ///used to compute connectivity: if the distance between two vertices is smaller than m_equalVertexThreshold, they are considered to be 'shared'
        public float m_edgeDistanceThreshold; ///used to determine edge contacts: if the closest distance between a contact point and an edge is smaller than this distance threshold it is considered to "hit the edge"
        public float m_maxEdgeAngleThreshold; //ignore edges that connect triangles at an angle larger than this m_maxEdgeAngleThreshold
        public float m_zeroAreaThreshold; ///used to determine if a triangle is degenerate (length squared of cross product of 2 triangle edges < threshold)

        public TriangleInfoMap()
        {
            m_convexEpsilon = 0.00f;
            m_planarEpsilon = 0.0001f;
            m_equalVertexThreshold = 0.0001f * 0.0001f;
            m_edgeDistanceThreshold = 0.1f;
            m_zeroAreaThreshold = 0.0001f * 0.0001f;
            m_maxEdgeAngleThreshold = MathUtil.SIMD_2_PI;

        }
    }

}

