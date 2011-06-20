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
using System.Diagnostics;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision
{

    ///btHeightfieldTerrainShape simulates a 2D heightfield terrain
    /**
      The caller is responsible for maintaining the heightfield array; this
      class does not make a copy.

      The heightfield can be dynamic so long as the min/max height values
      capture the extremes (heights must always be in that range).

      The local origin of the heightfield is assumed to be the exact
      center (as determined by width and length and height, with each
      axis multiplied by the localScaling).

      \b NOTE: be careful with coordinates.  If you have a heightfield with a local
      min height of -100m, and a max height of +500m, you may be tempted to place it
      at the origin (0,0) and expect the heights in world coordinates to be
      -100 to +500 meters.
      Actually, the heights will be -300 to +300m, because bullet will re-center
      the heightfield based on its AABB (which is determined by the min/max
      heights).  So keep in mind that once you create a btHeightfieldTerrainShape
      object, the heights will be adjusted relative to the center of the AABB.  This
      is different to the behavior of many rendering engines, but is useful for
      physics engines.

      Most (but not all) rendering and heightfield libraries assume upAxis = 1
      (that is, the y-axis is "up").  This class allows any of the 3 coordinates
      to be "up".  Make sure your choice of axis is consistent with your rendering
      system.

      The heightfield heights are determined from the data type used for the
      heightfieldData array.  

       - PHY_UCHAR: height at a point is the uchar value at the
           grid point, multipled by heightScale.  uchar isn't recommended
           because of its inability to deal with negative values, and
           low resolution (8-bit).

       - PHY_SHORT: height at a point is the short int value at that grid
           point, multipled by heightScale.

       - PHY_FLOAT: height at a point is the float value at that grid
           point.  heightScale is ignored when using the float heightfield
           data type.

      Whatever the caller specifies as minHeight and maxHeight will be honored.
      The class will not inspect the heightfield to discover the actual minimum
      or maximum heights.  These values are used to determine the heightfield's
      axis-aligned bounding box, multiplied by localScaling.

      For usage and testing see the TerrainDemo.
     */
    public class HeightfieldTerrainShape : ConcaveShape
    {
        protected Vector3 m_localAabbMin;
        protected Vector3 m_localAabbMax;
        protected Vector3 m_localOrigin;

        ///terrain data
        protected int m_heightStickWidth;
        protected int m_heightStickLength;
        protected float m_minHeight;
        protected float m_maxHeight;
        protected float m_width;
        protected float m_length;
        protected float m_heightScale;

        protected byte[] m_heightFieldData;
        //union
        //{
        //    unsigned char*	m_heightfieldDataUnsignedChar;
        //    short*		m_heightfieldDataShort;
        //    btScalar*			m_heightfieldDataFloat;
        //    void*			m_heightfieldDataUnknown;
        //};

        protected PHY_ScalarType m_heightDataType;
        protected bool m_flipQuadEdges;
        protected bool m_useDiamondSubdivision;

        protected int m_upAxis;

        protected Vector3 m_localScaling;

        protected virtual float GetRawHeightFieldValue(int x, int y)
        {
            float val = 0f;
            switch (m_heightDataType)
            {
                case PHY_ScalarType.PHY_FLOAT:
                    {
                        // float offset (4 for sizeof)
                        int index = ((y * m_heightStickWidth) + x) * 4;
                        val = BitConverter.ToSingle(m_heightFieldData, index);
                        break;
                    }

                case PHY_ScalarType.PHY_UCHAR:
                    {
                        byte heightFieldValue = m_heightFieldData[(y * m_heightStickWidth) + x];
                        val = heightFieldValue * m_heightScale;
                        break;
                    }

                case PHY_ScalarType.PHY_SHORT:
                    {
                        int index = ((y * m_heightStickWidth) + x)*2;
                        short hfValue = BitConverter.ToInt16(m_heightFieldData,index);
                        val = hfValue * m_heightScale;
                        break;
                    }

                default:
                    {
                        Debug.Assert(false, "Bad m_heightDataType");
                        break;
                    }
            }

            return val;


        }
        protected void QuantizeWithClamp(int[] output, ref Vector3 point, int isMax)
        {
            /// given input vector, return quantized version
            /**
              This routine is basically determining the gridpoint indices for a given
              input vector, answering the question: "which gridpoint is closest to the
              provided point?".

              "with clamp" means that we restrict the point to be in the heightfield's
              axis-aligned bounding box.
             */
            Vector3 clampedPoint = point;
            MathUtil.VectorClampMax(ref clampedPoint, ref m_localAabbMax);
            MathUtil.VectorClampMin(ref clampedPoint, ref m_localAabbMin);

            output[0] = MathUtil.GetQuantized(clampedPoint.X);
            output[1] = MathUtil.GetQuantized(clampedPoint.Y);
            output[2] = MathUtil.GetQuantized(clampedPoint.Z);
        }


        protected void GetVertex(int x, int y, out Vector3 vertex)
        {
            Debug.Assert(x >= 0);
            Debug.Assert(y >= 0);
            Debug.Assert(x < m_heightStickWidth);
            Debug.Assert(y < m_heightStickLength);

            float height = GetRawHeightFieldValue(x, y);

            switch (m_upAxis)
            {
                case 0:
                    {
                        vertex = new Vector3(height - m_localOrigin.X,
                            (-m_width / 2f) + x,
                            (-m_length / 2f) + y
                            );
                        break;
                    }
                case 1:
                    {
                        vertex = new Vector3(
                        (-m_width / 2f) + x,
                        height - m_localOrigin.Y,
                        (-m_length / 2f) + y
                        );
                        break;
                    };
                case 2:
                    {
                        vertex = new Vector3(
                        (-m_width / 2f) + x,
                        (-m_length / 2f) + y,
                        height - m_localOrigin.Z
                        );
                        break;
                    }
                default:
                    {
                        //need to get valid m_upAxis
                        Debug.Assert(false);
                        vertex = Vector3.Zero;
                        break;
                    }
            }

            vertex *= m_localScaling;
        }



        /// protected initialization
        /**
          Handles the work of constructors so that public constructors can be
          backwards-compatible without a lot of copy/paste.
         */
        protected void Initialize(int heightStickWidth, int heightStickLength,
                        byte[] heightfieldData, float heightScale,
                        float minHeight, float maxHeight, int upAxis,
                        PHY_ScalarType hdt, bool flipQuadEdges)
        {
            // validation
            Debug.Assert(heightStickWidth > 1, "bad width");
            Debug.Assert(heightStickLength > 1, "bad length");
            Debug.Assert(heightfieldData != null, "null heightfield data");
            // Debug.Assert(heightScale) -- do we care?  Trust caller here
            Debug.Assert(minHeight <= maxHeight, "bad min/max height");
            Debug.Assert(upAxis >= 0 && upAxis < 3,
                "bad upAxis--should be in range [0,2]");
            Debug.Assert(hdt != PHY_ScalarType.PHY_UCHAR || hdt != PHY_ScalarType.PHY_FLOAT || hdt != PHY_ScalarType.PHY_SHORT,
                "Bad height data type enum");

            // initialize member variables
            m_shapeType = BroadphaseNativeType.TERRAIN_SHAPE_PROXYTYPE;
            m_heightStickWidth = heightStickWidth;
            m_heightStickLength = heightStickLength;
            m_minHeight = minHeight;
            m_maxHeight = maxHeight;
            m_width = (heightStickWidth - 1);
            m_length = (heightStickLength - 1);
            m_heightScale = heightScale;
            m_heightFieldData = heightfieldData;
            m_heightDataType = hdt;
            m_flipQuadEdges = flipQuadEdges;
            m_useDiamondSubdivision = false;
            m_upAxis = upAxis;
            m_localScaling = new Vector3(1f);

            // determine min/max axis-aligned bounding box (aabb) values
            switch (m_upAxis)
            {
                case 0:
                    {
                        m_localAabbMin = new Vector3(m_minHeight, 0, 0);
                        m_localAabbMax = new Vector3(m_maxHeight, m_width, m_length);
                        break;
                    }
                case 1:
                    {
                        m_localAabbMin = new Vector3(0, m_minHeight, 0);
                        m_localAabbMax = new Vector3(m_width, m_maxHeight, m_length);
                        break;
                    };
                case 2:
                    {
                        m_localAabbMin = new Vector3(0, 0, m_minHeight);
                        m_localAabbMax = new Vector3(m_width, m_length, m_maxHeight);
                        break;
                    }
                default:
                    {
                        //need to get valid m_upAxis
                        Debug.Assert(false, "Bad m_upAxis");
                        break;
                    }
            }

            // remember origin (defined as exact middle of aabb)
            m_localOrigin = 0.5f * (m_localAabbMin + m_localAabbMax);


        }

        /// preferred constructor
        /**
          This constructor supports a range of heightfield
          data types, and allows for a non-zero minimum height value.
          heightScale is needed for any integer-based heightfield data types.
         */
        public HeightfieldTerrainShape(int heightStickWidth, int heightStickLength,
                                  byte[] heightfieldData, float heightScale,
                                  float minHeight, float maxHeight,
                                  int upAxis, PHY_ScalarType heightDataType,
                                  bool flipQuadEdges)
        {
            Initialize(heightStickWidth, heightStickLength, heightfieldData, heightScale, minHeight, maxHeight, upAxis, heightDataType, flipQuadEdges);
        }

        /// legacy constructor
        /**
          The legacy constructor assumes the heightfield has a minimum height
          of zero.  Only unsigned char or floats are supported.  For legacy
          compatibility reasons, heightScale is calculated as maxHeight / 65535 
          (and is only used when useFloatData = false).
         */
        public HeightfieldTerrainShape(int heightStickWidth, int heightStickLength, byte[] heightfieldData, float maxHeight, int upAxis, bool useFloatData, bool flipQuadEdges)
        {
            // legacy constructor: support only float or unsigned char,
            // 	and min height is zero
            PHY_ScalarType hdt = (useFloatData) ? PHY_ScalarType.PHY_FLOAT : PHY_ScalarType.PHY_UCHAR;
            float minHeight = 0.0f;

            // previously, height = uchar * maxHeight / 65535.
            // So to preserve legacy behavior, heightScale = maxHeight / 65535
            float heightScale = maxHeight / 65535;

            Initialize(heightStickWidth, heightStickLength, heightfieldData,
                       heightScale, minHeight, maxHeight, upAxis, hdt,
                       flipQuadEdges);

        }

        public void SetUseDiamondSubdivision(bool useDiamondSubdivision)
        {
            m_useDiamondSubdivision = useDiamondSubdivision;
        }


        public override void GetAabb(ref Matrix t, out Vector3 aabbMin, out Vector3 aabbMax)
        {
            Vector3 halfExtents = (m_localAabbMax - m_localAabbMin) * m_localScaling * 0.5f;

            Vector3 localOrigin = Vector3.Zero;
            MathUtil.VectorComponent(ref localOrigin, m_upAxis, (m_minHeight + m_maxHeight) * 0.5f);
            localOrigin *= m_localScaling;

            Matrix abs_b;
            MathUtil.AbsoluteMatrix(ref t, out abs_b);
            Vector3 center = t.Translation;
            Vector3 extent = new Vector3(Vector3.Dot(abs_b.Right, halfExtents),
                                            Vector3.Dot(abs_b.Up, halfExtents),
                                            Vector3.Dot(abs_b.Backward, halfExtents));

            extent += new Vector3(Margin);

            aabbMin = center - extent;
            aabbMax = center + extent;

        }

        /// process all triangles within the provided axis-aligned bounding box
        /**
          basic algorithm:
            - convert input aabb to local coordinates (scale down and shift for local origin)
            - convert input aabb to a range of heightfield grid points (quantize)
            - iterate over all triangles in that subset of the grid
         */
        public override void ProcessAllTriangles(ITriangleCallback callback, ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
            // scale down the input aabb's so they are in local (non-scaled) coordinates
            Vector3 invScale = new Vector3(1f) / m_localScaling;

            Vector3 localAabbMin = aabbMin * invScale;
            Vector3 localAabbMax = aabbMax * invScale;

            // account for local origin
            localAabbMin += m_localOrigin;
            localAabbMax += m_localOrigin;

            //quantize the aabbMin and aabbMax, and adjust the start/end ranges
            int[] quantizedAabbMin = new int[3];
            int[] quantizedAabbMax = new int[3];
            QuantizeWithClamp(quantizedAabbMin, ref localAabbMin, 0);
            QuantizeWithClamp(quantizedAabbMax, ref localAabbMax, 1);

            // expand the min/max quantized values
            // this is to catch the case where the input aabb falls between grid points!
            for (int i = 0; i < 3; ++i)
            {
                quantizedAabbMin[i]--;
                quantizedAabbMax[i]++;
            }

            int startX = 0;
            int endX = m_heightStickWidth - 1;
            int startJ = 0;
            int endJ = m_heightStickLength - 1;

            switch (m_upAxis)
            {
                case 0:
                    {
                        if (quantizedAabbMin[1] > startX)
                            startX = quantizedAabbMin[1];
                        if (quantizedAabbMax[1] < endX)
                            endX = quantizedAabbMax[1];
                        if (quantizedAabbMin[2] > startJ)
                            startJ = quantizedAabbMin[2];
                        if (quantizedAabbMax[2] < endJ)
                            endJ = quantizedAabbMax[2];
                        break;
                    }
                case 1:
                    {
                        if (quantizedAabbMin[0] > startX)
                            startX = quantizedAabbMin[0];
                        if (quantizedAabbMax[0] < endX)
                            endX = quantizedAabbMax[0];
                        if (quantizedAabbMin[2] > startJ)
                            startJ = quantizedAabbMin[2];
                        if (quantizedAabbMax[2] < endJ)
                            endJ = quantizedAabbMax[2];
                        break;
                    };
                case 2:
                    {
                        if (quantizedAabbMin[0] > startX)
                            startX = quantizedAabbMin[0];
                        if (quantizedAabbMax[0] < endX)
                            endX = quantizedAabbMax[0];
                        if (quantizedAabbMin[1] > startJ)
                            startJ = quantizedAabbMin[1];
                        if (quantizedAabbMax[1] < endJ)
                            endJ = quantizedAabbMax[1];
                        break;
                    }
                default:
                    {
                        //need to get valid m_upAxis
                        Debug.Assert(false);
                        break;
                    }
            }

            Vector3[] vertices = new Vector3[3];
            for (int j = startJ; j < endJ; j++)
            {
                for (int x = startX; x < endX; x++)
                {
                    if (m_flipQuadEdges || (m_useDiamondSubdivision && (((j + x) & 1) > 0)))
                    {
                        //first triangle
                        GetVertex(x, j, out vertices[0]);
                        GetVertex(x + 1, j, out vertices[1]);
                        GetVertex(x + 1, j + 1, out vertices[2]);
                        callback.ProcessTriangle(vertices, x, j);
                        //second triangle
                        GetVertex(x, j, out vertices[0]);
                        GetVertex(x + 1, j + 1, out vertices[1]);
                        GetVertex(x, j + 1, out vertices[2]);

                        callback.ProcessTriangle(vertices, x, j);
                    }
                    else
                    {
                        //first triangle
                        GetVertex(x, j, out vertices[0]);
                        GetVertex(x, j + 1, out vertices[1]);
                        GetVertex(x + 1, j, out vertices[2]);
                        callback.ProcessTriangle(vertices, x, j);

                        //second triangle
                        GetVertex(x + 1, j, out vertices[0]);
                        GetVertex(x, j + 1, out vertices[1]);
                        GetVertex(x + 1, j + 1, out vertices[2]);
                        callback.ProcessTriangle(vertices, x, j);
                    }
                }
            }
        }

        public override void CalculateLocalInertia(float mass, out Vector3 inertia)
        {
            //moving concave objects not supported
            inertia = Vector3.Zero;
        }

        public override void SetLocalScaling(ref Vector3 scaling)
        {
            m_localScaling = scaling;
        }

        public override Vector3 GetLocalScaling()
        {
            return m_localScaling;
        }

        //debugging
        public override String GetName()
        {
            return "HEIGHTFIELD";
        }
    }
}