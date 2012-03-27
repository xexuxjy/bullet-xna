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
using BulletXNA.LinearMath;

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
        protected BulletXNA.LinearMath.IndexedVector3 m_localAabbMin;
        protected IndexedVector3 m_localAabbMax;
        protected IndexedVector3 m_localOrigin;

        ///terrain data
        protected int m_heightStickWidth;
        protected int m_heightStickLength;
        protected float m_minHeight;
        protected float m_maxHeight;
        protected float m_width;
        protected float m_length;
        protected float m_heightScale;

        protected byte[] m_heightFieldDataByte;
        protected float[] m_heightFieldDataFloat;
        //union
        //{
        //    unsigned char*	m_heightfieldDataUnsignedChar;
        //    short*		m_heightfieldDataShort;
        //    float*			m_heightfieldDataFloat;
        //    void*			m_heightfieldDataUnknown;
        //};

        protected PHY_ScalarType m_heightDataType;
        protected bool m_flipQuadEdges;
        protected bool m_useDiamondSubdivision;

        protected int m_upAxis;

        protected IndexedVector3 m_localScaling;


        class QuadTreeNode
        {
            public int[] quantizedAabbMin = new int[3];
            public int[] quantizedAabbMax = new int[3];
            public QuadTreeNode[] children;

            public Boolean Intersects(int[] raySource, int[] rayTarget)
            {
                return false;
            }

            public void AdjustHeightValues(int yAxis,ref int min,ref int max)
            {
                if (children == null)
                {
                    min = quantizedAabbMin[yAxis];
                    max = quantizedAabbMax[yAxis];
                }
                else
                {
                    int newMin = min;
                    int newMax = max;

                    children[0].AdjustHeightValues(yAxis, ref newMin, ref newMax);
                    if (newMin < min)
                    {
                        min = newMin;
                    }
                    if (newMax > max)
                    {
                        max = newMax;
                    }
                    children[1].AdjustHeightValues(yAxis, ref newMin, ref newMax);
                    if (newMin < min)
                    {
                        min = newMin;
                    }
                    if (newMax > max)
                    {
                        max = newMax;
                    }
                    children[2].AdjustHeightValues(yAxis, ref newMin, ref newMax);
                    if (newMin < min)
                    {
                        min = newMin;
                    }
                    if (newMax > max)
                    {
                        max = newMax;
                    }
                    children[3].AdjustHeightValues(yAxis, ref newMin, ref newMax);
                    if (newMin < min)
                    {
                        min = newMin;
                    }
                    if (newMax > max)
                    {
                        max = newMax;
                    }

                    quantizedAabbMin[yAxis] = min;
                    quantizedAabbMax[yAxis] = max;
                }
            }
        }


        public QuadTreeNode m_rootQuadTreeNode;



        public void RebuildQuadTree(int maxDepth=5)
        {
            m_rootQuadTreeNode = new QuadTreeNode();

            int xAxis = 0;
            int yAxis = 1;
            int zAxis = 2;

            // need these as quantized vals?
            int min = -100;
            int max = 100;

            if (m_upAxis == 0)
            {
                xAxis = 1;
                yAxis = 0;
                zAxis = 2;
            }
            else if(m_upAxis == 2)
            {
                xAxis = 0;
                yAxis = 2;
                zAxis = 1;
            }
            
            BuildNodes(m_rootQuadTreeNode,0,maxDepth,xAxis,yAxis,zAxis);
            // cheat second pass to rebuild heights.
            // adjust heights.
            m_rootQuadTreeNode.AdjustHeightValues(yAxis, ref min, ref max);

        }


        private void BuildNodes(QuadTreeNode parent,int depth,int maxDepth,int xAxis,int yAxis,int zAzis)
        {
            if (depth < maxDepth)
            {
                if (parent.children == null)
                {
                    parent.children = new QuadTreeNode[4];
                    //split nodes


                    // do something funky to build the heights.
                    // adjust for up axis.
                    int diff1 = (parent.quantizedAabbMax[xAxis] - parent.quantizedAabbMin[xAxis]) / 2;
                    int diff2 = (parent.quantizedAabbMax[yAxis] - parent.quantizedAabbMin[yAxis]) / 2;
                    int diff3 = (parent.quantizedAabbMax[zAzis] - parent.quantizedAabbMin[zAzis]) / 2;

                    for (int i = 0; i < 4; ++i)
                    {
                        parent.children[i].quantizedAabbMin[xAxis] = parent.quantizedAabbMin[xAxis] + diff1;
                        parent.children[i].quantizedAabbMin[zAzis] = parent.quantizedAabbMin[zAzis] + diff3;
                        parent.children[i].quantizedAabbMax[xAxis] = parent.quantizedAabbMax[xAxis] - diff1;
                        parent.children[i].quantizedAabbMax[zAzis] = parent.quantizedAabbMax[zAzis] - diff3;
                    }
                }
            }
            else
            {
                // at lowest depth, we should now go through and find min/max height values for each vertex.
                InspectVertexHeights();

            }
        }

        public bool HasAccelerator()
        {
            return m_rootQuadTreeNode != null;
        }


        private void InspectVertexHeights(int startX,int endX,int startJ,int endJ,int upAxis,ref int min,ref int max)
        {
                IndexedVector3 vertex = new IndexedVector3();
                for (int j = startJ; j < endJ; j++)
                {
                    for (int x = startX; x < endX; x++)
                    {
                        GetVertex(x, j, out vertex);
                        int quantizedHeight = MathUtil.GetQuantized(vertex[upAxis]);
                        if (quantizedHeight < min)
                        {
                            min = quantizedHeight;
                        }
                        if (quantizedHeight > max)
                        {
                            max = quantizedHeight;
                        }
                    }
                }
            }




        public void PerformRaycast(ITriangleCallback callback, ref IndexedVector3 raySource, ref IndexedVector3 rayTarget)
        {
            ObjectArray<QuadTreeNode> results = new ObjectArray<QuadTreeNode>();
            // quantize source and target.
            int[] quantizedSource = new int[3];
            int[] quantizedTarget = new int[3];

            QuantizeWithClamp(quantizedSource, ref raySource, 0);
            QuantizeWithClamp(quantizedTarget, ref rayTarget, 1);

            // build list of squares
            QueryNode(m_rootQuadTreeNode, results, quantizedSource, quantizedTarget);

            // go through each of the results.
            foreach (QuadTreeNode quadTreeNode in results)
            {
                int startX = 0;
                int endX = m_heightStickWidth - 1;
                int startJ = 0;
                int endJ = m_heightStickLength - 1;

                switch (m_upAxis)
                {
                    case 0:
                        {
                            if (quadTreeNode.quantizedAabbMin[1] > startX)
                                startX = quadTreeNode.quantizedAabbMin[1];
                            if (quadTreeNode.quantizedAabbMax[1] < endX)
                                endX = quadTreeNode.quantizedAabbMax[1];
                            if (quadTreeNode.quantizedAabbMin[2] > startJ)
                                startJ = quadTreeNode.quantizedAabbMin[2];
                            if (quadTreeNode.quantizedAabbMax[2] < endJ)
                                endJ = quadTreeNode.quantizedAabbMax[2];
                            break;
                        }
                    case 1:
                        {
                            if (quadTreeNode.quantizedAabbMin[0] > startX)
                                startX = quadTreeNode.quantizedAabbMin[0];
                            if (quadTreeNode.quantizedAabbMax[0] < endX)
                                endX = quadTreeNode.quantizedAabbMax[0];
                            if (quadTreeNode.quantizedAabbMin[2] > startJ)
                                startJ = quadTreeNode.quantizedAabbMin[2];
                            if (quadTreeNode.quantizedAabbMax[2] < endJ)
                                endJ = quadTreeNode.quantizedAabbMax[2];
                            break;
                        };
                    case 2:
                        {
                            if (quadTreeNode.quantizedAabbMin[0] > startX)
                                startX = quadTreeNode.quantizedAabbMin[0];
                            if (quadTreeNode.quantizedAabbMax[0] < endX)
                                endX = quadTreeNode.quantizedAabbMax[0];
                            if (quadTreeNode.quantizedAabbMin[1] > startJ)
                                startJ = quadTreeNode.quantizedAabbMin[1];
                            if (quadTreeNode.quantizedAabbMax[1] < endJ)
                                endJ = quadTreeNode.quantizedAabbMax[1];
                            break;
                        }
                    default:
                        {
                            //need to get valid m_upAxis
                            Debug.Assert(false);
                            break;
                        }
                }

                IndexedVector3[] vertices = new IndexedVector3[3];
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
        }


        public void QueryNode(QuadTreeNode node, ObjectArray<QuadTreeNode> results, int[] raySource, int[] rayTarget)
        {
            //if(node.children == null && node.Intersects(raySource,rayTarget))
            // add the lowest level.
            if (node.children == null)
            {
                results.Add(node);
            }
            else
            {
                // simple rescursive for now.
                for (int i = 0; i < 4; ++i)
                {
                    if (node.children[i].Intersects(raySource, rayTarget))
                    {
                        QueryNode(node.children[i], results, raySource, rayTarget);
                    }
                }
            }
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

        public HeightfieldTerrainShape(int heightStickWidth, int heightStickLength,
                                  float[] heightfieldData, float heightScale,
                                  float minHeight, float maxHeight,
                                  int upAxis, bool flipQuadEdges)
        {
            Initialize(heightStickWidth, heightStickLength, heightfieldData, heightScale, minHeight, maxHeight, upAxis, PHY_ScalarType.PHY_FLOAT, flipQuadEdges);
        }



        protected virtual float GetRawHeightFieldValue(int x, int y)
        {
            float val = 0f;
            switch (m_heightDataType)
            {
                case PHY_ScalarType.PHY_FLOAT:
                    {
                    if(m_heightFieldDataFloat != null)
                    {
                            // float offset (4 for sizeof)
                            int index = ((y * m_heightStickWidth) + x);
                            val = m_heightFieldDataFloat[index];
                    }
                    else
                    {
                        // float offset (4 for sizeof)
                        int index = ((y * m_heightStickWidth) + x) * 4;
                        val = BitConverter.ToSingle(m_heightFieldDataByte, index);
                    }
                        break;
                    }

                case PHY_ScalarType.PHY_UCHAR:
                    {
                        byte heightFieldValue = m_heightFieldDataByte[(y * m_heightStickWidth) + x];
                        val = heightFieldValue * m_heightScale;
                        break;
                    }

                case PHY_ScalarType.PHY_SHORT:
                    {
                        int index = ((y * m_heightStickWidth) + x) * 2;
                        short hfValue = BitConverter.ToInt16(m_heightFieldDataByte, index);
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

        protected void QuantizeWithClamp(int[] output, ref IndexedVector3 point, int isMax)
        {
            /// given input vector, return quantized version
            /**
              This routine is basically determining the gridpoint indices for a given
              input vector, answering the question: "which gridpoint is closest to the
              provided point?".

              "with clamp" means that we restrict the point to be in the heightfield's
              axis-aligned bounding box.
             */
            IndexedVector3 clampedPoint = point;
            MathUtil.VectorClampMax(ref clampedPoint, ref m_localAabbMax);
            MathUtil.VectorClampMin(ref clampedPoint, ref m_localAabbMin);

            output[0] = MathUtil.GetQuantized(clampedPoint.X);
            output[1] = MathUtil.GetQuantized(clampedPoint.Y);
            output[2] = MathUtil.GetQuantized(clampedPoint.Z);
        }


        protected void GetVertex(int x, int y, out IndexedVector3 vertex)
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
                        vertex = new IndexedVector3(height - m_localOrigin.X,
                            (-m_width / 2f) + x,
                            (-m_length / 2f) + y
                            );
                        break;
                    }
                case 1:
                    {
                        vertex = new IndexedVector3(
                        (-m_width / 2f) + x,
                        height - m_localOrigin.Y,
                        (-m_length / 2f) + y
                        );
                        break;
                    };
                case 2:
                    {
                        vertex = new IndexedVector3(
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
                        vertex = IndexedVector3.Zero;
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
                        Object heightfieldData, float heightScale,
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
            m_shapeType = BroadphaseNativeTypes.TERRAIN_SHAPE_PROXYTYPE;
            m_heightStickWidth = heightStickWidth;
            m_heightStickLength = heightStickLength;
            m_minHeight = minHeight;
            m_maxHeight = maxHeight;
            m_width = (heightStickWidth - 1);
            m_length = (heightStickLength - 1);
            m_heightScale = heightScale;
            // copy the data in 
            m_heightFieldDataByte = heightfieldData as byte[];
            m_heightFieldDataFloat = heightfieldData as float[]; 
            m_heightDataType = hdt;

            m_flipQuadEdges = flipQuadEdges;
            m_useDiamondSubdivision = false;
            m_upAxis = upAxis;
            m_localScaling = new IndexedVector3(1f);

            // determine min/max axis-aligned bounding box (aabb) values
            switch (m_upAxis)
            {
                case 0:
                    {
                        m_localAabbMin = new IndexedVector3(m_minHeight, 0, 0);
                        m_localAabbMax = new IndexedVector3(m_maxHeight, m_width, m_length);
                        break;
                    }
                case 1:
                    {
                        m_localAabbMin = new IndexedVector3(0, m_minHeight, 0);
                        m_localAabbMax = new IndexedVector3(m_width, m_maxHeight, m_length);
                        break;
                    };
                case 2:
                    {
                        m_localAabbMin = new IndexedVector3(0, 0, m_minHeight);
                        m_localAabbMax = new IndexedVector3(m_width, m_length, m_maxHeight);
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

 
        public void SetUseDiamondSubdivision(bool useDiamondSubdivision)
        {
            m_useDiamondSubdivision = useDiamondSubdivision;
        }


        public override void GetAabb(ref IndexedMatrix t, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            IndexedVector3 halfExtents = (m_localAabbMax - m_localAabbMin) * m_localScaling * 0.5f;

            IndexedVector3 localOrigin = IndexedVector3.Zero;
            localOrigin[m_upAxis] =  (m_minHeight + m_maxHeight) * 0.5f;
            localOrigin *= m_localScaling;

            IndexedBasisMatrix abs_b = t._basis.Absolute();
            IndexedVector3 center = t._origin;
            IndexedVector3 extent = new IndexedVector3(abs_b[0].Dot(ref halfExtents),
		                                                abs_b[1].Dot(ref halfExtents),
		                                                abs_b[2].Dot(ref halfExtents));

            extent += new IndexedVector3(GetMargin());

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
        public override void ProcessAllTriangles(ITriangleCallback callback, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
            // scale down the input aabb's so they are in local (non-scaled) coordinates
            IndexedVector3 invScale = new IndexedVector3(1f) / m_localScaling;

            IndexedVector3 localAabbMin = aabbMin * invScale;
            IndexedVector3 localAabbMax = aabbMax * invScale;

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

            IndexedVector3[] vertices = new IndexedVector3[3];
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

        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {
            //moving concave objects not supported
            inertia = IndexedVector3.Zero;
        }

        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {
            m_localScaling = scaling;
        }

        public override IndexedVector3 GetLocalScaling()
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