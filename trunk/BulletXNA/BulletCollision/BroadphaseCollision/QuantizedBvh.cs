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

#define DEBUG_CHECK_DEQUANTIZATION
#define DEBUG_TREE_BUILDING
#define VISUALLY_ANALYZE_BVH
#define RAYAABB2

using System;
using System.Collections.Generic;

using System.Diagnostics;
using Microsoft.Xna.Framework;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision.BroadphaseCollision
{


    public interface INodeOverlapCallback
    {
        void ProcessNode(int subPart, int triangleIndex);
        void Cleanup();
    }

    ///btQuantizedBvhNode is a compressed aabb node, 16 bytes.
    ///Node can be used for leafnode or internal node. Leafnodes can point to 32-bit triangle index (non-negative range).
    ///

    public class QuantizedBvhNode
    {
        //12 bytes - hah yeah right...
        public UShortVector3 m_quantizedAabbMin;
        public UShortVector3 m_quantizedAabbMax;

        //4 bytes
        public int m_escapeIndexOrTriangleIndex;

        public bool IsLeafNode()
        {
            //skipindex is negative (internal node), triangleindex >=0 (leafnode)
            return (m_escapeIndexOrTriangleIndex >= 0);
        }
        public int GetEscapeIndex()
        {
            Debug.Assert(!IsLeafNode());
            return -m_escapeIndexOrTriangleIndex;
        }

        public int GetTriangleIndex()
        {
            // relax this as we're using it in a cheat on the recursive walker.
            //Debug.Assert(isLeafNode());
            // Get only the lower bits where the triangle index is stored
            int result = (m_escapeIndexOrTriangleIndex & ~((~0) << (31 - QuantizedBvh.MAX_NUM_PARTS_IN_BITS)));
            int result2 = (m_escapeIndexOrTriangleIndex & ~((~0) << (31 - QuantizedBvh.MAX_NUM_PARTS_IN_BITS)));

            return result;
        }
        public int GetPartId()
        {
            Debug.Assert(IsLeafNode());
            // Get only the highest bits where the part index is stored
            return (m_escapeIndexOrTriangleIndex >> (31 - QuantizedBvh.MAX_NUM_PARTS_IN_BITS));
        }
    }

    /// btOptimizedBvhNode contains both internal and leaf node information.
    /// Total node size is 44 bytes / node. You can use the compressed version of 16 bytes.
    public class OptimizedBvhNode
    {
        public OptimizedBvhNode()
        {

            int ibreak = 0;
        }
        //32 bytes
        public Vector3 m_aabbMinOrg;
        public Vector3 m_aabbMaxOrg;

        //4
        public int m_escapeIndex;

        //8
        //for child nodes
        public int m_subPart;
        public int m_triangleIndex;
        //int	m_padding[5];//bad, due to alignment
    }


    ///btBvhSubtreeInfo provides info to gather a subtree of limited size
    public class BvhSubtreeInfo
    {
        //12 bytes - hah yeah right...

        public UShortVector3 m_quantizedAabbMin;
        public UShortVector3 m_quantizedAabbMax;

        //4 bytes, points to the root of the subtree
        public int m_rootNodeIndex;
        //4 bytes
        public int m_subtreeSize;
        //int			m_padding[3];

        public void SetAabbFromQuantizeNode(QuantizedBvhNode quantizedNode)
        {
            m_quantizedAabbMin = quantizedNode.m_quantizedAabbMin;
            m_quantizedAabbMax = quantizedNode.m_quantizedAabbMax;
        }
    }

    public enum TraversalMode
    {
        TRAVERSAL_STACKLESS = 0,
        TRAVERSAL_STACKLESS_CACHE_FRIENDLY,
        TRAVERSAL_RECURSIVE
    }

    ///The btQuantizedBvh class stores an AABB tree that can be quickly traversed on CPU and Cell SPU.
    ///It is used by the btBvhTriangleMeshShape as midphase, and by the btMultiSapBroadphase.
    ///It is recommended to use quantization for better performance and lower memory requirements.
    public class QuantizedBvh
    {
        //Note: currently we have 16 bytes per quantized node
        // MAN - cheat here rather then using size-of , calc a fixed number?
        public const int MAX_SUBTREE_SIZE_IN_BYTES = 2048 / 16;

        // 10 gives the potential for 1024 parts, with at most 2^21 (2097152) (minus one
        // actually) triangles each (since the sign bit is reserved
        public const int MAX_NUM_PARTS_IN_BITS = 10;

        public static int gStackDepth = 0;
        public static int gMaxStackDepth = 0;

        protected Vector3 m_bvhAabbMin;
        protected Vector3 m_bvhAabbMax;
        protected Vector3 m_bvhQuantization;

        protected int m_bulletVersion;	//for serialization versioning. It could also be used to detect endianess.

        protected int m_curNodeIndex;
        //quantization data
        protected bool m_useQuantization;

        protected ObjectArray<OptimizedBvhNode> m_leafNodes = new ObjectArray<OptimizedBvhNode>();
        protected ObjectArray<OptimizedBvhNode> m_contiguousNodes = new ObjectArray<OptimizedBvhNode>();
        protected ObjectArray<QuantizedBvhNode> m_quantizedLeafNodes = new ObjectArray<QuantizedBvhNode>();
        protected ObjectArray<QuantizedBvhNode> m_quantizedContiguousNodes = new ObjectArray<QuantizedBvhNode>();

        protected TraversalMode m_traversalMode;
        protected ObjectArray<BvhSubtreeInfo> m_SubtreeHeaders = new ObjectArray<BvhSubtreeInfo>();

        //This is only used for serialization so we don't have to add serialization directly to btAlignedObjectArray
        protected int m_subtreeHeaderCount;

        public int m_maxIterations;

        public QuantizedBvh()
        {
            m_bulletVersion = BulletGlobals.BT_BULLET_VERSION;
            m_useQuantization = false;
            //m_traversalMode = TraversalMode.TRAVERSAL_STACKLESS_CACHE_FRIENDLY;
            m_traversalMode = TraversalMode.TRAVERSAL_STACKLESS;
            //m_traversalMode = TraversalMode.TRAVERSAL_RECURSIVE;
            m_subtreeHeaderCount = 0; //PCK: add this line
            m_bvhAabbMin = new Vector3(-MathUtil.SIMD_INFINITY);
            m_bvhAabbMax = new Vector3(MathUtil.SIMD_INFINITY);
        }


        public virtual void Cleanup()
        {
        }

        ///two versions, one for quantized and normal nodes. This allows code-reuse while maintaining readability (no template/macro!)
        ///this might be refactored into a virtual, it is usually not calculated at run-time
        protected void SetInternalNodeAabbMin(int nodeIndex, ref Vector3 aabbMin)
        {
            if (m_useQuantization)
            {
                QuantizedBvhNode bvh = m_quantizedContiguousNodes[nodeIndex];
                Quantize(out bvh.m_quantizedAabbMin, ref aabbMin, false);
                m_quantizedContiguousNodes[nodeIndex] = bvh;
            }
            else
            {
                OptimizedBvhNode bvh = m_contiguousNodes[nodeIndex];
                bvh.m_aabbMinOrg = aabbMin;
                m_contiguousNodes[nodeIndex] = bvh;
            }
        }
        public void SetInternalNodeAabbMax(int nodeIndex, ref Vector3 aabbMax)
        {
            if (m_useQuantization)
            {
                QuantizedBvhNode bvh = m_quantizedContiguousNodes[nodeIndex];
                Quantize(out bvh.m_quantizedAabbMax, ref aabbMax, true);
                m_quantizedContiguousNodes[nodeIndex] = bvh;
            }
            else
            {
                OptimizedBvhNode bvh = m_contiguousNodes[nodeIndex];
                bvh.m_aabbMaxOrg = aabbMax;
                m_contiguousNodes[nodeIndex] = bvh;
            }
        }

        public Vector3 GetAabbMin(int nodeIndex)
        {
            if (m_useQuantization)
            {
                Vector3 output;
                UnQuantize(ref m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMin, out output);
                return output;
            }
            //non-quantized
            return m_leafNodes[nodeIndex].m_aabbMinOrg;

        }
        public Vector3 GetAabbMax(int nodeIndex)
        {
            if (m_useQuantization)
            {
                Vector3 output;
                UnQuantize(ref m_quantizedLeafNodes[nodeIndex].m_quantizedAabbMax, out output);
                return output;
            }
            //non-quantized
            return m_leafNodes[nodeIndex].m_aabbMaxOrg;
        }

        public void SetInternalNodeEscapeIndex(int nodeIndex, int escapeIndex)
        {
            if (m_useQuantization)
            {
                m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = -escapeIndex;
            }
            else
            {
                m_contiguousNodes[nodeIndex].m_escapeIndex = escapeIndex;
            }


        }

        public void MergeInternalNodeAabb(int nodeIndex, Vector3 newAabbMin, Vector3 newAabbMax)
        {
            MergeInternalNodeAabb(nodeIndex, ref newAabbMin, ref newAabbMax);
        }

        public void MergeInternalNodeAabb(int nodeIndex, ref Vector3 newAabbMin, ref Vector3 newAabbMax)
        {
            if (m_useQuantization)
            {
                UShortVector3 quantizedAabbMin;
                UShortVector3 quantizedAabbMax;
                Quantize(out quantizedAabbMin, ref newAabbMin, false);
                Quantize(out quantizedAabbMax, ref newAabbMax, true);

                QuantizedBvhNode node = m_quantizedContiguousNodes[nodeIndex];
                node.m_quantizedAabbMin.min(ref quantizedAabbMin);
                node.m_quantizedAabbMax.max(ref quantizedAabbMax);
                m_quantizedContiguousNodes[nodeIndex] = node;
            }
            else
            {
                OptimizedBvhNode node = m_contiguousNodes[nodeIndex];
                //non-quantized
                MathUtil.VectorMin(ref newAabbMin, ref node.m_aabbMinOrg);
                MathUtil.VectorMin(ref newAabbMax, ref node.m_aabbMaxOrg);
                m_contiguousNodes[nodeIndex] = node;
            }
        }

        public void SwapLeafNodes(int firstIndex, int secondIndex)
        {
            if (m_useQuantization)
            {
                QuantizedBvhNode tmp = m_quantizedLeafNodes[firstIndex];
                m_quantizedLeafNodes[firstIndex] = m_quantizedLeafNodes[secondIndex];
                m_quantizedLeafNodes[secondIndex] = tmp;
            }
            else
            {
                OptimizedBvhNode tmp = m_leafNodes[firstIndex];
                m_leafNodes[firstIndex] = m_leafNodes[secondIndex];
                m_leafNodes[secondIndex] = tmp;
            }
        }

        public void AssignInternalNodeFromLeafNode(int internalNode, int leafNodeIndex)
        {
            if (m_useQuantization)
            {
                m_quantizedContiguousNodes[internalNode] = m_quantizedLeafNodes[leafNodeIndex];
            }
            else
            {
                m_contiguousNodes[internalNode] = m_leafNodes[leafNodeIndex];
            }
        }


        protected void BuildTree(int startIndex, int endIndex)
        {
#if DEBUG_TREE_BUILDING
            gStackDepth++;
            if (gStackDepth > gMaxStackDepth)
            {
                gMaxStackDepth = gStackDepth;
            }
#endif //DEBUG_TREE_BUILDING


            int splitAxis, splitIndex, i;
            int numIndices = endIndex - startIndex;
            int curIndex = m_curNodeIndex;

            Debug.Assert(numIndices > 0);

            if (numIndices == 1)
            {
#if DEBUG_TREE_BUILDING
                gStackDepth--;
#endif //DEBUG_TREE_BUILDING

                AssignInternalNodeFromLeafNode(m_curNodeIndex, startIndex);

                m_curNodeIndex++;
                return;
            }
            //calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.

            splitAxis = CalcSplittingAxis(startIndex, endIndex);

            splitIndex = SortAndCalcSplittingIndex(startIndex, endIndex, splitAxis);

            int internalNodeIndex = m_curNodeIndex;

            //set the min aabb to 'inf' or a max value, and set the max aabb to a -inf/minimum value.
            //the aabb will be expanded during buildTree/mergeInternalNodeAabb with actual node values
            SetInternalNodeAabbMin(m_curNodeIndex, ref m_bvhAabbMax);//can't use Vector3(SIMD_INFINITY,SIMD_INFINITY,SIMD_INFINITY)) because of quantization
            SetInternalNodeAabbMax(m_curNodeIndex, ref m_bvhAabbMin);//can't use Vector3(-SIMD_INFINITY,-SIMD_INFINITY,-SIMD_INFINITY)) because of quantization

            for (i = startIndex; i < endIndex; i++)
            {
                MergeInternalNodeAabb(m_curNodeIndex, GetAabbMin(i), GetAabbMax(i));
            }

            m_curNodeIndex++;


            //internalNode->m_escapeIndex;

            int leftChildNodexIndex = m_curNodeIndex;

            //build left child tree
            BuildTree(startIndex, splitIndex);

            int rightChildNodexIndex = m_curNodeIndex;
            //build right child tree
            BuildTree(splitIndex, endIndex);

#if DEBUG_TREE_BUILDING
            gStackDepth--;
#endif //DEBUG_TREE_BUILDING

            int escapeIndex = m_curNodeIndex - curIndex;

            if (m_useQuantization)
            {
                //escapeIndex is the number of nodes of this subtree

                //const int sizeQuantizedNode =sizeof(btQuantizedBvhNode);
                //const int treeSizeInBytes = escapeIndex * sizeQuantizedNode;
                // MAN - Cheat and use the number of nodes rather then size.
                if (escapeIndex > MAX_SUBTREE_SIZE_IN_BYTES)
                {
                    UpdateSubtreeHeaders(leftChildNodexIndex, rightChildNodexIndex);
                }
            }
            else
            {

            }

            SetInternalNodeEscapeIndex(internalNodeIndex, escapeIndex);
        }

        protected int CalcSplittingAxis(int startIndex, int endIndex)
        {
            Vector3 means = Vector3.Zero;
            Vector3 variance = Vector3.Zero;
            int numIndices = endIndex - startIndex;

            for (int i = startIndex; i < endIndex; i++)
            {
                Vector3 center = 0.5f * (GetAabbMax(i) + GetAabbMin(i));
                means += center;
            }
            means *= (1f / numIndices);

            for (int i = startIndex; i < endIndex; i++)
            {
                Vector3 center = 0.5f * (GetAabbMax(i) + GetAabbMin(i));
                Vector3 diff2 = center - means;
                diff2 = diff2 * diff2;
                variance += diff2;
            }
            variance *= (1f / ((float)(numIndices - 1)));

            return MathUtil.MaxAxis(ref variance);

        }

        protected int SortAndCalcSplittingIndex(int startIndex, int endIndex, int splitAxis)
        {
            int splitIndex = startIndex;
            int numIndices = endIndex - startIndex;
            float splitValue;

            Vector3 means = Vector3.Zero;
            for (int i = startIndex; i < endIndex; i++)
            {
                Vector3 center = 0.5f * (GetAabbMax(i) + GetAabbMin(i));
                means += center;
            }
            means *= (1f / (float)numIndices);

            splitValue = MathUtil.VectorComponent(ref means, splitAxis);

            //sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
            for (int i = startIndex; i < endIndex; i++)
            {
                Vector3 center = 0.5f * (GetAabbMax(i) + GetAabbMin(i));
                if (MathUtil.VectorComponent(ref center, splitAxis) > splitValue)
                {
                    //swap
                    SwapLeafNodes(i, splitIndex);
                    splitIndex++;
                }
            }

            //if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex and endIndex
            //otherwise the tree-building might fail due to stack-overflows in certain cases.
            //unbalanced1 is unsafe: it can cause stack overflows
            //bool unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));

            //unbalanced2 should work too: always use center (perfect balanced trees)	
            //bool unbalanced2 = true;

            //this should be safe too:
            int rangeBalancedIndices = numIndices / 3;
            bool unbalanced = ((splitIndex <= (startIndex + rangeBalancedIndices)) || (splitIndex >= (endIndex - 1 - rangeBalancedIndices)));

            if (unbalanced)
            {
                splitIndex = startIndex + (numIndices >> 1);
            }

            bool unbal = (splitIndex == startIndex) || (splitIndex == (endIndex));
            //(void)unbal;
            Debug.Assert(!unbal);

            return splitIndex;
        }

        protected void WalkStacklessTree(INodeOverlapCallback nodeCallback, ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
            Debug.Assert(!m_useQuantization);
            int escapeIndex, curIndex = 0;

            OptimizedBvhNode rootNode = m_contiguousNodes[curIndex];
            int walkIterations = 0;
            bool isLeafNode;
            //PCK: unsigned instead of bool
            bool aabbOverlap;

            while (curIndex < m_curNodeIndex)
            {
                //catch bugs in tree data
                Debug.Assert(walkIterations < m_curNodeIndex);

                walkIterations++;
                aabbOverlap = AabbUtil2.TestAabbAgainstAabb2(ref aabbMin, ref aabbMax, ref rootNode.m_aabbMinOrg, ref rootNode.m_aabbMaxOrg);
                isLeafNode = rootNode.m_escapeIndex == -1;

                //PCK: unsigned instead of bool
                if (isLeafNode && (aabbOverlap))
                {
                    nodeCallback.ProcessNode(rootNode.m_subPart, rootNode.m_triangleIndex);
                }

                //PCK: unsigned instead of bool
                if ((aabbOverlap) || isLeafNode)
                {
                    curIndex++;
                }
                else
                {
                    escapeIndex = rootNode.m_escapeIndex;
                    curIndex += escapeIndex;
                }
                rootNode = m_contiguousNodes[curIndex];

            }
            if (m_maxIterations < walkIterations)
            {
                m_maxIterations = walkIterations;
            }
        }

        protected void WalkStacklessQuantizedTreeAgainstRay(INodeOverlapCallback nodeCallback, ref Vector3 raySource, ref Vector3 rayTarget, ref Vector3 aabbMin, ref Vector3 aabbMax, int startNodeIndex, int endNodeIndex)
        {
            Debug.Assert(m_useQuantization);

            int curIndex = startNodeIndex;
            int walkIterations = 0;
            int subTreeSize = endNodeIndex - startNodeIndex;
            //(void)subTreeSize;
            QuantizedBvhNode rootNode = m_quantizedContiguousNodes[curIndex];
            int escapeIndex = 0;

            bool isLeafNode = false;
            //PCK: unsigned instead of bool
            bool boxBoxOverlap = false;
            bool rayBoxOverlap = false;

            float lambda_max = 1.0f;

#if RAYAABB2
            Vector3 rayDirection = (rayTarget - raySource);
            rayDirection.Normalize();
            lambda_max = Vector3.Dot(rayDirection, rayTarget - raySource);
            ///what about division by zero? --> just set rayDirection[i] to 1.0
            rayDirection.X = MathUtil.FuzzyZero(rayDirection.X) ? MathUtil.BT_LARGE_FLOAT : 1f / rayDirection.X;
            rayDirection.Y = MathUtil.FuzzyZero(rayDirection.Y) ? MathUtil.BT_LARGE_FLOAT : 1f / rayDirection.Y;
            rayDirection.Z = MathUtil.FuzzyZero(rayDirection.Z) ? MathUtil.BT_LARGE_FLOAT : 1f / rayDirection.Z;

            bool[] sign = new bool[] { rayDirection.X < 0.0f, rayDirection.Y < 0.0f, rayDirection.Z < 0.0f };
#endif

            /* Quick pruning by quantized box */
            Vector3 rayAabbMin = raySource;
            Vector3 rayAabbMax = raySource;
            MathUtil.VectorMin(ref rayTarget, ref rayAabbMin);
            MathUtil.VectorMax(ref rayTarget, ref rayAabbMax);

            /* Add box cast extents to bounding box */
            rayAabbMin += aabbMin;
            rayAabbMax += aabbMax;

            UShortVector3 quantizedQueryAabbMin;
            UShortVector3 quantizedQueryAabbMax;
            QuantizeWithClamp(out quantizedQueryAabbMin, ref rayAabbMin, false);
            QuantizeWithClamp(out quantizedQueryAabbMax, ref rayAabbMax, true);

            while (curIndex < endNodeIndex)
            {

                //#define VISUALLY_ANALYZE_BVH 1
#if VISUALLY_ANALYZE_BVH
                //some code snippet to debugDraw aabb, to visually analyze bvh structure
                int drawPatch = 3;
                //need some global access to a debugDrawer
                IDebugDraw debugDrawerPtr = BulletGlobals.gDebugDraw;
                //IDebugDraw debugDrawerPtr = null;
                //if (curIndex == drawPatch&& debugDrawerPtr != null)
                if (debugDrawerPtr != null && curIndex == drawPatch)
                {
                    Vector3 aabbMin2;
                    Vector3 aabbMax2;
                    UnQuantize(ref rootNode.m_quantizedAabbMin, out aabbMin2);
                    UnQuantize(ref rootNode.m_quantizedAabbMax, out aabbMax2);
                    Vector3 color = new Vector3(1f / curIndex, 0, 0);
                    debugDrawerPtr.DrawAabb(ref aabbMin2, ref aabbMax2, ref color);
                    //Console.Out.WriteLine(String.Format("min[{0},{1},{2}] max[{3},{4},{5}]\n", aabbMin.X, aabbMin.Y, aabbMin.Z, aabbMax.X, aabbMax.Y, aabbMax.Z));

                }
#endif//VISUALLY_ANALYZE_BVH

                //catch bugs in tree data
                Debug.Assert(walkIterations < subTreeSize);

                walkIterations++;
                //PCK: unsigned instead of bool
                // only interested if this is closer than any previous hit
                float param = 1.0f;
                rayBoxOverlap = false;
                boxBoxOverlap = AabbUtil2.TestQuantizedAabbAgainstQuantizedAabb(ref quantizedQueryAabbMin, ref quantizedQueryAabbMax, ref rootNode.m_quantizedAabbMin, ref rootNode.m_quantizedAabbMax);
                isLeafNode = rootNode.IsLeafNode();
                if (boxBoxOverlap)
                {
                    Vector3[] bounds = new Vector3[2];
                    UnQuantize(ref rootNode.m_quantizedAabbMin, out bounds[0]);
                    UnQuantize(ref rootNode.m_quantizedAabbMax, out bounds[1]);
                    /* Add box cast extents */
                    bounds[0] -= aabbMax;
                    bounds[1] -= aabbMin;
                    Vector3 normal;
#if false
			        bool ra2 = btRayAabb2 (raySource, rayDirection, sign, bounds, param, 0.0, lambda_max);
			        bool ra = btRayAabb (raySource, rayTarget, bounds[0], bounds[1], param, normal);
			        if (ra2 != ra)
			        {
				        printf("functions don't match\n");
			        }
#endif
#if RAYAABB2
                    ///careful with this check: need to check division by zero (above) and fix the unQuantize method
                    ///thanks Joerg/hiker for the reproduction case!
                    ///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858

                    BulletGlobals.StartProfile("btRayAabb2");
                    rayBoxOverlap = AabbUtil2.RayAabb2(ref raySource, ref rayDirection, sign, bounds, out param, 0.0f, lambda_max);
                    BulletGlobals.StopProfile();
#else
                    rayBoxOverlap = true;//btRayAabb(raySource, rayTarget, bounds[0], bounds[1], param, normal);
#endif
                }

                if (isLeafNode && rayBoxOverlap)
                {
                    nodeCallback.ProcessNode(rootNode.GetPartId(), rootNode.GetTriangleIndex());
                }

                //PCK: unsigned instead of bool
                if ((rayBoxOverlap) || isLeafNode)
                {
                    curIndex++;
                }
                else
                {
                    escapeIndex = rootNode.GetEscapeIndex();
                    curIndex += escapeIndex;
                }
                rootNode = m_quantizedContiguousNodes[curIndex];
            }
            if (m_maxIterations < walkIterations)
            {
                m_maxIterations = walkIterations;
            }
        }

        protected void WalkStacklessQuantizedTree(INodeOverlapCallback nodeCallback, ref UShortVector3 quantizedQueryAabbMin, ref UShortVector3 quantizedQueryAabbMax, int startNodeIndex, int endNodeIndex)
        {
            Debug.Assert(m_useQuantization);

            int curIndex = startNodeIndex;
            int walkIterations = 0;
            int subTreeSize = endNodeIndex - startNodeIndex;
            //(void)subTreeSize;
            QuantizedBvhNode rootNode = m_quantizedContiguousNodes[curIndex];
            int escapeIndex = 0;

            bool isLeafNode;
            //PCK: unsigned instead of bool
            bool aabbOverlap = false;

            while (curIndex < endNodeIndex)
            {

                //#define VISUALLY_ANALYZE_BVH 1
#if VISUALLY_ANALYZE_BVH
                //some code snippet to debugDraw aabb, to visually analyze bvh structure
                int drawPatch = 9;
                //need some global access to a debugDrawer
                IDebugDraw debugDrawerPtr = BulletGlobals.gDebugDraw;
                //IDebugDraw debugDrawerPtr = null;
                //if (curIndex == drawPatch&& debugDrawerPtr != null)
                if (debugDrawerPtr != null && curIndex == drawPatch)
                //if (debugDrawerPtr != null)
                {
                    Vector3 aabbMin, aabbMax;
                    UnQuantize(ref rootNode.m_quantizedAabbMin, out aabbMin);
                    UnQuantize(ref rootNode.m_quantizedAabbMax, out aabbMax);
                    Vector3 color = new Vector3(1, 0, 0);
                    debugDrawerPtr.DrawAabb(ref aabbMin, ref aabbMax, ref color);
                    //Console.Out.WriteLine(String.Format("min[{0},{1},{2}] max[{3},{4},{5}]\n", aabbMin.X, aabbMin.Y, aabbMin.Z, aabbMax.X, aabbMax.Y, aabbMax.Z));


                }
#endif//VISUALLY_ANALYZE_BVH

                //unQuantize version with out param


                //catch bugs in tree data
                Debug.Assert(walkIterations < subTreeSize);

                walkIterations++;

                //PCK: unsigned instead of bool
                aabbOverlap = AabbUtil2.TestQuantizedAabbAgainstQuantizedAabb(ref quantizedQueryAabbMin, ref quantizedQueryAabbMax, ref rootNode.m_quantizedAabbMin, ref rootNode.m_quantizedAabbMax);
                isLeafNode = rootNode.IsLeafNode();

                if (isLeafNode && aabbOverlap)
                {
                    nodeCallback.ProcessNode(rootNode.GetPartId(), rootNode.GetTriangleIndex());
                }

                //PCK: unsigned instead of bool
                if ((aabbOverlap) || isLeafNode)
                {
                    curIndex++;
                }
                else
                {
                    escapeIndex = rootNode.GetEscapeIndex();
                    curIndex += escapeIndex;
                }
                rootNode = m_quantizedContiguousNodes[curIndex];

            }
            if (m_maxIterations < walkIterations)
            {
                m_maxIterations = walkIterations;
            }
        }

        protected void WalkStacklessTreeAgainstRay(INodeOverlapCallback nodeCallback, ref Vector3 raySource, ref Vector3 rayTarget, ref Vector3 aabbMin, ref Vector3 aabbMax, int startNodeIndex, int endNodeIndex)
        {
            Debug.Assert(!m_useQuantization);

            int escapeIndex, curIndex = 0;
            OptimizedBvhNode rootNode = m_contiguousNodes[curIndex];
            int walkIterations = 0;
            bool isLeafNode;

            bool aabbOverlap = false;
            bool rayBoxOverlap = false;
            float lambda_max = 1.0f;

            /* Quick pruning by quantized box */
            Vector3 rayAabbMin = raySource;
            Vector3 rayAabbMax = raySource;
            MathUtil.VectorMin(ref rayTarget, ref rayAabbMin);
            MathUtil.VectorMax(ref rayTarget, ref rayAabbMax);

            /* Add box cast extents to bounding box */
            rayAabbMin += aabbMin;
            rayAabbMax += aabbMax;

#if RAYAABB2
            Vector3 rayDir = (rayTarget - raySource);
            rayDir.Normalize();
            lambda_max = Vector3.Dot(rayDir, rayTarget - raySource);
            ///what about division by zero? --> just set rayDirection[i] to 1.0
            Vector3 rayDirectionInverse = new Vector3();
            rayDirectionInverse.X = MathUtil.FuzzyZero(rayDir.X) ? MathUtil.BT_LARGE_FLOAT : 1.0f / rayDir.X;
            rayDirectionInverse.Y = MathUtil.FuzzyZero(rayDir.Y) ? MathUtil.BT_LARGE_FLOAT : 1.0f / rayDir.Y;
            rayDirectionInverse.Z = MathUtil.FuzzyZero(rayDir.Z) ? MathUtil.BT_LARGE_FLOAT : 1.0f / rayDir.Z;
            bool[] sign = new bool[] { rayDirectionInverse.X < 0.0f, rayDirectionInverse.Y < 0.0f, rayDirectionInverse.Z < 0.0f };
#endif

            Vector3[] bounds = new Vector3[2];

            while (curIndex < m_curNodeIndex)
            {
                float param = 1.0f;
                //catch bugs in tree data
                Debug.Assert(walkIterations < m_curNodeIndex);

                walkIterations++;

                bounds[0] = rootNode.m_aabbMinOrg;
                bounds[1] = rootNode.m_aabbMaxOrg;
                /* Add box cast extents */
                bounds[0] -= aabbMax;
                bounds[1] -= aabbMin;

                aabbOverlap = AabbUtil2.TestAabbAgainstAabb2(ref rayAabbMin, ref rayAabbMax, ref rootNode.m_aabbMinOrg, ref rootNode.m_aabbMaxOrg);
                //perhaps profile if it is worth doing the aabbOverlap test first

#if RAYAABB2
                ///careful with this check: need to check division by zero (above) and fix the unQuantize method
                ///thanks Joerg/hiker for the reproduction case!
                ///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858
                rayBoxOverlap = aabbOverlap ? AabbUtil2.RayAabb2(ref raySource, ref rayDirectionInverse, sign, bounds, out param, 0.0f, lambda_max) : false;

#else
                Vector3 normal = Vector3.Zero;
                rayBoxOverlap = AabbUtil2.RayAabb(raySource, rayTarget, bounds[0], bounds[1], param, normal);
#endif

                isLeafNode = rootNode.m_escapeIndex == -1;

                //PCK: unsigned instead of bool
                if (isLeafNode && (rayBoxOverlap))
                {
                    nodeCallback.ProcessNode(rootNode.m_subPart, rootNode.m_triangleIndex);
                }

                //PCK: unsigned instead of bool
                if ((rayBoxOverlap) || isLeafNode)
                {
                    curIndex++;
                }
                else
                {
                    escapeIndex = rootNode.m_escapeIndex;
                    curIndex += escapeIndex;
                }
                rootNode = m_contiguousNodes[curIndex];
            }
            if (m_maxIterations < walkIterations)
            {
                m_maxIterations = walkIterations;
            }
        }

        ///tree traversal designed for small-memory processors like PS3 SPU
        protected void WalkStacklessQuantizedTreeCacheFriendly(INodeOverlapCallback nodeCallback, ref UShortVector3 quantizedQueryAabbMin, ref UShortVector3 quantizedQueryAabbMax)
        {
            Debug.Assert(m_useQuantization);

            for (int i = 0; i < m_SubtreeHeaders.Count; i++)
            {
                BvhSubtreeInfo subtree = m_SubtreeHeaders[i];

                //PCK: unsigned instead of bool
                bool overlap = AabbUtil2.TestQuantizedAabbAgainstQuantizedAabb(ref quantizedQueryAabbMin, ref quantizedQueryAabbMax, ref subtree.m_quantizedAabbMin, ref subtree.m_quantizedAabbMax);
                if (overlap)
                {
                    WalkStacklessQuantizedTree(nodeCallback, ref quantizedQueryAabbMin, ref quantizedQueryAabbMax,
                        subtree.m_rootNodeIndex,
                        subtree.m_rootNodeIndex + subtree.m_subtreeSize);
                }
            }

        }

        ///use the 16-byte stackless 'skipindex' node tree to do a recursive traversal
        protected void WalkRecursiveQuantizedTreeAgainstQueryAabb(ref QuantizedBvhNode currentNode, INodeOverlapCallback nodeCallback, ref UShortVector3 quantizedQueryAabbMin, ref UShortVector3 quantizedQueryAabbMax)
        {
            Debug.Assert(m_useQuantization);

            bool isLeafNode;
            //PCK: unsigned instead of bool
            bool aabbOverlap = false;

            //PCK: unsigned instead of bool
            aabbOverlap = AabbUtil2.TestQuantizedAabbAgainstQuantizedAabb(ref quantizedQueryAabbMin, ref quantizedQueryAabbMax, ref currentNode.m_quantizedAabbMin, ref currentNode.m_quantizedAabbMax);
            isLeafNode = currentNode.IsLeafNode();

            //PCK: unsigned instead of bool
            if (aabbOverlap)
            {
                if (isLeafNode)
                {
                    nodeCallback.ProcessNode(currentNode.GetPartId(), currentNode.GetTriangleIndex());
                }
                else
                {
                    //process left and right children

                    //QuantizedBvhNode leftChildNode = currentNode + 1;
                    // Not sure bout thie replacement... but avoids pointer arithmetic
                    // this is broken ...
                    int nodeIndex = currentNode.GetTriangleIndex() + 1;
                    QuantizedBvhNode leftChildNode = m_quantizedContiguousNodes[nodeIndex];

                    WalkRecursiveQuantizedTreeAgainstQueryAabb(ref leftChildNode, nodeCallback, ref quantizedQueryAabbMin, ref quantizedQueryAabbMax);

                    int newIndex = leftChildNode.IsLeafNode() ? leftChildNode.GetTriangleIndex() + 1 : leftChildNode.GetTriangleIndex() + leftChildNode.GetEscapeIndex();
                    QuantizedBvhNode rightChildNode = m_quantizedContiguousNodes[newIndex];

                    WalkRecursiveQuantizedTreeAgainstQueryAabb(ref rightChildNode, nodeCallback, ref quantizedQueryAabbMin, ref quantizedQueryAabbMax);
                }
            }
        }

        ///use the 16-byte stackless 'skipindex' node tree to do a recursive traversal
        protected void WalkRecursiveQuantizedTreeAgainstQuantizedTree(QuantizedBvhNode treeNodeA, QuantizedBvhNode treeNodeB, INodeOverlapCallback nodeCallback)
        {
            // MAN - No implementation??
        }

        protected void UpdateSubtreeHeaders(int leftChildNodexIndex, int rightChildNodexIndex)
        {
            Debug.Assert(m_useQuantization);

            QuantizedBvhNode leftChildNode = m_quantizedContiguousNodes[leftChildNodexIndex];
            int leftSubTreeSize = leftChildNode.IsLeafNode() ? 1 : leftChildNode.GetEscapeIndex();
            //int leftSubTreeSizeInBytes = leftSubTreeSize * static_cast<int>(sizeof(btQuantizedBvhNode));

            QuantizedBvhNode rightChildNode = m_quantizedContiguousNodes[rightChildNodexIndex];
            int rightSubTreeSize = rightChildNode.IsLeafNode() ? 1 : rightChildNode.GetEscapeIndex();
            //int rightSubTreeSizeInBytes = rightSubTreeSize * static_cast<int>(sizeof(btQuantizedBvhNode));

            if (leftChildNode.GetEscapeIndex() <= MAX_SUBTREE_SIZE_IN_BYTES)
            {
                BvhSubtreeInfo subtree = new BvhSubtreeInfo();
                m_SubtreeHeaders.Add(subtree);
                subtree.SetAabbFromQuantizeNode(leftChildNode);
                subtree.m_rootNodeIndex = leftChildNodexIndex;
                subtree.m_subtreeSize = leftSubTreeSize;
            }

            if (rightChildNode.GetEscapeIndex() <= MAX_SUBTREE_SIZE_IN_BYTES)
            {
                BvhSubtreeInfo subtree = new BvhSubtreeInfo();
                m_SubtreeHeaders.Add(subtree);
                subtree.SetAabbFromQuantizeNode(rightChildNode);
                subtree.m_rootNodeIndex = rightChildNodexIndex;
                subtree.m_subtreeSize = rightSubTreeSize;
            }

            //PCK: update the copy of the size
            m_subtreeHeaderCount = m_SubtreeHeaders.Count;
        }

        ///***************************************** expert/internal use only *************************
        ///
        public void SetQuantizationValues(ref Vector3 bvhAabbMin, ref Vector3 bvhAabbMax)
        {
            SetQuantizationValues(ref bvhAabbMin, ref bvhAabbMax, 1.0f);
        }

        public void SetQuantizationValues(ref Vector3 bvhAabbMin, ref Vector3 bvhAabbMax, float quantizationMargin)
        {
            //enlarge the AABB to avoid division by zero when initializing the quantization values
            Vector3 clampValue = new Vector3(quantizationMargin);
            m_bvhAabbMin = bvhAabbMin - clampValue;
            m_bvhAabbMax = bvhAabbMax + clampValue;
            Vector3 aabbSize = m_bvhAabbMax - m_bvhAabbMin;
            m_bvhQuantization = new Vector3(65533.000f) / aabbSize;
            m_useQuantization = true;
        }

        public ObjectArray<QuantizedBvhNode> GetLeafNodeArray()
        {
            return m_quantizedLeafNodes;
        }
        ///buildInternal is expert use only: assumes that setQuantizationValues and LeafNodeArray are initialized
        public void BuildInternal()
        {
            ///assumes that caller filled in the m_quantizedLeafNodes
            m_useQuantization = true;
            int numLeafNodes = 0;

            if (m_useQuantization)
            {
                //now we have an array of leafnodes in m_leafNodes
                numLeafNodes = m_quantizedLeafNodes.Count;
                m_quantizedContiguousNodes.Capacity = 2 * numLeafNodes;
            }

            m_curNodeIndex = 0;

            BuildTree(0, numLeafNodes);
            for (int i = 0; i < m_quantizedContiguousNodes.Count; ++i)
            {
                QuantizedBvhNode bvhn = m_quantizedContiguousNodes[i];
                System.Console.WriteLine(String.Format("QNode[{0}] Esc[{1}] min[{2},{3},{4}] max[{5},{6},{7}]", i, bvhn.m_escapeIndexOrTriangleIndex, bvhn.m_quantizedAabbMin.X, bvhn.m_quantizedAabbMin.Y, bvhn.m_quantizedAabbMin.Z, bvhn.m_quantizedAabbMax.X, bvhn.m_quantizedAabbMax.Y, bvhn.m_quantizedAabbMax.Z));
            }

            ///if the entire tree is small then subtree size, we need to create a header info for the tree
            if (m_useQuantization && m_SubtreeHeaders.Count == 0)
            {
                //BvhSubtrreeInfo subtree = m_SubtreeHeaders.expand();
                BvhSubtreeInfo subtree = new BvhSubtreeInfo();
                m_SubtreeHeaders.Add(subtree);
                subtree.SetAabbFromQuantizeNode(m_quantizedContiguousNodes[0]);
                subtree.m_rootNodeIndex = 0;
                subtree.m_subtreeSize = m_quantizedContiguousNodes[0].IsLeafNode() ? 1 : m_quantizedContiguousNodes[0].GetEscapeIndex();
            }

            //PCK: update the copy of the size
            m_subtreeHeaderCount = m_SubtreeHeaders.Count;

            //PCK: clear m_quantizedLeafNodes and m_leafNodes, they are temporary
            m_quantizedLeafNodes.Clear();
            m_leafNodes.Clear();

        }
        ///***************************************** expert/internal use only *************************

        public void ReportAabbOverlappingNodex(INodeOverlapCallback nodeCallback, ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
            //either choose recursive traversal (walkTree) or stackless (walkStacklessTree)

            if (m_useQuantization)
            {
                ///quantize query AABB
                UShortVector3 quantizedQueryAabbMin;
                UShortVector3 quantizedQueryAabbMax;
                QuantizeWithClamp(out quantizedQueryAabbMin, ref aabbMin, false);
                QuantizeWithClamp(out quantizedQueryAabbMax, ref aabbMax, true);

                switch (m_traversalMode)
                {
                    case TraversalMode.TRAVERSAL_STACKLESS:
                        if (m_useQuantization)
                        {
                            WalkStacklessQuantizedTree(nodeCallback, ref quantizedQueryAabbMin, ref quantizedQueryAabbMax, 0, m_curNodeIndex);
                        }
                        else
                        {
                            WalkStacklessTree(nodeCallback, ref aabbMin, ref aabbMax);
                        }
                        break;
                    case TraversalMode.TRAVERSAL_STACKLESS_CACHE_FRIENDLY:
                        WalkStacklessQuantizedTreeCacheFriendly(nodeCallback, ref quantizedQueryAabbMin, ref quantizedQueryAabbMax);
                        break;
                    case TraversalMode.TRAVERSAL_RECURSIVE:
                        {
                            QuantizedBvhNode rootNode = m_quantizedContiguousNodes[0];
                            WalkRecursiveQuantizedTreeAgainstQueryAabb(ref rootNode, nodeCallback, ref quantizedQueryAabbMin, ref quantizedQueryAabbMax);
                        }
                        break;
                    default:
                        //unsupported
                        Debug.Assert(false);
                        break;
                }
            }
            else
            {
                WalkStacklessTree(nodeCallback, ref aabbMin, ref aabbMax);
            }
        }

        public void ReportRayOverlappingNodex(INodeOverlapCallback nodeCallback, ref Vector3 raySource, ref Vector3 rayTarget)
        {
            Vector3 abMin = Vector3.Zero, abMax = Vector3.Zero;
            ReportBoxCastOverlappingNodex(nodeCallback, ref raySource, ref rayTarget, ref abMin, ref abMax);
        }

        public void ReportBoxCastOverlappingNodex(INodeOverlapCallback nodeCallback, ref Vector3 raySource, ref Vector3 rayTarget, ref Vector3 aabbMin, ref Vector3 aabbMax)
        {
            //always use stackless

            if (m_useQuantization)
            {
                WalkStacklessQuantizedTreeAgainstRay(nodeCallback, ref raySource, ref rayTarget, ref aabbMin, ref aabbMax, 0, m_curNodeIndex);
            }
            else
            {
                WalkStacklessTreeAgainstRay(nodeCallback, ref raySource, ref rayTarget, ref aabbMin, ref aabbMax, 0, m_curNodeIndex);
            }
            /*
            {
                //recursive traversal
                Vector3 qaabbMin = raySource;
                Vector3 qaabbMax = raySource;
                qaabbMin.setMin(rayTarget);
                qaabbMax.setMax(rayTarget);
                qaabbMin += aabbMin;
                qaabbMax += aabbMax;
                reportAabbOverlappingNodex(nodeCallback,qaabbMin,qaabbMax);
            }
            */

        }

        public void Quantize(out UShortVector3 result, ref Vector3 point, bool isMax)
        {
            Debug.Assert(m_useQuantization);

            Debug.Assert(point.X <= m_bvhAabbMax.X);
            Debug.Assert(point.Y <= m_bvhAabbMax.Y);
            Debug.Assert(point.Z <= m_bvhAabbMax.Z);

            Debug.Assert(point.X >= m_bvhAabbMin.X);
            Debug.Assert(point.Y >= m_bvhAabbMin.Y);
            Debug.Assert(point.Z >= m_bvhAabbMin.Z);

            Vector3 v = (point - m_bvhAabbMin) * m_bvhQuantization;
            ///Make sure rounding is done in a way that unQuantize(quantizeWithClamp(...)) is conservative
            ///end-points always set the first bit, so that they are sorted properly (so that neighbouring AABBs overlap properly)
            ///@todo: double-check this

            result = new UShortVector3();
            if (isMax)
            {
                result.X = (ushort)(((ushort)(v.X + 1f) | 1));
                result.Y = (ushort)(((ushort)(v.Y + 1f) | 1));
                result.Z = (ushort)(((ushort)(v.Z + 1f) | 1));
            }
            else
            {
                result.X = (ushort)(((ushort)(v.X) & 0xfffe));
                result.Y = (ushort)(((ushort)(v.Y) & 0xfffe));
                result.Z = (ushort)(((ushort)(v.Z) & 0xfffe));
            }


#if DEBUG_CHECK_DEQUANTIZATION
            Vector3 newPoint;
            UnQuantize(ref result, out newPoint);
            if (isMax)
            {
                if (newPoint.X < point.X)
                {
                    System.Console.WriteLine("unconservative X, diffX = {0}, oldX={1},newX={2}\n", newPoint.X - point.X, newPoint.X, point.X);
                }
                if (newPoint.Y < point.Y)
                {
                    System.Console.WriteLine("unconservative Y, diffY = {0}, oldY={1},newY={2}\n", newPoint.Y - point.Y, newPoint.Y, point.Y);
                }
                if (newPoint.Z < point.Z)
                {
                    System.Console.WriteLine("unconservative Z, diffZ = {0}, oldZ={1},newZ={2}\n", newPoint.Z - point.Z, newPoint.Z, point.Z);
                }
            }
            else
            {
                if (newPoint.X > point.X)
                {
                    System.Console.WriteLine("unconservative X, diffX = {0}, oldX={1},newX={2}\n", newPoint.X - point.X, newPoint.X, point.X);
                }
                if (newPoint.Y > point.Y)
                {
                    System.Console.WriteLine("unconservative Y, diffY = {0}, oldY={1},newY={2}\n", newPoint.Y - point.Y, newPoint.Y, point.Y);
                }
                if (newPoint.Z > point.Z)
                {
                    System.Console.WriteLine("unconservative Z, diffZ = {0}, oldZ={1},newZ={2}\n", newPoint.Z - point.Z, newPoint.Z, point.Z);
                }
            }
#endif //DEBUG_CHECK_DEQUANTIZATION

        }

        //public void quantizeWithClamp(ref UShortVector3 result, ref UShortVector3 point2, bool isMax)
        //{

        //    Debug.Assert(m_useQuantization);

        //    Vector3 clampedPoint = new Vector3(point2.X, point2.Y, point2.Z);
        //    MathUtil.vectorMax(ref m_bvhAabbMin, ref clampedPoint);
        //    MathUtil.vectorMin(ref m_bvhAabbMax, ref clampedPoint);

        //    quantize(ref result, ref clampedPoint, isMax);
        //}

        public void QuantizeWithClamp(out UShortVector3 result, ref Vector3 point2, bool isMax)
        {

            Debug.Assert(m_useQuantization);

            Vector3 clampedPoint = point2;
            MathUtil.VectorMax(ref m_bvhAabbMin, ref clampedPoint);
            MathUtil.VectorMin(ref m_bvhAabbMax, ref clampedPoint);

            Quantize(out result, ref clampedPoint, isMax);
        }



        //public void UnQuantize(UShortVector3 vecIn,out Vector3 vecOut)
        //{
        //    UnQuantize(ref vecIn,out vecOut);
        //}

        public void UnQuantize(ref UShortVector3 vecIn, out Vector3 vecOut)
        {
            vecOut = new Vector3(((float)vecIn.X) / m_bvhQuantization.X,
                                        ((float)vecIn.Y) / m_bvhQuantization.Y,
                                        ((float)vecIn.Z) / m_bvhQuantization.Z);
            vecOut += m_bvhAabbMin;
        }

        ///setTraversalMode let's you choose between stackless, recursive or stackless cache friendly tree traversal. Note this is only implemented for quantized trees.
        void SetTraversalMode(TraversalMode traversalMode)
        {
            m_traversalMode = traversalMode;
        }


        public ObjectArray<QuantizedBvhNode> GetQuantizedNodeArray()
        {
            return m_quantizedContiguousNodes;
        }


        public ObjectArray<BvhSubtreeInfo> GetSubtreeInfoArray()
        {
            return m_SubtreeHeaders;
        }

        public bool IsQuantized()
        {
            return m_useQuantization;
        }
    }
}
