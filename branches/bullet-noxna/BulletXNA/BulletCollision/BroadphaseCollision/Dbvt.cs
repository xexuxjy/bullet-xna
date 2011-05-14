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
using System.Collections.Generic;
using System.Text;
using Microsoft.Xna.Framework;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision.BroadphaseCollision
{
    public class Dbvt
    {
        public static void SetMin(ref Vector3 a, ref Vector3 b)
        {
            a.X = Math.Min(a.X, b.X);
            a.Y = Math.Min(a.Y, b.Y);
            a.Z = Math.Min(a.Z, b.Z);
        }

        public static void SetMax(ref Vector3 a, ref Vector3 b)
        {
            a.X = Math.Max(a.X, b.X);
            a.Y = Math.Max(a.Y, b.Y);
            a.Z = Math.Max(a.Z, b.Z);
        }

        ///* Collide	*/
        //public interface Collide : IDisposable
        //{
        //    void Process(DbvtNode a, DbvtNode b);
        //    void Process(DbvtNode a);
        //    bool Descent(DbvtNode a);
        //    bool AllLeafs(DbvtNode a);
        //};

        public Dbvt()
        {
            m_lkhd = -1;
        }



        public DbvtNode Root
        {
            get { return m_root; }
            set { m_root = value; }
        }



        //public static bool Intersect(ref DbvtAabbMm a, ref DbvtAabbMm b)
        //{
        //    Vector3 amin = a.Mins();
        //    Vector3 amax = a.Maxs();
        //    Vector3 bmin = b.Mins();
        //    Vector3 bmax = b.Maxs();
        //    return ((amin.X <= bmin.X) &&
        //        (amin.Y <= bmax.Y) &&
        //        (amin.Z <= bmax.Z) &&
        //        (amax.X >= bmin.X) &&
        //        (amax.Y >= bmin.Y) &&
        //        (amax.Z >= bmin.Z));
        //}



        //public static bool Intersect(ref DbvtAabbMm a, ref Vector3 b)
        //{
        //    Vector3 amin = a.Mins();
        //    Vector3 amax = a.Maxs();

        //    return ((b.X >= amin.X) &&
        //            (b.Y >= amin.Y) &&
        //            (b.Z >= amin.Z) &&
        //            (b.X <= amax.X) &&
        //            (b.Y <= amax.Y) &&
        //            (b.Z <= amax.Z));
        //}



        public static bool Intersect(ref DbvtAabbMm a, ref Vector3 org, ref Vector3 invdir, int[] signs)
        {
            Vector3 amin = a.Mins();
            Vector3 amax = a.Maxs();

            Vector3[] bounds = new Vector3[] { amin, amax };
            float txmin = (bounds[signs[0]].X - org.X) * invdir.X;
            float txmax = (bounds[1 - signs[0]].X - org.X) * invdir.X;
            float tymin = (bounds[signs[1]].Y - org.Y) * invdir.Y;
            float tymax = (bounds[1 - signs[1]].Y - org.Y) * invdir.Y;
            if ((txmin > tymax) || (tymin > txmax)) return (false);
            if (tymin > txmin) txmin = tymin;
            if (tymax < txmax) txmax = tymax;
            float tzmin = (bounds[signs[2]].Z - org.Z) * invdir.Z;
            float tzmax = (bounds[1 - signs[2]].Z - org.Z) * invdir.Z;
            if ((txmin > tzmax) || (tzmin > txmax)) return (false);
            if (tzmin > txmin) txmin = tzmin;
            if (tzmax < txmax) txmax = tzmax;
            return (txmax > 0);
        }



        public void Clear()
        {
        }



        public bool Empty()
        {
            return Root == null;
        }



        public void OptimizeBottomUp()
        {
            if (Root != null)
            {
                List<DbvtNode> leafs = new List<DbvtNode>(m_leaves);
                FetchLeafs(this, Root, leafs);
                BottomUp(this, leafs);
                Root = leafs[0];
            }
        }


        public void OptimizeTopDown()
        {
            OptimizeTopDown(128);
        }

        public void OptimizeTopDown(int bu_threshold)
        {
            // threshhold defaults to 128
            if (Root != null)
            {
                List<DbvtNode> leafs = new List<DbvtNode>(m_leaves);
                FetchLeafs(this, Root, leafs);
                Root = TopDown(this, leafs, bu_threshold);
            }
        }


        public virtual void Cleanup()
        {
            Clear();
        }

        public void OptimizeIncremental(int passes)
        {
            if (Root != null && (passes > 0))
            {
                int sizeOfUnsigned = 4;
                int computedValue = (sizeOfUnsigned * 8 - 1);
                do
                {
                    DbvtNode node = Root;
                    int bit = 0;
                    while (node.IsInternal())
                    {
                        node = node._children[(int)((m_opath >> bit) & 1)];
                        bit = (bit + 1) & computedValue;
                    }
                    Update(node);
                    ++m_opath;
                } while (--passes > 0);
            }
        }



        public DbvtNode Insert(ref DbvtAabbMm box, Object data)
        {
            DbvtNode leaf = new DbvtNode(this, null, ref box, data);
            InsertLeaf(this, Root, leaf);
            ++m_leaves;
            return leaf;
        }


        public void Update(DbvtNode leaf)
        {
            Update(leaf, -1);
        }

        public void Update(DbvtNode leaf, int lookahead)
        {
            DbvtNode root = RemoveLeaf(this, leaf);
            if (root != null)
            {
                if (lookahead >= 0)
                {
                    for (int i = 0; (i < lookahead) && (root.parent != null); ++i)
                    {
                        root = root.parent;
                    }
                }
                else
                {
                    root = Root;
                }
            }
            InsertLeaf(this, root, leaf);
        }



        public void Update(DbvtNode leaf, ref DbvtAabbMm volume)
        {
            DbvtNode root = RemoveLeaf(this, leaf);
            if (root != null)
            {
                if (m_lkhd >= 0)
                {
                    for (int i = 0; (i < m_lkhd) && (root.parent != null); ++i)
                    {
                        root = root.parent;
                    }
                }
                else
                {
                    root = Root;
                }
            }
            leaf.volume = volume;
            InsertLeaf(this, root, leaf);
        }



        public bool Update(DbvtNode leaf, ref DbvtAabbMm volume, ref Vector3 velocity, float margin)
        {
            if (leaf.volume.Contain(ref volume))
            {
                return (false);
            }
            volume.Expand(new Vector3(margin, margin, margin));
            volume.SignedExpand(velocity);
            Update(leaf, ref volume);
            return (true);
        }



        public bool Update(DbvtNode leaf, ref DbvtAabbMm volume, ref Vector3 velocity)
        {
            if (leaf.volume.Contain(ref volume))
            {
                return (false);
            }
            volume.SignedExpand(velocity);
            Update(leaf, ref volume);
            return (true);
        }



        public bool Update(DbvtNode leaf, ref DbvtAabbMm volume, float margin)
        {
            if (leaf.volume.Contain(ref volume))
            {
                return (false);
            }
            volume.Expand(new Vector3(margin, margin, margin));
            Update(leaf, ref volume);
            return (true);
        }



        public void Remove(DbvtNode leaf)
        {
            RemoveLeaf(this, leaf);
            DeleteNode(this, ref leaf);
            --m_leaves;
        }


        //
        // depth is defaulted to -1
        public static void FetchLeafs(Dbvt pdbvt, DbvtNode root, List<DbvtNode> leafs)
        {
            FetchLeafs(pdbvt, root, leafs, -1);
        }
        public static void FetchLeafs(Dbvt pdbvt, DbvtNode root, List<DbvtNode> leafs, int depth)
        {
            if (root.IsInternal() && depth != 0)
            {
                FetchLeafs(pdbvt, root._children[0], leafs, depth - 1);
                FetchLeafs(pdbvt, root._children[1], leafs, depth - 1);
                DeleteNode(pdbvt, ref root);
            }
            else
            {
                leafs.Add(root);
            }
        }



        public static void Split(List<DbvtNode> leafs, List<DbvtNode> left, List<DbvtNode> right, ref Vector3 org, ref Vector3 axis)
        {
            left.Clear();
            right.Clear();
            for (int i = 0, ni = leafs.Count; i < ni; ++i)
            {
                if (Vector3.Dot(axis, leafs[i].volume.Center() - org) < 0)
                {
                    left.Add(leafs[i]);
                }
                else
                {
                    right.Add(leafs[i]);
                }
            }
        }




        public static void EnumNodes(DbvtNode root, Collide collideable)
        {
            collideable.Process(root);
            if (root.IsInternal())
            {
                EnumNodes(root._children[0], collideable);
                EnumNodes(root._children[1], collideable);
            }
        }



        public static void EnumLeafs(DbvtNode root, Collide collideable)
        {
            if (root.IsInternal())
            {
                EnumLeafs(root._children[0], collideable);
                EnumLeafs(root._children[1], collideable);
            }
            else
            {
                collideable.Process(root);
            }
        }



        public static void CollideTTpersistentStack(DbvtNode m_root0, DbvtNode m_root1, Collide collider)
        {
            CollideTT(m_root0, m_root1, collider);
        }

        public static void CollideTT(DbvtNode root0, DbvtNode root1, Collide collideable)
        {
            if (root0 != null && root1 != null)
            {
                Stack<sStkNN> stack = new Stack<sStkNN>(DOUBLE_STACKSIZE);
                stack.Push(new sStkNN(root0, root1));
                do
                {
                    sStkNN p = stack.Pop();
                    if (p.a == p.b)
                    {
                        if (p.a.IsInternal())
                        {
                            stack.Push(new sStkNN(p.a._children[0], p.a._children[0]));
                            stack.Push(new sStkNN(p.a._children[1], p.a._children[1]));
                            stack.Push(new sStkNN(p.a._children[0], p.a._children[1]));
                        }
                    }
                    else if (DbvtAabbMm.Intersect(ref p.a.volume, ref p.b.volume))
                    {
                        if (p.a.IsInternal())
                        {
                            if (p.b.IsInternal())
                            {
                                stack.Push(new sStkNN(p.a._children[0], p.b._children[0]));
                                stack.Push(new sStkNN(p.a._children[1], p.b._children[0]));
                                stack.Push(new sStkNN(p.a._children[0], p.b._children[1]));
                                stack.Push(new sStkNN(p.a._children[1], p.b._children[1]));
                            }
                            else
                            {
                                stack.Push(new sStkNN(p.a._children[0], p.b));
                                stack.Push(new sStkNN(p.a._children[1], p.b));
                            }
                        }
                        else
                        {
                            if (p.b.IsInternal())
                            {
                                stack.Push(new sStkNN(p.a, p.b._children[0]));
                                stack.Push(new sStkNN(p.a, p.b._children[1]));
                            }
                            else
                            {
                                collideable.Process(p.a, p.b);
                            }
                        }
                    }
                } while (stack.Count > 0);
            }
        }


        public void RayTestInternal(DbvtNode root,
                                    ref Vector3 rayFrom,
                                    ref Vector3 rayTo,
                                    ref Vector3 rayDirectionInverse,
                                    bool[] signs,
                                    float lambda_max,
                                    ref Vector3 aabbMin,
                                    ref Vector3 aabbMax,
                                    Collide policy)
        {
            //    (void) rayTo;
            //DBVT_CHECKTYPE
            if (root != null)
            {
                Vector3 resultNormal = Vector3.Up;

                int depth = 1;
                int treshold = DOUBLE_STACKSIZE - 2;
                ObjectArray<DbvtNode> stack = new ObjectArray<DbvtNode>(DOUBLE_STACKSIZE);
                stack[0] = root;
                Vector3[] bounds = new Vector3[2];
                do
                {
                    DbvtNode node = stack[--depth];
                    bounds[0] = node.volume.Mins() - aabbMax;
                    bounds[1] = node.volume.Maxs() - aabbMin;
                    float tmin, lambda_min = 0.0f;
                    bool result1 = AabbUtil2.RayAabb2(ref rayFrom, ref rayDirectionInverse, signs, bounds, out tmin, lambda_min, lambda_max);
                    if (result1)
                    {
                        if (node.IsInternal())
                        {
                            //if(depth>treshold)
                            //{
                            //    stack.resize(stack.size()*2);
                            //    treshold=stack.size()-2;
                            //}
                            stack[depth++] = node._children[0];
                            stack[depth++] = node._children[1];
                        }
                        else
                        {
                            policy.Process(node);
                        }
                    }
                } while (depth != 0);
            }
        }



        public static void CollideTV(DbvtNode root, ref DbvtAabbMm volume, Collide collideable)
        {
            if (root != null)
            {
                Stack<DbvtNode> stack = new Stack<DbvtNode>(SIMPLE_STACKSIZE);
                stack.Push(root);
                do
                {
                    DbvtNode n = stack.Pop();
                    if (DbvtAabbMm.Intersect(ref n.volume, ref volume))
                    {
                        if (n.IsInternal())
                        {
                            stack.Push(n._children[0]);
                            stack.Push(n._children[1]);
                        }
                        else
                        {
                            collideable.Process(n);
                        }
                    }
                } while (stack.Count > 0);
            }
        }



        public static void CollideRAY(DbvtNode root, ref Vector3 origin, ref Vector3 direction, Collide collideable)
        {
            if (root != null)
            {
                Vector3 normal = direction;
                normal.Normalize();
                Vector3 invdir = new Vector3(1 / normal.X, 1 / normal.Y, 1 / normal.Z);
                int[] signs = new int[] { direction.X < 0f ? 1 : 0, direction.Y < 0f ? 1 : 0, direction.Z < 0f ? 1 : 0 };
                Stack<DbvtNode> stack = new Stack<DbvtNode>(SIMPLE_STACKSIZE);
                stack.Push(root);
                do
                {
                    DbvtNode node = stack.Pop();
                    if (Intersect(ref node.volume, ref origin, ref invdir, signs))
                    {
                        if (node.IsInternal())
                        {
                            stack.Push(node._children[0]);
                            stack.Push(node._children[1]);
                        }
                        else
                        {
                            collideable.Process(node);
                        }
                    }
                } while (stack.Count > 0);
            }
        }



        public static void CollideKDOP(DbvtNode root, Vector3[] normals, float[] offsets, int count, Collide collideable)
        {
            if (root != null)
            {
                int inside = (1 << count) - 1;
                Stack<sStkNP> stack = new Stack<sStkNP>(SIMPLE_STACKSIZE);
                int[] signs = new int[count];

                for (int i = 0; i < count; ++i)
                {
                    signs[i] = ((normals[i].X >= 0) ? 1 : 0) +
                                ((normals[i].Y >= 0) ? 2 : 0) +
                                ((normals[i].Z >= 0) ? 4 : 0);
                }
                stack.Push(new sStkNP(root, 0));
                do
                {
                    sStkNP se = stack.Pop();
                    bool outp = false;
                    for (int i = 0, j = 1; (!outp) && (i < count); ++i, j <<= 1)
                    {
                        if (0 == (se.mask & j))
                        {
                            int side = se.node.volume.Classify(ref normals[i], offsets[i], signs[i]);
                            switch (side)
                            {
                                case -1: outp = true; break;
                                case +1: se.mask |= (uint)j; break;
                            }
                        }
                    }
                    if (!outp)
                    {
                        if ((se.mask != inside) && (se.node.IsInternal()))
                        {
                            stack.Push(new sStkNP(se.node._children[0], se.mask));
                            stack.Push(new sStkNP(se.node._children[1], se.mask));
                        }
                        else
                        {
                            if (collideable.AllLeaves(se.node))
                            {
                                EnumLeafs(se.node, collideable);
                            }
                        }
                    }
                } while (stack.Count > 0);
            }
        }



        public static void CollideOCL(DbvtNode root, Vector3[] normals, float[] offsets, ref Vector3 sortaxis, int count, Collide collideable)
        {
            if (root != null)
            {
                uint srtsgns = (uint)((sortaxis.X >= 0 ? 1 : 0) + (sortaxis.Y >= 0 ? 2 : 0) + (sortaxis.Z >= 0 ? 4 : 0));
                int inside = (1 << count) - 1;
                //Stack<sStkNPS>	stack = new Stack<sStkNPS>(SIMPLE_STACKSIZE);
                List<sStkNPS> stack = new List<sStkNPS>(SIMPLE_STACKSIZE);
                int[] signs = new int[count];

                for (int i = 0; i < count; ++i)
                {
                    signs[i] = ((normals[i].X >= 0) ? 1 : 0) +
                             ((normals[i].Y >= 0) ? 2 : 0) +
                             ((normals[i].Z >= 0) ? 4 : 0);
                }
                stack.Insert(0, new sStkNPS(root, 0, root.volume.ProjectMinimum(ref sortaxis, srtsgns)));
                do
                {
                    sStkNPS se = stack[0];
                    stack.RemoveAt(0);
                    if (se.mask != inside)
                    {
                        bool outp = false;
                        for (int i = 0, j = 1; (!outp) && (i < count); ++i, j <<= 1)
                        {
                            if (0 == (se.mask & j))
                            {
                                int side = se.node.volume.Classify(ref normals[i], offsets[i], signs[i]);
                                switch (side)
                                {
                                    case -1: outp = true; break;
                                    case +1: se.mask |= (uint)j; break;
                                }
                            }
                        }
                        if (outp)
                        {
                            continue;
                        }
                    }
                    if (collideable.Descent(se.node))
                    {
                        if (se.node.IsInternal())
                        {
                            for (int i = 0; i < 2; ++i)
                            {
                                DbvtNode n = se.node._children[i];
                                int j = stack.Count;
                                sStkNPS ne = new sStkNPS(n, se.mask, n.volume.ProjectMinimum(ref sortaxis, srtsgns));
                                stack.Insert(0, ne);
                                while ((j > 0) && (ne.value > stack[j - 1].value))
                                {
                                    sStkNPS left = stack[j];
                                    sStkNPS right = stack[j - 1];
                                    stack[j] = right;
                                    stack[j - 1] = left;
                                    --j;
                                    //btSwap(stack[j],stack[j-1]);--j;
                                }
                            }
                        }
                        else
                        {
                            collideable.Process(se.node);
                        }
                    }
                } while (stack.Count > 0);
            }
        }


        public static void CollideTU(DbvtNode root, Collide collideable)
        {
            if (root != null)
            {
                Stack<DbvtNode> stack = new Stack<DbvtNode>(SIMPLE_STACKSIZE);
                stack.Push(root);
                do
                {
                    DbvtNode n = stack.Pop();

                    if (collideable.Descent(n))
                    {
                        if (n.IsInternal())
                        {
                            stack.Push(n._children[0]);
                            stack.Push(n._children[1]);
                        }
                        else
                        {
                            collideable.Process(n);
                        }
                    }
                } while (stack.Count > 0);
            }
        }



        //
        public static DbvtAabbMm Bounds(List<DbvtNode> leafs)
        {
            DbvtAabbMm volume = leafs[0].volume;
            for (int i = 1, ni = leafs.Count; i < ni; ++i)
            {
                volume = DbvtAabbMm.Merge(ref volume, ref leafs[i].volume);
            }
            return (volume);
        }


        //
        public static void BottomUp(Dbvt pdbvt, List<DbvtNode> leafs)
        {
            while (leafs.Count > 1)
            {
                float minsize = float.MaxValue;
                int[] minidx = { -1, -1 };
                for (int i = 0; i < leafs.Count; ++i)
                {
                    for (int j = i + 1; j < leafs.Count; ++j)
                    {
                        DbvtAabbMm mergeResults = DbvtAabbMm.Merge(ref leafs[i].volume, ref leafs[j].volume);
                        float sz = Size(ref mergeResults);
                        if (sz < minsize)
                        {
                            minsize = sz;
                            minidx[0] = i;
                            minidx[1] = j;
                        }
                    }
                }
                DbvtNode[] n = { leafs[minidx[0]], leafs[minidx[1]] };
                DbvtAabbMm mergeResults2 = DbvtAabbMm.Merge(ref n[0].volume, ref n[1].volume);
                DbvtNode p = new DbvtNode(pdbvt, null, ref mergeResults2, null);
                p._children[0] = n[0];
                p._children[1] = n[1];
                n[0].parent = p;
                n[1].parent = p;
                leafs[minidx[0]] = p;

                DbvtNode left = leafs[minidx[1]];
                DbvtNode right = leafs[leafs.Count - 1];
                leafs[minidx[1]] = right;
                leafs[leafs.Count - 1] = left;
                leafs.RemoveAt(leafs.Count - 1);
            }
        }


        //

        public static Vector3[] axis = { new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1) };

        public static DbvtNode TopDown(Dbvt pdbvt, List<DbvtNode> leafs, int bu_treshold)
        {
            if (leafs.Count > 1)
            {
                if (leafs.Count > bu_treshold)
                {
                    DbvtAabbMm volume = Bounds(leafs);
                    Vector3 org = volume.Center();
                    List<DbvtNode>[] sets = { new List<DbvtNode>(), new List<DbvtNode>() };
                    int bestaxis = -1;
                    int bestmidp = leafs.Count;
                    int[] a1 = new int[] { 0, 0 };
                    int[] a2 = new int[] { 0, 0 };
                    int[] a3 = new int[] { 0, 0 };

                    int[][] splitcount = new int[][] { a1, a2, a3 };
                    for (int i = 0; i < leafs.Count; ++i)
                    {
                        Vector3 x = leafs[i].volume.Center() - org;
                        for (int j = 0; j < 3; ++j)
                        {
                            ++splitcount[j][Vector3.Dot(x, axis[j]) > 0 ? 1 : 0];
                        }
                    }
                    for (int i = 0; i < 3; ++i)
                    {
                        if ((splitcount[i][0] > 0) && (splitcount[i][1] > 0))
                        {
                            int midp = (int)Math.Abs((splitcount[i][0] - splitcount[i][1]));
                            if (midp < bestmidp)
                            {
                                bestaxis = i;
                                bestmidp = midp;
                            }
                        }
                    }
                    if (bestaxis >= 0)
                    {
                        sets[0].Capacity = (splitcount[bestaxis][0]);
                        sets[1].Capacity = (splitcount[bestaxis][1]);
                        Split(leafs, sets[0], sets[1], ref org, ref axis[bestaxis]);
                    }
                    else
                    {
                        sets[0].Capacity = (leafs.Count / 2 + 1);
                        sets[1].Capacity = (leafs.Count / 2);
                        for (int i = 0, ni = leafs.Count; i < ni; ++i)
                        {
                            sets[i & 1].Add(leafs[i]);
                        }
                    }
                    DbvtNode node = new DbvtNode(pdbvt, null, ref volume, null);
                    node._children[0] = TopDown(pdbvt, sets[0], bu_treshold);
                    node._children[1] = TopDown(pdbvt, sets[1], bu_treshold);
                    node._children[0].parent = node;
                    node._children[1].parent = node;
                    return (node);
                }
                else
                {
                    BottomUp(pdbvt, leafs);
                    return (leafs[0]);
                }
            }
            return (leafs[0]);
        }



        public static void InsertLeaf(Dbvt pdbvt, DbvtNode root, DbvtNode leaf)
        {
            if (pdbvt.Root == null)
            {
                pdbvt.Root = leaf;
                leaf.parent = null;
            }
            else
            {
                if (!root.IsLeaf())
                {
                    do
                    {
                        if (DbvtAabbMm.Proximity(ref root._children[0].volume, ref leaf.volume) <
                            DbvtAabbMm.Proximity(ref root._children[1].volume, ref leaf.volume))
                        {
                            root = root._children[0];
                        }
                        else
                        {
                            root = root._children[1];
                        }
                    } while (!root.IsLeaf());
                }
                DbvtNode prev = root.parent;
                DbvtAabbMm mergeResults = DbvtAabbMm.Merge(ref leaf.volume, ref root.volume);
                DbvtNode node = new DbvtNode(pdbvt, prev, ref mergeResults, null);
                if (prev != null)
                {
                    prev._children[IndexOf(root)] = node;
                    node._children[0] = root;
                    root.parent = node;
                    node._children[1] = leaf;
                    leaf.parent = node;
                    do
                    {
                        if (!prev.volume.Contain(ref node.volume))
                        {
                            DbvtAabbMm.Merge(ref prev._children[0].volume, ref prev._children[1].volume, ref prev.volume);
                        }
                        else
                        {
                            break;
                        }
                        node = prev;
                    } while (null != (prev = node.parent));
                }
                else
                {
                    node._children[0] = root;
                    root.parent = node;
                    node._children[1] = leaf;
                    leaf.parent = node;
                    pdbvt.Root = node;
                }
            }
        }



        public static DbvtNode RemoveLeaf(Dbvt pdbvt, DbvtNode leaf)
        {
            if (leaf == pdbvt.Root)
            {
                pdbvt.Root = null;
                return null;
            }
            else
            {
                DbvtNode parent = leaf.parent;
                DbvtNode prev = parent.parent;
                DbvtNode sibling = parent._children[1 - IndexOf(leaf)];
                if (prev != null)
                {
                    prev._children[IndexOf(parent)] = sibling;
                    sibling.parent = prev;
                    DeleteNode(pdbvt, ref parent);
                    while (prev != null)
                    {
                        DbvtAabbMm pb = prev.volume;
                        DbvtAabbMm.Merge(ref prev._children[0].volume, ref prev._children[1].volume, ref prev.volume);
                        if (DbvtAabbMm.NotEqual(ref pb, ref prev.volume))
                        {
                            sibling = prev;
                            prev = prev.parent;
                        }
                        else
                        {
                            break;
                        }
                    }
                    return (prev != null ? prev : pdbvt.Root);
                }
                else
                {
                    pdbvt.Root = sibling;
                    sibling.parent = null;
                    DeleteNode(pdbvt, ref parent);
                    return (pdbvt.Root);
                }
            }
        }




        public static void DeleteNode(Dbvt dbvt, ref DbvtNode node)
        {
            node = null;
        }



        public static int IndexOf(DbvtNode node)
        {
            return (node.parent._children[1] == node) ? 1 : 0;
        }



        // volume+edge lengths
        public static float Size(ref DbvtAabbMm a)
        {
            Vector3 edges = a.Lengths();
            return (edges.X * edges.Y * edges.Z +
                edges.X + edges.Y + edges.Z);
        }



        public static int SIMPLE_STACKSIZE = 64;
        public static int DOUBLE_STACKSIZE = SIMPLE_STACKSIZE * 2;

        public DbvtNode m_root;
        // Disabled free list for now until I see how bad memory allocs are.
        //public Node m_free; 
        public int m_lkhd;
        public int m_leaves;
        public uint m_opath;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    public class DbvtNode
    {
        public DbvtNode()
        {
        }
        public DbvtNode(Dbvt tree, DbvtNode aparent, ref DbvtAabbMm avolume, Object adata)
        {
            volume = avolume;
            parent = aparent;
            data = adata;
            if (data is int)
            {
                dataAsInt = (int)data;
            }
        }
        public DbvtAabbMm volume;
        public DbvtNode parent;
        public DbvtNode[] _children = new DbvtNode[2];
        public Object data;
        public int dataAsInt;

        public bool IsLeaf() { return (_children[1] == null); }
        public bool IsInternal() { return (!IsLeaf()); }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    public struct sStkNN
    {
        public DbvtNode a;
        public DbvtNode b;

        public sStkNN(DbvtNode na, DbvtNode nb) { a = na; b = nb; }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    public struct sStkNP
    {
        public DbvtNode node;
        public uint mask;
        public sStkNP(DbvtNode n, uint m) { node = n; mask = m; }
    };

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    public struct sStkNPS
    {
        public DbvtNode node;
        public uint mask;
        public float value;
        public sStkNPS(DbvtNode n, uint m, float v) { node = n; mask = m; value = v; }
    };

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////

    public struct DbvtAabbMm
    {
        public Vector3 Center() { return (_max + _min) / 2f; }
        public Vector3 Extent() { return (_max - _min) / 2f; }
        public Vector3 Mins() { return _min; }    // should be ref?
        public Vector3 Maxs() { return _max; }    // should be ref?
        public Vector3 Lengths() { return new Vector3(); }

        public static float Proximity(ref DbvtAabbMm a, ref DbvtAabbMm b)
        {
            Vector3 d = (a._min + a._max) - (b._min + b._max);
            return (Math.Abs(d.X) + Math.Abs(d.Y) + Math.Abs(d.Z));
        }


        public static DbvtAabbMm Merge(ref DbvtAabbMm a, ref DbvtAabbMm b)
        {
            DbvtAabbMm res = new DbvtAabbMm();
            Merge(ref a, ref b, ref res);
            return (res);
        }

        public static void Merge(ref DbvtAabbMm a, ref DbvtAabbMm b, ref DbvtAabbMm r)
        {
            //r = a;
            //SetMin(ref r._min, ref b._min);
            //SetMax(ref r._max, ref b._max);
            MathUtil.VectorMin(ref a._min, ref b._min, out r._min);
            MathUtil.VectorMax(ref a._max, ref b._max, out r._max);

        }

        public static bool NotEqual(ref DbvtAabbMm a, ref DbvtAabbMm b)
        {
            return ((a._min.X != b._min.X) ||
                    (a._min.Y != b._min.Y) ||
                    (a._min.Z != b._min.Z) ||
                    (a._max.X != b._max.X) ||
                    (a._max.Y != b._max.Y) ||
                    (a._max.Z != b._max.Z));

        }


        public static DbvtAabbMm FromCE(ref Vector3 c, ref Vector3 e)
        {
            DbvtAabbMm box;
            box._min = c - e; box._max = c + e;
            return (box);
        }
        public static DbvtAabbMm FromCR(ref Vector3 c, float r)
        {
            Vector3 temp = new Vector3(r, r, r);
            return (FromCE(ref c, ref temp));
        }
        public static DbvtAabbMm FromMM(ref Vector3 mi, ref Vector3 mx)
        {
            DbvtAabbMm box;
            box._min = mi; box._max = mx;
            return (box);
        }

        public static DbvtAabbMm FromPoints(List<Vector3> points)
        {
            DbvtAabbMm box;
            box._min = box._max = points[0];
            for (int i = 1; i < points.Count; ++i)
            {
                Vector3 temp = points[i];
                //SetMin(ref box._min, ref temp);
                //SetMax(ref box._max, ref temp);
                MathUtil.VectorMin(ref temp, ref box._min);
                MathUtil.VectorMax(ref temp, ref box._max);


            }
            return (box);
        }

        public static DbvtAabbMm FromPoints(List<List<Vector3>> points)
        {
            return new DbvtAabbMm();
        }

        public void Expand(Vector3 e)
        {
            _min -= e; _max += e;
        }

        public void SignedExpand(Vector3 e)
        {
            if (e.X > 0) _max.X = _max.X + e.X; else _min.X = _min.X + e.X;
            if (e.Y > 0) _max.Y = _max.Y + e.Y; else _min.Y = _min.Y + e.Y;
            if (e.Z > 0) _max.Z = _max.Z + e.Z; else _min.Z = _min.Z + e.Z;
        }
        public bool Contain(ref DbvtAabbMm a)
        {
            return ((_min.X <= a._min.X) &&
                (_min.Y <= a._min.Y) &&
                (_min.Z <= a._min.Z) &&
                (_max.X >= a._max.X) &&
                (_max.Y >= a._max.Y) &&
                (_max.Z >= a._max.Z));
        }

        public static bool Intersect(ref DbvtAabbMm a, ref DbvtAabbMm b)
        {
            return ((a._min.X <= b._max.X) &&
                (a._max.X >= b._min.X) &&
                (a._min.Y <= b._max.Y) &&
                (a._max.Y >= b._min.Y) &&
                (a._min.Z <= b._max.Z) &&
                (a._max.Z >= b._min.Z));
        }

        public static bool Intersect(DbvtAabbMm a, ref Vector3 b)
        {
            return ((b.X >= a._min.X) &&
                (b.Y >= a._min.Y) &&
                (b.Z >= a._min.Z) &&
                (b.X <= a._max.X) &&
                (b.Y <= a._max.Y) &&
                (b.Z <= a._max.Z));
        }



        public int Classify(ref Vector3 n, float o, int s)
        {
            Vector3 pi, px;
            switch (s)
            {
                case (0 + 0 + 0):
                    {
                        px = new Vector3(_min.X, _min.Y, _min.Z);
                        pi = new Vector3(_max.X, _max.Y, _max.Z);
                        break;
                    }
                case (1 + 0 + 0):
                    {
                        px = new Vector3(_max.X, _min.Y, _min.Z);
                        pi = new Vector3(_min.X, _max.Y, _max.Z); break;
                    }
                case (0 + 2 + 0):
                    {
                        px = new Vector3(_min.X, _max.Y, _min.Z);
                        pi = new Vector3(_max.X, _min.Y, _max.Z); break;
                    }
                case (1 + 2 + 0):
                    {
                        px = new Vector3(_max.X, _max.Y, _min.Z);
                        pi = new Vector3(_min.X, _min.Y, _max.Z); break;
                    }
                case (0 + 0 + 4):
                    {
                        px = new Vector3(_min.X, _min.Y, _max.Z);
                        pi = new Vector3(_max.X, _max.Y, _min.Z); break;
                    }
                case (1 + 0 + 4):
                    {
                        px = new Vector3(_max.X, _min.Y, _max.Z);
                        pi = new Vector3(_min.X, _max.Y, _min.Z); break;
                    }
                case (0 + 2 + 4):
                    {
                        px = new Vector3(_min.X, _max.Y, _max.Z);
                        pi = new Vector3(_max.X, _min.Y, _min.Z); break;
                    }
                case (1 + 2 + 4):
                    {
                        px = new Vector3(_max.X, _max.Y, _max.Z);
                        pi = new Vector3(_min.X, _min.Y, _min.Z); break;
                    }
                default:
                    {
                        px = new Vector3();
                        pi = new Vector3();
                        break;
                    }
            }
            if ((Vector3.Dot(n, px) + o) < 0) return (-1);
            if ((Vector3.Dot(n, pi) + o) >= 0) return (+1);
            return (0);
        }

        public float ProjectMinimum(ref Vector3 v, uint signs)
        {
            Vector3[] b = { _max, _min };
            Vector3 p = new Vector3(b[(signs >> 0) & 1].X,
                                    b[(signs >> 1) & 1].Y,
                                    b[(signs >> 2) & 1].Z);
            return (Vector3.Dot(p, v));
        }


        public Vector3 _min;
        public Vector3 _max;
    }

    public class Collide
    {
        public virtual void Process(DbvtNode n, DbvtNode n2)
        {
        }
        public virtual void Process(DbvtNode n)
        {
        }
        public virtual void Process(DbvtNode n, float f)
        {
            Process(n);
        }
        public virtual bool Descent(DbvtNode n)
        {
            return true;
        }
        public virtual bool AllLeaves(DbvtNode n)
        {
            return true;
        }
    }


}
