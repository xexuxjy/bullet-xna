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
using System.Diagnostics;
using Microsoft.Xna.Framework;

namespace BulletXNA.BulletCollision
{
    public class CompoundShape : CollisionShape
    {
        public CompoundShape() : this(true) { }
        public CompoundShape(bool enableDynamicAabbTree)
        {
            m_children = new List<CompoundShapeChild>();
            m_localAabbMax = new Vector3(float.MinValue);
            m_localAabbMin = new Vector3(float.MaxValue);
            m_collisionMargin = 0f;
            m_localScaling = new Vector3(1f);
            m_dynamicAabbTree = null;
            m_updateRevision = 1;
            m_shapeType = BroadphaseNativeType.CompoundShape;
            if (enableDynamicAabbTree)
            {
                m_dynamicAabbTree = new Dbvt();
            }
        }

        public override void Cleanup()
        {
            base.Cleanup();
            if (m_dynamicAabbTree != null)
            {
                m_dynamicAabbTree.Cleanup();
                m_dynamicAabbTree = null;
            }
        }

        public void AddChildShape(ref Matrix localTransform, CollisionShape shape)
        {
            m_updateRevision++;
            //m_childTransforms.push_back(localTransform);
            //m_childShapes.push_back(shape);
            CompoundShapeChild child = new CompoundShapeChild();
            child.m_transform = localTransform;
            child.m_childShape = shape;
            child.m_childShapeType = shape.ShapeType;
            child.m_childMargin = shape.Margin;

            //extend the local aabbMin/aabbMax
            Vector3 localAabbMin;
            Vector3 localAabbMax;
            shape.GetAabb(ref localTransform, out localAabbMin, out localAabbMax);
            MathUtil.VectorMin(ref localAabbMin, ref m_localAabbMin);
            MathUtil.VectorMax(ref localAabbMax, ref m_localAabbMax);

            if (m_dynamicAabbTree != null)
            {
                DbvtAabbMm bounds = DbvtAabbMm.FromMM(ref localAabbMin, ref localAabbMax);
                int index = m_children.Count;
                child.m_treeNode = m_dynamicAabbTree.Insert(ref bounds, (Object)index);
            }

            m_children.Add(child);
        }

        /// Remove all children shapes that contain the specified shape
        public virtual void RemoveChildShape(CollisionShape shape)
        {
            m_updateRevision++;
            // Find the children containing the shape specified, and remove those children.
            //note: there might be multiple children using the same shape!
            for (int i = m_children.Count - 1; i >= 0; i--)
            {
                if (m_children[i].m_childShape == shape)
                {
                    RemoveChildShapeByIndex(i);
                }
            }
            RecalculateLocalAabb();
        }

        public void RemoveChildShapeByIndex(int childShapeIndex)
        {
            m_updateRevision++;
            Debug.Assert(childShapeIndex >= 0 && childShapeIndex < m_children.Count);
            if (m_dynamicAabbTree != null)
            {
                m_dynamicAabbTree.Remove(m_children[childShapeIndex].m_treeNode);
            }
            m_children.RemoveAt(childShapeIndex);
            if (m_dynamicAabbTree != null)
            {
                m_children[childShapeIndex].m_treeNode.dataAsInt = childShapeIndex;
            }

            //m_children[childShapeIndex] = m_children[m_children.Count - 1];
            //m_children.RemoveAt(m_children.Count - 1);
            //m_children.pop_back();
        }

        public int GetNumChildShapes()
        {
            return m_children.Count;
        }

        public CollisionShape GetChildShape(int index)
        {
            return m_children[index].m_childShape;
        }

        public Matrix GetChildTransform(int index)
        {
            return m_children[index].m_transform;
        }

        ///set a new transform for a child, and update internal data structures (local aabb and dynamic tree)
        public void UpdateChildTransform(int childIndex, ref Matrix newChildTransform)
        {
            UpdateChildTransform(childIndex, ref newChildTransform, true);
        }

        public void UpdateChildTransform(int childIndex, ref Matrix newChildTransform, bool shouldRecalculateLocalAabb)
        {
            m_children[childIndex].m_transform = newChildTransform;

            if (m_dynamicAabbTree != null)
            {
                ///update the dynamic aabb tree
                Vector3 localAabbMin;
                Vector3 localAabbMax;
                m_children[childIndex].m_childShape.GetAabb(ref newChildTransform, out localAabbMin, out localAabbMax);
                DbvtAabbMm bounds = DbvtAabbMm.FromMM(ref localAabbMin, ref localAabbMax);
                //int index = m_children.Count - 1;
                m_dynamicAabbTree.Update(m_children[childIndex].m_treeNode, ref bounds);
            }

            if (shouldRecalculateLocalAabb)
            {
                RecalculateLocalAabb();
            }
        }

        public override void SetLocalScaling(ref Vector3 scaling)
        {

            for (int i = 0; i < m_children.Count; i++)
            {
                Matrix childTrans = GetChildTransform(i);
                Vector3 childScale = m_children[i].m_childShape.GetLocalScaling();
                //		childScale = childScale * (childTrans.getBasis() * scaling);
                childScale = childScale * scaling / m_localScaling;
                m_children[i].m_childShape.SetLocalScaling(ref childScale);
                childTrans.Translation = ((childTrans.Translation) * scaling);
                UpdateChildTransform(i, ref childTrans, false);
            }
            m_localScaling = scaling;
            RecalculateLocalAabb();
        }

        public void CreateAabbTreeFromChildren()
        {
            if (m_dynamicAabbTree == null)
            {
                m_dynamicAabbTree = new Dbvt();

                for (int index = 0; index < m_children.Count; index++)
                {
                    CompoundShapeChild child = m_children[index];

                    //extend the local aabbMin/aabbMax
                    Vector3 localAabbMin, localAabbMax;
                    child.m_childShape.GetAabb(ref child.m_transform, out localAabbMin, out localAabbMax);

                    DbvtAabbMm bounds = DbvtAabbMm.FromMM(ref localAabbMin, ref localAabbMax);
                    child.m_treeNode = m_dynamicAabbTree.Insert(ref bounds, (object)index);
                }
            }
        }


        public IList<CompoundShapeChild> GetChildList()
        {
            return m_children;
        }

        ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
        public override void GetAabb(ref Matrix trans, out Vector3 aabbMin, out Vector3 aabbMax)
        {
            Vector3 localHalfExtents = .5f * (m_localAabbMax - m_localAabbMin);
            Vector3 localCenter = .5f * (m_localAabbMax + m_localAabbMin);

            //avoid an illegal AABB when there are no children
            if (m_children.Count == 0)
            {
                localHalfExtents = Vector3.Zero;
                localCenter = Vector3.Zero;
            }
            float margin = Margin;
            localHalfExtents += new Vector3(margin);

            Matrix abs_b;
            MathUtil.AbsoluteMatrix(ref trans, out abs_b);


            //Vector3 center = trans.Translation;
            Vector3 center = Vector3.Transform(localCenter, trans);


            Vector3 extent = new Vector3(Vector3.Dot(abs_b.Right, localHalfExtents),
                                            Vector3.Dot(abs_b.Up, localHalfExtents),
                                            Vector3.Dot(abs_b.Backward, localHalfExtents));
            aabbMin = center - extent;
            aabbMax = center + extent;
        }
        /** Re-calculate the local Aabb. Is called at the end of removeChildShapes. 
        Use this yourself if you modify the children or their transforms. */
        public virtual void RecalculateLocalAabb()
        {
            // Recalculate the local aabb
            // Brute force, it iterates over all the shapes left.

            m_localAabbMin = new Vector3(float.MaxValue);
            m_localAabbMax = new Vector3(float.MinValue);

            //extend the local aabbMin/aabbMax
            Vector3 localAabbMin;
            Vector3 localAabbMax;
            for (int j = 0; j < m_children.Count; j++)
            {
                Matrix foo = m_children[j].m_transform;
                m_children[j].m_childShape.GetAabb(ref foo, out localAabbMin, out localAabbMax);
                MathUtil.VectorMin(ref localAabbMin, ref m_localAabbMin);
                MathUtil.VectorMax(ref localAabbMax, ref m_localAabbMax);
            }
        }

        public override Vector3 GetLocalScaling()
        {
            return m_localScaling;
        }

        public override void CalculateLocalInertia(float mass, out Vector3 inertia)
        {
            //approximation: take the inertia from the aabb for now
            Matrix ident = Matrix.Identity;
            Vector3 aabbMin;
            Vector3 aabbMax;
            GetAabb(ref ident, out aabbMin, out aabbMax);

            Vector3 halfExtents = (aabbMax - aabbMin) * .5f;
            float lx = 2f * (halfExtents.X);
            float ly = 2f * (halfExtents.Y);
            float lz = 2f * (halfExtents.Z);

            inertia = new Vector3(mass / (12.0f) * (ly * ly + lz * lz),
                            mass / (12.0f) * (lx * lx + lz * lz),
                            mass / (12.0f) * (lx * lx + ly * ly));
        }

        public override float Margin
        {
            get
            {
                return m_collisionMargin;
            }
            set
            {
                m_collisionMargin = value;
            }
        }

        public override String GetName()
        {
            return "Compound";
        }

        //this is optional, but should make collision queries faster, by culling non-overlapping nodes
        //public void createAabbTreeFromChildren();

        public Dbvt GetDynamicAabbTree()
        {
            return m_dynamicAabbTree;
        }

        ///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
        ///and the center of mass to the current coordinate system. "masses" points to an array of masses of the children. The resulting transform
        ///"principal" has to be applied inversely to all children transforms in order for the local coordinate system of the compound
        ///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
        ///of the collision object by the principal transform.
        public void CalculatePrincipalAxisTransform(IList<float> masses, ref Matrix principal, out Vector3 inertia)
        {
            int n = m_children.Count;

            float totalMass = 0;
            Vector3 center = Vector3.Zero;

            for (int k = 0; k < n; k++)
            {
                Debug.Assert(masses[k] > 0f);
                center += m_children[k].m_transform.Translation * masses[k];
                totalMass += masses[k];
            }

            Debug.Assert(totalMass > 0f);
            center /= totalMass;
            principal.Translation = center;

            Matrix tensor = new Matrix();
            for (int k = 0; k < n; k++)
            {
                Vector3 i;
                m_children[k].m_childShape.CalculateLocalInertia(masses[k], out i);

                Matrix t = m_children[k].m_transform;
                Vector3 o = t.Translation - center;

                //compute inertia tensor in coordinate system of compound shape
                Matrix j = Matrix.Transpose(t);
                j.Right = j.Right * i.X;
                j.Up = j.Up * i.Y;
                j.Backward = j.Backward * i.Z;

                Matrix basis = MathUtil.BasisMatrix(ref t);
                j = basis * j;

                //add inertia tensor
                //tensor[0] += j[0];
                //tensor[1] += j[1];
                //tensor[2] += j[2];
                tensor += j;

                //compute inertia tensor of pointmass at o
                float o2 = o.LengthSquared();
                Vector3 a = new Vector3(o2, 0, 0);
                MathUtil.SetMatrixVector(ref j, 0, ref a);
                a = new Vector3(0, o2, 0);
                MathUtil.SetMatrixVector(ref j, 1, ref a);
                a = new Vector3(0, 0, o2);
                MathUtil.SetMatrixVector(ref j, 2, ref a);

                a = o * -o.X;
                MathUtil.SetMatrixVector(ref j, 0, ref a);
                a = o * -o.Y;
                MathUtil.SetMatrixVector(ref j, 1, ref a);
                a = o * -o.Z;
                MathUtil.SetMatrixVector(ref j, 2, ref a);

                //add inertia tensor of pointmass
                //tensor[0] += masses[k] * j[0];
                //tensor[1] += masses[k] * j[1];
                //tensor[2] += masses[k] * j[2];
                tensor.Right += masses[k] * j.Right;
                tensor.Up += masses[k] * j.Up;
                tensor.Backward += masses[k] * j.Backward;
            }
            MathUtil.Diagonalize(ref tensor, ref principal, 0.00001f, 20);
            inertia = new Vector3(tensor.M11, tensor.M22, tensor.M33);
        }

        public int GetUpdateRevision()
        {
            return m_updateRevision;
        }

        private IList<CompoundShapeChild> m_children;
        private Vector3 m_localAabbMin;
        private Vector3 m_localAabbMax;

        private Dbvt m_dynamicAabbTree;

        ///increment m_updateRevision when adding/removing/replacing child shapes, so that some caches can be updated
        private int m_updateRevision;
        private float m_collisionMargin;
        protected Vector3 m_localScaling;
    }

    public class CompoundShapeChild
    {
        public Matrix m_transform;
        public CollisionShape m_childShape;
        public BroadphaseNativeType m_childShapeType;
        public float m_childMargin;
        public DbvtNode m_treeNode;

        public override bool Equals(object obj)
        {
            CompoundShapeChild other = (CompoundShapeChild)obj;
            return (m_transform == other.m_transform &&
                m_childShape == other.m_childShape &&
                m_childShapeType == other.m_childShapeType &&
                m_childMargin == other.m_childMargin);
        }

    }

}
