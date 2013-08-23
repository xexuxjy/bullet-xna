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
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public class CompoundShape : CollisionShape
    {
        public CompoundShape() : this(true) { }
        public CompoundShape(bool enableDynamicAabbTree)
        {
            m_children = new ObjectArray<CompoundShapeChild>();
            m_localAabbMax = new IndexedVector3(float.MinValue);
            m_localAabbMin = new IndexedVector3(float.MaxValue);
            m_collisionMargin = 0f;
            m_localScaling = new IndexedVector3(1f);
            m_dynamicAabbTree = null;
            m_updateRevision = 1;
            m_shapeType = BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE;
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

        public void AddChildShape(ref IndexedMatrix localTransform, CollisionShape shape)
        {
            m_updateRevision++;
            //m_childTransforms.push_back(localTransform);
            //m_childShapes.push_back(shape);
            CompoundShapeChild child = new CompoundShapeChild();
            child.m_transform = localTransform;
            child.m_childShape = shape;
            child.m_childShapeType = shape.GetShapeType();
            child.m_childMargin = shape.GetMargin();

            //extend the local aabbMin/aabbMax
            IndexedVector3 localAabbMin;
            IndexedVector3 localAabbMax;
            shape.GetAabb(ref localTransform, out localAabbMin, out localAabbMax);
            MathUtil.VectorMin(ref localAabbMin, ref m_localAabbMin);
            MathUtil.VectorMax(ref localAabbMax, ref m_localAabbMax);

            if (m_dynamicAabbTree != null)
            {
                DbvtAabbMm bounds = DbvtAabbMm.FromMM(ref localAabbMin, ref localAabbMax);
                int index = m_children.Count;
                child.m_treeNode = m_dynamicAabbTree.Insert(ref bounds, (object)index);
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
            m_children.RemoveAtQuick(childShapeIndex);
            if (m_dynamicAabbTree != null && m_children.Count > childShapeIndex)
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

        public IndexedMatrix GetChildTransform(int index)
        {
            return m_children[index].m_transform;
        }

        ///set a new transform for a child, and update internal data structures (local aabb and dynamic tree)
        public void UpdateChildTransform(int childIndex, ref IndexedMatrix newChildTransform)
        {
            UpdateChildTransform(childIndex, ref newChildTransform, true);
        }

        public void UpdateChildTransform(int childIndex, ref IndexedMatrix newChildTransform, bool shouldRecalculateLocalAabb)
        {
            m_children[childIndex].m_transform = newChildTransform;

            if (m_dynamicAabbTree != null)
            {
                ///update the dynamic aabb tree
                IndexedVector3 localAabbMin;
                IndexedVector3 localAabbMax;
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

        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {

            for (int i = 0; i < m_children.Count; i++)
            {
                IndexedMatrix childTrans = GetChildTransform(i);
                IndexedVector3 childScale = m_children[i].m_childShape.GetLocalScaling();
                //		childScale = childScale * (childTrans.getBasis() * scaling);
                childScale = childScale * scaling / m_localScaling;
                m_children[i].m_childShape.SetLocalScaling(ref childScale);
                childTrans._origin = ((childTrans._origin) * scaling);
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
                    IndexedVector3 localAabbMin, localAabbMax;
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
        public override void GetAabb(ref IndexedMatrix trans, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            IndexedVector3 localHalfExtents = .5f * (m_localAabbMax - m_localAabbMin);
            IndexedVector3 localCenter = .5f * (m_localAabbMax + m_localAabbMin);

            //avoid an illegal AABB when there are no children
            if (m_children.Count == 0)
            {
                localHalfExtents = IndexedVector3.Zero;
                localCenter = IndexedVector3.Zero;
            }
            float margin = GetMargin();
            localHalfExtents += new IndexedVector3(margin);

         
           	IndexedBasisMatrix abs_b = trans._basis.Absolute();  

    	    IndexedVector3 center = trans * localCenter;

	        IndexedVector3 extent = new IndexedVector3(abs_b._el0.Dot(ref localHalfExtents),
		                            abs_b._el1.Dot(ref localHalfExtents),
		                            abs_b._el2.Dot(ref localHalfExtents));
 
            aabbMin = center - extent;
            aabbMax = center + extent;
        }
        /** Re-calculate the local Aabb. Is called at the end of removeChildShapes. 
        Use this yourself if you modify the children or their transforms. */
        public virtual void RecalculateLocalAabb()
        {
            // Recalculate the local aabb
            // Brute force, it iterates over all the shapes left.

            m_localAabbMin = new IndexedVector3(float.MaxValue);
            m_localAabbMax = new IndexedVector3(float.MinValue);

            //extend the local aabbMin/aabbMax
            IndexedVector3 localAabbMin;
            IndexedVector3 localAabbMax;
            for (int j = 0; j < m_children.Count; j++)
            {
                IndexedMatrix foo = m_children[j].m_transform;
                m_children[j].m_childShape.GetAabb(ref foo, out localAabbMin, out localAabbMax);
                MathUtil.VectorMin(ref localAabbMin, ref m_localAabbMin);
                MathUtil.VectorMax(ref localAabbMax, ref m_localAabbMax);
            }
        }

        public override IndexedVector3 GetLocalScaling()
        {
            return m_localScaling;
        }

        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {
            //approximation: take the inertia from the aabb for now
            IndexedMatrix ident = IndexedMatrix.Identity;
            IndexedVector3 aabbMin;
            IndexedVector3 aabbMax;
            GetAabb(ref ident, out aabbMin, out aabbMax);

            IndexedVector3 halfExtents = (aabbMax - aabbMin) * .5f;
            float lx = 2f * (halfExtents.X);
            float ly = 2f * (halfExtents.Y);
            float lz = 2f * (halfExtents.Z);

            inertia = new IndexedVector3(mass / (12.0f) * (ly * ly + lz * lz),
                            mass / (12.0f) * (lx * lx + lz * lz),
                            mass / (12.0f) * (lx * lx + ly * ly));
        }

        public override void SetMargin(float margin)
        {
            m_collisionMargin = margin;
        }

        public override float GetMargin()
        {
            return m_collisionMargin;
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
        public void CalculatePrincipalAxisTransform(IList<float> masses, ref IndexedMatrix principal, out IndexedVector3 inertia)
        {
            int n = m_children.Count;

            float totalMass = 0;
            IndexedVector3 center = IndexedVector3.Zero;

            for (int k = 0; k < n; k++)
            {
                Debug.Assert(masses[k] > 0f);
                center += m_children[k].m_transform._origin * masses[k];
                totalMass += masses[k];
            }

            Debug.Assert(totalMass > 0f);
            center /= totalMass;
            principal._origin = center;

            IndexedBasisMatrix tensor = new IndexedBasisMatrix();
            for (int k = 0; k < n; k++)
            {
                IndexedVector3 i;
                m_children[k].m_childShape.CalculateLocalInertia(masses[k], out i);

                IndexedMatrix t = m_children[k].m_transform;
                IndexedVector3 o = t._origin - center;

                //compute inertia tensor in coordinate system of compound shape
                IndexedBasisMatrix j = t._basis.Transpose();
                j._el0 *= i.X;
                j._el1 *= i.Y;
                j._el2 *= i.Z;
                j = t._basis * j;

                //add inertia tensor
                tensor._el0 += j._el0;
                tensor._el1 += j._el1;
                tensor._el2 += j._el2;
                //tensor += j;

                //compute inertia tensor of pointmass at o
                float o2 = o.LengthSquared();
                j._el0 = new IndexedVector3(o2, 0, 0);
                j._el1 = new IndexedVector3(0, o2, 0);
                j._el2 = new IndexedVector3(0, 0, o2);

                j._el0 += o * -o.X;
                j._el1 += o * -o.Y;
                j._el2 += o * -o.Z;

                //add inertia tensor of pointmass
                tensor._el0 += masses[k] * j._el0;
                tensor._el1 += masses[k] * j._el1;
                tensor._el2 += masses[k] * j._el2;
            }
            tensor.Diagonalize(out principal, 0.00001f, 20);
            inertia = new IndexedVector3(tensor._el0.X, tensor._el1.Y, tensor._el2.Z);
        }

        public int GetUpdateRevision()
        {
            return m_updateRevision;
        }

        private ObjectArray<CompoundShapeChild> m_children;
        private IndexedVector3 m_localAabbMin;
        private IndexedVector3 m_localAabbMax;

        private Dbvt m_dynamicAabbTree;

        ///increment m_updateRevision when adding/removing/replacing child shapes, so that some caches can be updated
        private int m_updateRevision;
        private float m_collisionMargin;
        protected IndexedVector3 m_localScaling;
    }

    public class CompoundShapeChild
    {
        public IndexedMatrix m_transform;
        public CollisionShape m_childShape;
        public BroadphaseNativeTypes m_childShapeType;
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
