/*
 * 
 * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
 *
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*! \file btGImpactShape.h
\author Francisco Len Nßjera
*/
/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


using System;
using System.Collections.Generic;
using System.Diagnostics;
using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision
{
    public enum GIMPACT_SHAPE_TYPE
    {
        CONST_GIMPACT_COMPOUND_SHAPE = 0,
        CONST_GIMPACT_TRIMESH_SHAPE_PART,
        CONST_GIMPACT_TRIMESH_SHAPE
    };

    //! Helper class for tetrahedrons
    public class TetrahedronShapeEx : BU_Simplex1to4
    {
        public TetrahedronShapeEx()
        {
            m_numVertices = 4;
        }


        public void SetVertices(
            ref IndexedVector3 v0, ref IndexedVector3 v1,
            ref IndexedVector3 v2, ref IndexedVector3 v3)
        {
            m_vertices[0] = v0;
            m_vertices[1] = v1;
            m_vertices[2] = v2;
            m_vertices[3] = v3;
            RecalcLocalAabb();
        }
    }

    public abstract class GImpactShapeInterface : ConcaveShape
    {

        protected AABB m_localAABB;
        protected bool m_needs_update;
        protected IndexedVector3 localScaling;
        protected GImpactQuantizedBvh m_box_set = new GImpactQuantizedBvh();// optionally boxset

        //! declare Quantized trees, (you can change to float based trees)
        //typedef btGImpactQuantizedBvh btGImpactBoxSet;





        //! Base class for gimpact shapes
        //class btGImpactShapeInterface : public ConcaveShape
        //{
        //protected:
        //    btAABB m_localAABB;
        //    bool m_needs_update;
        //    btVector3  localScaling;
        //    btGImpactBoxSet m_box_set;// optionally boxset

        //! use this function for perfofm refit in bounding boxes
        //! use this function for perfofm refit in bounding boxes
        protected virtual void CalcLocalAABB()
        {
            LockChildShapes();
            if (m_box_set.GetNodeCount() == 0)
            {
                m_box_set.BuildSet();
            }
            else
            {
                m_box_set.Update();
            }
            UnlockChildShapes();

            m_localAABB = m_box_set.GetGlobalBox();


        }


        public GImpactShapeInterface()
        {
            m_shapeType = BroadphaseNativeTypes.GIMPACT_SHAPE_PROXYTYPE;
            m_localAABB.Invalidate();
            m_needs_update = true;
            localScaling = IndexedVector3.One;
        }


        //! performs refit operation
        /*!
        Updates the entire Box set of this shape.
        \pre postUpdate() must be called for attemps to calculating the box set, else this function
            will does nothing.
        \post if m_needs_update == true, then it calls calcLocalAABB();
        */
        public void UpdateBound()
        {
            if (!m_needs_update) return;
            CalcLocalAABB();
            m_needs_update = false;
        }

        //! If the Bounding box is not updated, then this class attemps to calculate it.
        /*!
        \post Calls updateBound() for update the box set.
        */
        public override void GetAabb(ref IndexedMatrix t, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            AABB transformedbox = m_localAABB;
            transformedbox.ApplyTransform(ref t);
            aabbMin = transformedbox.m_min;
            aabbMax = transformedbox.m_max;
        }

        //! Tells to this object that is needed to refit the box set
        public virtual void PostUpdate()
        {
            m_needs_update = true;
        }

        //! Obtains the local box, which is the global calculated box of the total of subshapes
        public AABB GetLocalBox()
        {
            return m_localAABB;
        }

        /*!
        \post You must call updateBound() for update the box set.
        */
        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {
            localScaling = scaling;
            PostUpdate();
        }

        public override IndexedVector3 GetLocalScaling()
        {
            return localScaling;
        }


        public override void SetMargin(float margin)
        {
            m_collisionMargin = margin;
            int i = GetNumChildShapes();
            while (i-- != 0)
            {
                CollisionShape child = GetChildShape(i);
                child.SetMargin(margin);
            }

            m_needs_update = true;
        }


        //! Subshape member functions
        //!@{

        //! Base method for determinig which kind of GIMPACT shape we get
        public abstract GIMPACT_SHAPE_TYPE GetGImpactShapeType();

        //! gets boxset
        public GImpactQuantizedBvh GetBoxSet()
        {
            return m_box_set;
        }

        //! Determines if this class has a hierarchy structure for sorting its primitives
        public bool HasBoxSet()
        {
            if (m_box_set.GetNodeCount() == 0) return false;
            return true;
        }

        //! Obtains the primitive manager
        public abstract IPrimitiveManagerBase GetPrimitiveManager();


        //! Gets the number of children
        public abstract int GetNumChildShapes();

        //! if true, then its children must get transforms.
        public abstract bool ChildrenHasTransform();

        //! Determines if this shape has triangles
        public abstract bool NeedsRetrieveTriangles();

        //! Determines if this shape has tetrahedrons
        public abstract bool NeedsRetrieveTetrahedrons();

        public abstract void GetBulletTriangle(int prim_index, TriangleShapeEx triangle);

        public abstract void GetBulletTetrahedron(int prim_index, TetrahedronShapeEx tetrahedron);



        //! call when reading child shapes
        public virtual void LockChildShapes()
        {
        }

        public virtual void UnlockChildShapes()
        {
        }

        //! if this trimesh
        public void GetPrimitiveTriangle(int index, PrimitiveTriangle triangle)
        {
            GetPrimitiveManager().GetPrimitiveTriangle(index, triangle);
        }


        //! Retrieves the bound from a child
        /*!
        */
        public virtual void GetChildAabb(int child_index, ref IndexedMatrix t, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            AABB child_aabb;
            GetPrimitiveManager().GetPrimitiveBox(child_index, out child_aabb);
            child_aabb.ApplyTransform(ref t);
            aabbMin = child_aabb.m_min;
            aabbMax = child_aabb.m_max;
        }

        //! Gets the child
        public abstract CollisionShape GetChildShape(int index);

        //! Gets the children transform
        public abstract IndexedMatrix GetChildTransform(int index);

        //! Sets the children transform
        /*!
        \post You must call updateBound() for update the box set.
        */
        public abstract void SetChildTransform(int index, ref IndexedMatrix transform);

        //!@}


        //! virtual method for ray collision
        public virtual void RayTest(ref IndexedVector3 rayFrom, ref IndexedVector3 rayTo, RayResultCallback resultCallback)
        {

        }

        //! Function for retrieve triangles.
        /*!
        It gives the triangles in local space
        */
        public override void ProcessAllTriangles(ITriangleCallback callback, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {

        }


        //!@}

    }

    //! compound primitive manager
    public class CompoundPrimitiveManager : IPrimitiveManagerBase
    {
        //virtual ~CompoundPrimitiveManager() {}
        public virtual void Cleanup()
        {
        }
        public GImpactCompoundShape m_compoundShape;


        public CompoundPrimitiveManager(CompoundPrimitiveManager compound)
        {
            m_compoundShape = compound.m_compoundShape;
        }

        public CompoundPrimitiveManager(GImpactCompoundShape compoundShape)
        {
            m_compoundShape = compoundShape;
        }

        public CompoundPrimitiveManager()
        {
            m_compoundShape = null;
        }

        public virtual bool IsTrimesh()
        {
            return false;
        }

        public virtual int GetPrimitiveCount()
        {
            return m_compoundShape.GetNumChildShapes();
        }

        public virtual void GetPrimitiveBox(int prim_index, out AABB primbox)
        {
            IndexedMatrix prim_trans;
            if (m_compoundShape.ChildrenHasTransform())
            {
                prim_trans = m_compoundShape.GetChildTransform(prim_index);
            }
            else
            {
                prim_trans = IndexedMatrix.Identity;
            }
            CollisionShape shape = m_compoundShape.GetChildShape(prim_index);
            shape.GetAabb(prim_trans, out primbox.m_min, out primbox.m_max);
        }

        public virtual void GetPrimitiveTriangle(int prim_index, PrimitiveTriangle triangle)
        {
            Debug.Assert(false);
        }

    }



    //! btGImpactCompoundShape allows to handle multiple btCollisionShape objects at once
    /*!
    This class only can manage Convex subshapes
    */
    public class GImpactCompoundShape : GImpactShapeInterface
    {

        protected CompoundPrimitiveManager m_primitive_manager;
        protected ObjectArray<IndexedMatrix> m_childTransforms = new ObjectArray<IndexedMatrix>();

        //protected ObjectArray<CollisionShape>	m_childShapes = new ObjectArray<CollisionShape>();
        // List here as collisionshape is abstract
        protected List<CollisionShape> m_childShapes = new List<CollisionShape>();


        public GImpactCompoundShape()
            : this(true)
        {

        }

        public GImpactCompoundShape(bool children_has_transform)
        {
            m_primitive_manager.m_compoundShape = this;
            m_box_set.SetPrimitiveManager(m_primitive_manager);
        }

        public override void Cleanup()
        {

        }

        //! if true, then its children must get transforms.
        public override bool ChildrenHasTransform()
        {
            if (m_childTransforms.Count == 0) return false;
            return true;
        }


        //! Obtains the primitive manager
        public override IPrimitiveManagerBase GetPrimitiveManager()
        {
            return m_primitive_manager;
        }

        //! Obtains the compopund primitive manager
        public CompoundPrimitiveManager GetCompoundPrimitiveManager()
        {
            return m_primitive_manager;
        }

        //! Gets the number of children
        public override int GetNumChildShapes()
        {
            return m_childShapes.Count;
        }


        //! Use this method for adding children. Only Convex shapes are allowed.
        public void AddChildShape(ref IndexedMatrix localTransform, CollisionShape shape)
        {
            Debug.Assert(shape.IsConvex());
            m_childTransforms.Add(localTransform);
            m_childShapes.Add(shape);
        }

        //! Use this method for adding children. Only Convex shapes are allowed.
        public void AddChildShape(CollisionShape shape)
        {
            Debug.Assert(shape.IsConvex());
            m_childShapes.Add(shape);
        }

        //! Gets the children
        public override CollisionShape GetChildShape(int index)
        {
            return m_childShapes[index];
        }

        //! Retrieves the bound from a child
        /*!
        */
        public override void GetChildAabb(int child_index, ref IndexedMatrix t, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            if (ChildrenHasTransform())
            {
                IndexedMatrix temp = t * m_childTransforms[child_index];
                m_childShapes[child_index].GetAabb(ref temp, out aabbMin, out aabbMax);
            }
            else
            {
                m_childShapes[child_index].GetAabb(ref t, out aabbMin, out aabbMax);
            }
        }


        //! Gets the children transform
        public override IndexedMatrix GetChildTransform(int index)
        {
            Debug.Assert(m_childTransforms.Count == m_childShapes.Count);
            return m_childTransforms[index];
        }

        //! Sets the children transform
        /*!
        \post You must call updateBound() for update the box set.
        */
        public override void SetChildTransform(int index, ref IndexedMatrix transform)
        {
            Debug.Assert(m_childTransforms.Count == m_childShapes.Count);
            m_childTransforms[index] = transform;
            PostUpdate();
        }

        //! Determines if this shape has triangles
        public override bool NeedsRetrieveTriangles()
        {
            return false;
        }

        //! Determines if this shape has tetrahedrons
        public override bool NeedsRetrieveTetrahedrons()
        {
            return false;
        }

        public override void GetBulletTriangle(int prim_index, TriangleShapeEx triangle)
        {
            Debug.Assert(false);
        }

        public override void GetBulletTetrahedron(int prim_index, TetrahedronShapeEx tetrahedron)
        {
            Debug.Assert(false);
        }


        //! Calculates the exact inertia tensor for this shape
        public virtual void CalculateLocalInertia(float mass, ref IndexedVector3 inertia)
        {
            LockChildShapes();
            inertia = IndexedVector3.Zero;

            int i = GetNumChildShapes();
            float shapemass = mass / ((float)i);

            while (i-- != 0)
            {
                IndexedVector3 temp_inertia;
                m_childShapes[i].CalculateLocalInertia(shapemass, out temp_inertia);
                if (ChildrenHasTransform())
                {

                    inertia = GImpactMassUtil.GimInertiaAddTransformed(ref inertia, ref temp_inertia, ref m_childTransforms.GetRawArray()[i]);
                }
                else
                {
                    IndexedMatrix identity = IndexedMatrix.Identity;
                    inertia = GImpactMassUtil.GimInertiaAddTransformed(ref inertia, ref temp_inertia, ref identity);
                }

            }


        }

        public override String GetName()
        {
            return "GImpactCompound";
        }

        public override GIMPACT_SHAPE_TYPE GetGImpactShapeType()
        {
            return GIMPACT_SHAPE_TYPE.CONST_GIMPACT_COMPOUND_SHAPE;
        }

    }



    //! This class manages a sub part of a mesh supplied by the btStridingMeshInterface interface.
    /*!
    - Simply create this shape by passing the btStridingMeshInterface to the constructor btGImpactMeshShapePart, then you must call updateBound() after creating the mesh
    - When making operations with this shape, you must call <b>lock</b> before accessing to the trimesh primitives, and then call <b>unlock</b>
    - You can handle deformable meshes with this shape, by calling postUpdate() every time when changing the mesh vertices.

    */
    public class GImpactMeshShapePart : GImpactShapeInterface
    {
        //! Trimesh primitive manager
        /*!
        Manages the info from btStridingMeshInterface object and controls the Lock/Unlock mechanism
        */
        public class TrimeshPrimitiveManager : IPrimitiveManagerBase
        {
            public float m_margin;
            public StridingMeshInterface m_meshInterface;
            public IndexedVector3 m_scale;
            public int m_part;
            public int m_lock_count;
            public object vertexbase;
            public int numverts;
            public PHY_ScalarType type;
            public int stride;
            public object indexbase;
            public int indexstride;
            public int numfaces;
            public PHY_ScalarType indicestype;
            public int[] indicesHolder = new int[3];

            public TrimeshPrimitiveManager()
            {
                m_margin = 0.01f;
                m_scale = IndexedVector3.One;
                m_lock_count = 0;
                vertexbase = null;
                numverts = 0;
                stride = 0;
                indexbase = null;
                indexstride = 0;
                numfaces = 0;
            }

            public TrimeshPrimitiveManager(TrimeshPrimitiveManager manager)
            {
                m_meshInterface = manager.m_meshInterface;
                m_part = manager.m_part;
                m_margin = manager.m_margin;
                m_scale = manager.m_scale;
                m_lock_count = 0;
                vertexbase = 0;
                numverts = 0;
                stride = 0;
                indexbase = 0;
                indexstride = 0;
                numfaces = 0;
            }

            public TrimeshPrimitiveManager(StridingMeshInterface meshInterface, int part)
            {
                m_meshInterface = meshInterface;
                m_part = part;
                m_scale = m_meshInterface.GetScaling();
                m_margin = 0.1f;
                m_lock_count = 0;
                vertexbase = 0;
                numverts = 0;
                stride = 0;
                indexbase = 0;
                indexstride = 0;
                numfaces = 0;
            }

            public virtual void Cleanup()
            {
            }

            public void Lock()
            {
                if (m_lock_count > 0)
                {
                    m_lock_count++;
                    return;
                }
                m_meshInterface.GetLockedReadOnlyVertexIndexBase(out 
                vertexbase, out numverts,
                    out type, out stride, out indexbase, out indexstride, out numfaces, out indicestype, m_part);

                m_lock_count = 1;
            }

            public void Unlock()
            {
                if (m_lock_count == 0) return;
                if (m_lock_count > 1)
                {
                    --m_lock_count;
                    return;
                }
                m_meshInterface.UnLockReadOnlyVertexBase(m_part);
                vertexbase = null;
                m_lock_count = 0;
            }

            public virtual bool IsTrimesh()
            {
                return true;
            }

            public virtual int GetPrimitiveCount()
            {
                return (int)numfaces;
            }

            public int GetVertexCount()
            {
                return numverts;
            }

            public void GetIndices(int face_index, out int i0, out int i1, out int i2)
            {
                if (indicestype == PHY_ScalarType.PHY_SHORT)
                {
                    ObjectArray<ushort> ushortArray = indexbase as ObjectArray<ushort>;
                    if (ushortArray != null)
                    {
                        ushort[] temp = ushortArray.GetRawArray();
                        int index = face_index * indexstride;
                        i0 = temp[index];
                        i1 = temp[index + 1];
                        i2 = temp[index + 2];
                    }
                    else
                    {
                        i0 = 0;
                        i1 = 1;
                        i2 = 2;
                    }
                }
                else
                {
                    ObjectArray<int> intArray = indexbase as ObjectArray<int>;
                    if (intArray != null)
                    {
                        int[] temp = intArray.GetRawArray();
                        int index = face_index * indexstride;
                        i0 = temp[index];
                        i1 = temp[index + 1];
                        i2 = temp[index + 2];
                    }
                    else
                    {
                        i0 = 0;
                        i1 = 1;
                        i2 = 2;
                    }
                }

            }

            public void GetVertex(int vertex_index, out IndexedVector3 vertex)
            {
                ObjectArray<IndexedVector3> svertices = vertexbase as ObjectArray<IndexedVector3>;
                vertex = IndexedVector3.Zero;
                if (svertices != null)
                {
                    int index = vertex_index * stride;
                    vertex = svertices[index] * m_scale.X;
                }
                else
                {
                    ObjectArray<float> fvertices = vertexbase as ObjectArray<float>;
                    if (svertices != null)
                    {
                        float[] temp = fvertices.GetRawArray();
                        int index = vertex_index * stride;
                        vertex.X = temp[index] * m_scale.X;
                        vertex.Y = temp[index + 1] * m_scale.Y;
                        vertex.Z = temp[index + 2] * m_scale.Z;
                    }

                }
            }

            public virtual void GetPrimitiveBox(int prim_index, out AABB primbox)
            {
                PrimitiveTriangle triangle = new PrimitiveTriangle();
                GetPrimitiveTriangle(prim_index, triangle);
                primbox = new AABB();
                primbox.CalcFromTriangleMargin(
                    ref triangle.m_vertices[0],
                    ref triangle.m_vertices[1], ref triangle.m_vertices[2], triangle.m_margin);
            }

            public virtual void GetPrimitiveTriangle(int prim_index, PrimitiveTriangle triangle)
            {
                GetIndices(prim_index, out indicesHolder[0], out indicesHolder[1], out indicesHolder[2]);
                GetVertex(indicesHolder[0], out triangle.m_vertices[0]);
                GetVertex(indicesHolder[1], out triangle.m_vertices[1]);
                GetVertex(indicesHolder[2], out triangle.m_vertices[2]);
                triangle.m_margin = m_margin;
            }

            public void GetBulletTriangle(int prim_index, TriangleShapeEx triangle)
            {
                GetIndices(prim_index, out indicesHolder[0], out indicesHolder[1], out indicesHolder[2]);
                GetVertex(indicesHolder[0], out triangle.m_vertices1[0]);
                GetVertex(indicesHolder[1], out triangle.m_vertices1[1]);
                GetVertex(indicesHolder[2], out triangle.m_vertices1[2]);
                triangle.SetMargin(m_margin);
            }

        };



        protected TrimeshPrimitiveManager m_primitive_manager = new TrimeshPrimitiveManager();

        public GImpactMeshShapePart()
        {
            m_box_set.SetPrimitiveManager(m_primitive_manager);
        }


        public GImpactMeshShapePart(StridingMeshInterface meshInterface, int part)
        {
            m_primitive_manager.m_meshInterface = meshInterface;
            m_primitive_manager.m_part = part;
            m_box_set.SetPrimitiveManager(m_primitive_manager);
        }

        public override void Cleanup()
        {
            base.Cleanup();
        }

        //! if true, then its children must get transforms.
        public override bool ChildrenHasTransform()
        {
            return false;
        }


        //! call when reading child shapes
        public override void LockChildShapes()
        {
            TrimeshPrimitiveManager dummymanager = m_box_set.GetPrimitiveManager() as TrimeshPrimitiveManager;
            dummymanager.Lock();
        }

        public override void UnlockChildShapes()
        {
            TrimeshPrimitiveManager dummymanager = m_box_set.GetPrimitiveManager() as TrimeshPrimitiveManager;
            dummymanager.Unlock();
        }

        //! Gets the number of children
        public override int GetNumChildShapes()
        {
            return m_primitive_manager.GetPrimitiveCount();
        }


        //! Gets the children
        public override CollisionShape GetChildShape(int index)
        {
            Debug.Assert(false);
            return null;
        }

        //! Gets the children transform
        public override IndexedMatrix GetChildTransform(int index)
        {
            Debug.Assert(false);
            return IndexedMatrix.Identity;
        }

        //! Sets the children transform
        /*!
        \post You must call updateBound() for update the box set.
        */
        public override void SetChildTransform(int index, ref IndexedMatrix transform)
        {
            Debug.Assert(false);
        }


        //! Obtains the primitive manager
        public override IPrimitiveManagerBase GetPrimitiveManager()
        {
            return m_primitive_manager;
        }

        public TrimeshPrimitiveManager GetTrimeshPrimitiveManager()
        {
            return m_primitive_manager;
        }

        public virtual void CalculateLocalInertia(float mass, ref IndexedVector3 inertia)
        {
            LockChildShapes();


            inertia = IndexedVector3.Zero;

            int i = GetVertexCount();
            float pointmass = mass / ((float)i);

            while (i-- != 0)
            {
                IndexedVector3 pointintertia;
                GetVertex(i, out pointintertia);
                pointintertia = GImpactMassUtil.GimGetPointInertia(ref pointintertia, pointmass);
                inertia += pointintertia;
            }

            UnlockChildShapes();

        }

        public override String GetName()
        {
            return "GImpactMeshShapePart";
        }

        public override GIMPACT_SHAPE_TYPE GetGImpactShapeType()
        {
            return GIMPACT_SHAPE_TYPE.CONST_GIMPACT_TRIMESH_SHAPE_PART;
        }

        //! Determines if this shape has triangles
        public override bool NeedsRetrieveTriangles()
        {
            return true;
        }

        //! Determines if this shape has tetrahedrons
        public override bool NeedsRetrieveTetrahedrons()
        {
            return false;
        }

        public override void GetBulletTriangle(int prim_index, TriangleShapeEx triangle)
        {
            m_primitive_manager.GetBulletTriangle(prim_index, triangle);
        }

        public override void GetBulletTetrahedron(int prim_index, TetrahedronShapeEx tetrahedron)
        {
            Debug.Assert(false);
        }



        public int GetVertexCount()
        {
            return m_primitive_manager.GetVertexCount();
        }

        public void GetVertex(int vertex_index, out IndexedVector3 vertex)
        {
            m_primitive_manager.GetVertex(vertex_index, out vertex);
        }

        public override void SetMargin(float margin)
        {
            m_primitive_manager.m_margin = margin;
            PostUpdate();
        }

        public override float GetMargin()
        {
            return m_primitive_manager.m_margin;
        }

        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {
            m_primitive_manager.m_scale = scaling;
            PostUpdate();
        }

        public override IndexedVector3 GetLocalScaling()
        {
            return m_primitive_manager.m_scale;
        }

        public int GetPart()
        {
            return m_primitive_manager.m_part;
        }

        public override void ProcessAllTriangles(ITriangleCallback callback, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
            LockChildShapes();
            AABB box = new AABB();
            box.m_min = aabbMin;
            box.m_max = aabbMax;

            ObjectArray<int> collided = new ObjectArray<int>();
            m_box_set.BoxQuery(ref box, collided);

            if (collided.Count == 0)
            {
                UnlockChildShapes();
                return;
            }

            int part = GetPart();
            PrimitiveTriangle triangle = new PrimitiveTriangle();
            int i = collided.Count;
            while (i-- != 0)
            {
                GetPrimitiveTriangle(collided[i], triangle);
                callback.ProcessTriangle(triangle.m_vertices, part, collided[i]);
            }
            UnlockChildShapes();
        }
    }


    //! This class manages a mesh supplied by the btStridingMeshInterface interface.
    /*!
    Set of btGImpactMeshShapePart parts
    - Simply create this shape by passing the btStridingMeshInterface to the constructor btGImpactMeshShape, then you must call updateBound() after creating the mesh

    - You can handle deformable meshes with this shape, by calling postUpdate() every time when changing the mesh vertices.

    */
    public class GImpactMeshShape : GImpactShapeInterface
    {
        StridingMeshInterface m_meshInterface;

        protected ObjectArray<GImpactMeshShapePart> m_mesh_parts = new ObjectArray<GImpactMeshShapePart>();

        protected void BuildMeshParts(StridingMeshInterface meshInterface)
        {
            for (int i = 0; i < meshInterface.GetNumSubParts(); ++i)
            {
                GImpactMeshShapePart newpart = new GImpactMeshShapePart(meshInterface, i);
                m_mesh_parts.Add(newpart);
            }
        }

        //! use this function for perfofm refit in bounding boxes
        protected override void CalcLocalAABB()
        {
            m_localAABB.Invalidate();
            int i = m_mesh_parts.Count;
            while (i-- != 0)
            {
                m_mesh_parts[i].UpdateBound();
                m_localAABB.Merge(m_mesh_parts[i].GetLocalBox());
            }
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactShape)
            {
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "GImpact CalcLocalAABB min ", m_localAABB.m_min);
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "GImpact CalcLocalAABB max ", m_localAABB.m_max);
            }
#endif
        }

        public GImpactMeshShape(StridingMeshInterface meshInterface)
        {
            m_meshInterface = meshInterface;
            BuildMeshParts(meshInterface);
        }

        public override void Cleanup()
        {
            base.Cleanup();
            m_mesh_parts.Clear();
        }


        public StridingMeshInterface GetMeshInterface()
        {
            return m_meshInterface;
        }


        public int GetMeshPartCount()
        {
            return m_mesh_parts.Count;
        }

        public GImpactMeshShapePart GetMeshPart(int index)
        {
            return m_mesh_parts[index];
        }

        public override void SetLocalScaling(ref IndexedVector3 scaling)
        {
            localScaling = scaling;

            int i = m_mesh_parts.Count;
            while (i-- != 0)
            {
                GImpactMeshShapePart part = m_mesh_parts[i];
                part.SetLocalScaling(ref scaling);
            }

            m_needs_update = true;
        }

        public override void SetMargin(float margin)
        {
            m_collisionMargin = margin;

            int i = m_mesh_parts.Count;
            while (i-- != 0)
            {
                GImpactMeshShapePart part = m_mesh_parts[i];
                part.SetMargin(margin);
            }

            m_needs_update = true;
        }

        //! Tells to this object that is needed to refit all the meshes
        public override void PostUpdate()
        {
            int i = m_mesh_parts.Count;
            while (i-- != 0)
            {
                GImpactMeshShapePart part = m_mesh_parts[i];
                part.PostUpdate();
            }

            m_needs_update = true;
        }

        public override void CalculateLocalInertia(float mass, out IndexedVector3 inertia)
        {
            inertia = IndexedVector3.Zero;

            int i = GetMeshPartCount();
            float partmass = mass / (float)i;

            while (i-- != 0)
            {
                IndexedVector3 partinertia = IndexedVector3.Zero;
                GetMeshPart(i).CalculateLocalInertia(partmass, ref partinertia);
                inertia += partinertia;
            }
#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugGimpactShape)
            {
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "GImpact CalculateLocalInertia", inertia);
            }
#endif
        }


        //! Obtains the primitive manager
        public override IPrimitiveManagerBase GetPrimitiveManager()
        {
            Debug.Assert(false);
            return null;
        }


        //! Gets the number of children
        public override int GetNumChildShapes()
        {
            Debug.Assert(false);
            return 0;
        }


        //! if true, then its children must get transforms.
        public override bool ChildrenHasTransform()
        {
            Debug.Assert(false);
            return false;
        }

        //! Determines if this shape has triangles
        public override bool NeedsRetrieveTriangles()
        {
            Debug.Assert(false);
            return false;
        }

        //! Determines if this shape has tetrahedrons
        public override bool NeedsRetrieveTetrahedrons()
        {
            Debug.Assert(false);
            return false;
        }

        public override void GetBulletTriangle(int prim_index, TriangleShapeEx triangle)
        {
            Debug.Assert(false);
        }

        public override void GetBulletTetrahedron(int prim_index, TetrahedronShapeEx tetrahedron)
        {
            Debug.Assert(false);
        }

        //! call when reading child shapes
        public override void LockChildShapes()
        {
            Debug.Assert(false);
        }

        public override void UnlockChildShapes()
        {
            Debug.Assert(false);
        }

        //! Retrieves the bound from a child
        /*!
        */
        public override void GetChildAabb(int child_index, ref IndexedMatrix t, out IndexedVector3 aabbMin, out IndexedVector3 aabbMax)
        {
            Debug.Assert(false);
            aabbMin = IndexedVector3.Zero;
            aabbMax = IndexedVector3.Zero;
        }

        //! Gets the children
        public override CollisionShape GetChildShape(int index)
        {
            Debug.Assert(false);
            return null;
        }


        //! Gets the children transform
        public override IndexedMatrix GetChildTransform(int index)
        {
            Debug.Assert(false);
            return IndexedMatrix.Identity;
        }

        //! Sets the children transform
        /*!
        \post You must call updateBound() for update the box set.
        */
        public override void SetChildTransform(int index, ref IndexedMatrix transform)
        {
            Debug.Assert(false);
        }


        public override GIMPACT_SHAPE_TYPE GetGImpactShapeType()
        {
            return GIMPACT_SHAPE_TYPE.CONST_GIMPACT_TRIMESH_SHAPE;
        }


        public override String GetName()
        {
            return "GImpactMesh";
        }

        public void RayTest(IndexedVector3 rayFrom, ref IndexedVector3 rayTo, RayResultCallback resultCallback)
        {

        }

        //! Function for retrieve triangles.
        /*!
        It gives the triangles in local space
        */
        public override void ProcessAllTriangles(ITriangleCallback callback, ref IndexedVector3 aabbMin, ref IndexedVector3 aabbMax)
        {
            int i = m_mesh_parts.Count;
            while (i-- != 0)
            {
                m_mesh_parts[i].ProcessAllTriangles(callback, ref aabbMin, ref aabbMax);
            }

        }
    }



}

