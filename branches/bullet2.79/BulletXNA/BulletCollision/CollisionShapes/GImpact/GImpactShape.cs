///*! \file btGImpactShape.h
//\author Francisco Len Nßjera
//*/
///*
//This source file is part of GIMPACT Library.

//For the latest info, see http://gimpact.sourceforge.net/

//Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
//email: projectileman@yahoo.com


//This software is provided 'as-is', without any express or implied warranty.
//In no event will the authors be held liable for any damages arising from the use of this software.
//Permission is granted to anyone to use this software for any purpose,
//including commercial applications, and to alter it and redistribute it freely,
//subject to the following restrictions:

//1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//3. This notice may not be removed or altered from any source distribution.
//*/


//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using Microsoft.Xna.Framework;
//using BulletXNA.BulletCollision.CollisionShapes;
//using BulletXNA.BulletCollision.CollisionDispatch;
//using System.Diagnostics;
//using BulletXNA;

//namespace BulletXNA.BulletCollision.CollisionShapes.GImpact
//{
//    public enum eGIMPACT_SHAPE_TYPE
//    {
//        CONST_GIMPACT_COMPOUND_SHAPE = 0,
//        CONST_GIMPACT_TRIMESH_SHAPE_PART,
//        CONST_GIMPACT_TRIMESH_SHAPE
//    };

    
//    public class GImpactShapeInterface : ConcaveShape
//    {

//    protected AABB m_localAABB;
//    protected bool m_needs_update;
//    protected Vector3  localScaling;
//    protected GImpactBoxSet m_box_set;// optionally boxset

////! declare Quantized trees, (you can change to float based trees)
////typedef btGImpactQuantizedBvh btGImpactBoxSet;



//////! Helper class for tetrahedrons
////class btTetrahedronShapeEx:public btBU_Simplex1to4
////{
////public:
////    btTetrahedronShapeEx()
////    {
////        m_numVertices = 4;
////    }


////    SIMD_FORCE_INLINE void setVertices(
////        const btVector3 & v0,const btVector3 & v1,
////        const btVector3 & v2,const btVector3 & v3)
////    {
////        m_vertices[0] = v0;
////        m_vertices[1] = v1;
////        m_vertices[2] = v2;
////        m_vertices[3] = v3;
////        recalcLocalAabb();
////    }
////};


////! Base class for gimpact shapes
////class btGImpactShapeInterface : public ConcaveShape
////{
////protected:
////    btAABB m_localAABB;
////    bool m_needs_update;
////    btVector3  localScaling;
////    btGImpactBoxSet m_box_set;// optionally boxset

//    //! use this function for perfofm refit in bounding boxes
//    //! use this function for perfofm refit in bounding boxes
//    protected virtual void CalcLocalAABB()
//    {
//        LockChildShapes();
//        if(m_box_set.getNodeCount() == 0)
//        {
//            m_box_set.buildSet();
//        }
//        else
//        {
//            m_box_set.update();
//        }
//        UnlockChildShapes();

//        m_localAABB = m_box_set.getGlobalBox();
//    }


//    public GImpactShapeInterface()
//    {
//        m_shapeType=eGIMPACT_SHAPE_TYPE.GIMPACT_SHAPE_PROXYTYPE;
//        m_localAABB.Invalidate();
//        m_needs_update = true;
//        localScaling = Vector3.One;
//    }


//    //! performs refit operation
//    /*!
//    Updates the entire Box set of this shape.
//    \pre postUpdate() must be called for attemps to calculating the box set, else this function
//        will does nothing.
//    \post if m_needs_update == true, then it calls calcLocalAABB();
//    */
//    public void UpdateBound()
//    {
//        if(!m_needs_update) return;
//        CalcLocalAABB();
//        m_needs_update  = false;
//    }

//    //! If the Bounding box is not updated, then this class attemps to calculate it.
//    /*!
//    \post Calls updateBound() for update the box set.
//    */
//    public void GetAabb(ref Matrix  t,ref Vector3 aabbMin,ref Vector3 aabbMax)
//    {
//        btAABB transformedbox = m_localAABB;
//        transformedbox.appy_transform(t);
//        aabbMin = transformedbox.m_min;
//        aabbMax = transformedbox.m_max;
//    }

//    //! Tells to this object that is needed to refit the box set
//    virtual void PostUpdate()
//    {
//        m_needs_update = true;
//    }

//    //! Obtains the local box, which is the global calculated box of the total of subshapes
//    SIMD_FORCE_INLINE const btAABB & getLocalBox()
//    {
//        return m_localAABB;
//    }


//    public override int	GetShapeType()
//    {
//        return GIMPACT_SHAPE_PROXYTYPE;
//    }

//    /*!
//    \post You must call updateBound() for update the box set.
//    */
//    public override void SetLocalScaling(ref Vector3 scaling)
//    {
//        localScaling = scaling;
//        PostUpdate();
//    }

//    public override Vector3 GetLocalScaling()
//    {
//        return localScaling;
//    }


//    public override void SetMargin(float margin)
//    {
//        m_collisionMargin = margin;
//        int i = GetNumChildShapes();
//        while(i--)
//        {
//            CollisionShape child = GetChildShape(i);
//            child.SetMargin(margin);
//        }

//        m_needs_update = true;
//    }


//    //! Subshape member functions
//    //!@{

//    //! Base method for determinig which kind of GIMPACT shape we get
//    public virtual eGIMPACT_SHAPE_TYPE GetGImpactShapeType();

//    //! gets boxset
//    public GImpactBoxSet GetBoxSet()
//    {
//        return m_box_set;
//    }

//    //! Determines if this class has a hierarchy structure for sorting its primitives
//    public bool HasBoxSet()
//    {
//        if(m_box_set.GetNodeCount() == 0) return false;
//        return true;
//    }

//    //! Obtains the primitive manager
//    public abstract PrimitiveManagerBase GetPrimitiveManager();


//    //! Gets the number of children
//    public abstract int	GetNumChildShapes();

//    //! if true, then its children must get transforms.
//    public abstract bool ChildrenHasTransform();

//    //! Determines if this shape has triangles
//    public abstract bool NeedsRetrieveTriangles();

//    //! Determines if this shape has tetrahedrons
//    public abstract bool NeedsRetrieveTetrahedrons();

//    public abstract bool GetBulletTriangle(int prim_index,TriangleShapeEx  triangle);

//    public abstract void GetBulletTetrahedron(int prim_index,TetrahedronShapeEx & tetrahedron);



//    //! call when reading child shapes
//    public virtual void LockChildShapes()
//    {
//    }

//    public virtual void UnlockChildShapes()
//    {
//    }

//    //! if this trimesh
//    public void getPrimitiveTriangle(int index,PrimitiveTriangle triangle)
//    {
//        GetPrimitiveManager().Get_primitive_triangle(index,triangle);
//    }


//    //! Retrieves the bound from a child
//    /*!
//    */
//    public virtual void getChildAabb(int child_index,ref Matrix t,ref Vector3 aabbMin,ref Vector3 aabbMax)
//    {
//        btAABB child_aabb;
//        GetPrimitiveManager().get_primitive_box(child_index,child_aabb);
//        child_aabb.appy_transform(t);
//        aabbMin = child_aabb.m_min;
//        aabbMax = child_aabb.m_max;
//    }

//    //! Gets the children
//    public abstract CollisionShape GetChildShape(int index);


//    //! Gets the child
//    public abstract CollisionShape GetChildShape(int index);

//    //! Gets the children transform
//    public abstract Matrix GetChildTransform(int index);

//    //! Sets the children transform
//    /*!
//    \post You must call updateBound() for update the box set.
//    */
//    public abstract void SetChildTransform(int index, ref Matrix transform);

//    //!@}


//    //! virtual method for ray collision
//    public override void RayTest(ref Vector3 rayFrom, ref Vector3 rayTo, RayResultCallback resultCallback)
//    {
        
//    }

//    //! Function for retrieve triangles.
//    /*!
//    It gives the triangles in local space
//    */
//    public virtual void	ProcessAllTriangles(ITriangleCallback callback,ref Vector3 aabbMin,ref Vector3 aabbMax)
//    {
    
//    }

//    //!@}

//}

//    //! compound primitive manager
//    public class CompoundPrimitiveManager: PrimitiveManagerBase
//    {
//    public:
//        virtual ~CompoundPrimitiveManager() {}
//        btGImpactCompoundShape * m_compoundShape;


//        CompoundPrimitiveManager(const CompoundPrimitiveManager& compound)
//            : btPrimitiveManagerBase()
//        {
//            m_compoundShape = compound.m_compoundShape;
//        }

//        CompoundPrimitiveManager(btGImpactCompoundShape * compoundShape)
//        {
//            m_compoundShape = compoundShape;
//        }

//        CompoundPrimitiveManager()
//        {
//            m_compoundShape = NULL;
//        }

//        virtual bool is_trimesh() const
//        {
//            return false;
//        }

//        virtual int get_primitive_count() const
//        {
//            return (int )m_compoundShape.getNumChildShapes();
//        }

//        virtual void get_primitive_box(int prim_index ,btAABB & primbox) const
//        {
//            btTransform prim_trans;
//            if(m_compoundShape.childrenHasTransform())
//            {
//                prim_trans = m_compoundShape.getChildTransform(prim_index);
//            }
//            else
//            {
//                prim_trans.setIdentity();
//            }
//            const btCollisionShape* shape = m_compoundShape.getChildShape(prim_index);
//            shape.getAabb(prim_trans,primbox.m_min,primbox.m_max);
//        }

//        virtual void get_primitive_triangle(int prim_index,btPrimitiveTriangle & triangle) const
//        {
//            btAssert(0);
//            (void) prim_index; (void) triangle;
//        }

//    }



////! btGImpactCompoundShape allows to handle multiple btCollisionShape objects at once
///*!
//This class only can manage Convex subshapes
//*/
//public class GImpactCompoundShape : GImpactShapeInterface
//{

//    protected CompoundPrimitiveManager m_primitive_manager;
//    protected ObjectArray<Matrix>		m_childTransforms = new ObjectArray<Matrix>();
//    protected ObjectArray<CollisionShape>	m_childShapes = new ObjectArray<CollisionShape>();


//    public GImpactCompoundShape(bool children_has_transform = true)
//    {
//        (void) children_has_transform;
//        m_primitive_manager.m_compoundShape = this;
//        m_box_set.setPrimitiveManager(&m_primitive_manager);
//    }

//    public virtual void Cleanup()
//    {

//    }

//    //! if true, then its children must get transforms.
//    public virtual bool ChildrenHasTransform()
//    {
//        if(m_childTransforms.Count==0) return false;
//        return true;
//    }


//    //! Obtains the primitive manager
//    public virtual PrimitiveManagerBase GetPrimitiveManager() 
//    {
//        return m_primitive_manager;
//    }

//    //! Obtains the compopund primitive manager
//    public CompoundPrimitiveManager GetCompoundPrimitiveManager()
//    {
//        return m_primitive_manager;
//    }

//    //! Gets the number of children
//    public virtual int GetNumChildShapes()
//    {
//        return m_childShapes.Count;
//    }


//    //! Use this method for adding children. Only Convex shapes are allowed.
//    public void AddChildShape(ref Matrix localTransform,CollisionShape shape)
//    {
//        Debug.Assert(shape.IsConvex());
//        m_childTransforms.Add(localTransform);
//        m_childShapes.Add(shape);
//    }

//    //! Use this method for adding children. Only Convex shapes are allowed.
//    void AddChildShape(CollisionShape shape)
//    {
//        Debug.Assert(shape.IsConvex());
//        m_childShapes.Add(shape);
//    }

//    //! Gets the children
//    public abstract CollisionShape GetChildShape(int index)
//    {
//        return m_childShapes[index];
//    }

//    //! Retrieves the bound from a child
//    /*!
//    */
//    virtual void GetChildAabb(int child_index,ref Matrix  t,ref Vector3 aabbMin,ref Vector3 aabbMax)
//    {
//        if(ChildrenHasTransform())
//        {
//            Matrix temp = MathUtil.BulletMatrixMultiply(t,m_childTransforms[child_index]);
//            m_childShapes[child_index].GetAabb(ref temp,ref aabbMin,ref aabbMax);
//        }
//        else
//        {
//            m_childShapes[child_index].GetAabb(ref t,ref aabbMin,ref aabbMax);
//        }
//    }


//    //! Gets the children transform
//    public virtual Matrix GetChildTransform(int index)
//    {
//        Debug.Assert(m_childTransforms.Count == m_childShapes.Count);
//        return m_childTransforms[index];
//    }

//    //! Sets the children transform
//    /*!
//    \post You must call updateBound() for update the box set.
//    */
//    public virtual void SetChildTransform(int index, ref Matrix transform)
//    {
//        Debug.Assert(m_childTransforms.Count == m_childShapes.Count);
//        m_childTransforms[index] = transform;
//        PostUpdate();
//    }

//    //! Determines if this shape has triangles
//    public virtual bool NeedsRetrieveTriangles()
//    {
//        return false;
//    }

//    //! Determines if this shape has tetrahedrons
//    public virtual bool NeedsRetrieveTetrahedrons()
//    {
//        return false;
//    }

//    public virtual void GetBulletTriangle(int prim_index,TriangleShapeEx triangle)
//    {
//        Debug.Assert(false);
//    }

//    public virtual void gGetBulletTetrahedron(int prim_index,TetrahedronShapeEx tetrahedron)
//    {
//        Debug.Assert(false);
//    }


//    //! Calculates the exact inertia tensor for this shape
//    public virtual void	CalculateLocalInertia(float mass,ref Vector3 inertia)
//    {

//    }

//    public override String GetName()
//    {
//        return "GImpactCompound";
//    }

//    virtual GIMPACT_SHAPE_TYPE GetGImpactShapeType()
//    {
//        return GIMPACT_SHAPE_TYPE.CONST_GIMPACT_COMPOUND_SHAPE;
//    }

//}



////! This class manages a sub part of a mesh supplied by the btStridingMeshInterface interface.
///*!
//- Simply create this shape by passing the btStridingMeshInterface to the constructor btGImpactMeshShapePart, then you must call updateBound() after creating the mesh
//- When making operations with this shape, you must call <b>lock</b> before accessing to the trimesh primitives, and then call <b>unlock</b>
//- You can handle deformable meshes with this shape, by calling postUpdate() every time when changing the mesh vertices.

//*/
//class btGImpactMeshShapePart : public GImpactShapeInterface
//{
//public:
//    //! Trimesh primitive manager
//    /*!
//    Manages the info from btStridingMeshInterface object and controls the Lock/Unlock mechanism
//    */
//    class TrimeshPrimitiveManager:public btPrimitiveManagerBase
//    {
//    public:
//        float m_margin;
//        btStridingMeshInterface * m_meshInterface;
//        btVector3 m_scale;
//        int m_part;
//        int m_lock_count;
//        const unsigned char *vertexbase;
//        int numverts;
//        PHY_ScalarType type;
//        int stride;
//        const unsigned char *indexbase;
//        int indexstride;
//        int  numfaces;
//        PHY_ScalarType indicestype;

//        TrimeshPrimitiveManager()
//        {
//            m_meshInterface = NULL;
//            m_part = 0;
//            m_margin = 0.01f;
//            m_scale = btVector3(1.f,1.f,1.f);
//            m_lock_count = 0;
//            vertexbase = 0;
//            numverts = 0;
//            stride = 0;
//            indexbase = 0;
//            indexstride = 0;
//            numfaces = 0;
//        }

//        TrimeshPrimitiveManager(const TrimeshPrimitiveManager & manager)
//            : btPrimitiveManagerBase()
//        {
//            m_meshInterface = manager.m_meshInterface;
//            m_part = manager.m_part;
//            m_margin = manager.m_margin;
//            m_scale = manager.m_scale;
//            m_lock_count = 0;
//            vertexbase = 0;
//            numverts = 0;
//            stride = 0;
//            indexbase = 0;
//            indexstride = 0;
//            numfaces = 0;

//        }

//        TrimeshPrimitiveManager(
//            btStridingMeshInterface * meshInterface,	int part)
//        {
//            m_meshInterface = meshInterface;
//            m_part = part;
//            m_scale = m_meshInterface.getScaling();
//            m_margin = 0.1f;
//            m_lock_count = 0;
//            vertexbase = 0;
//            numverts = 0;
//            stride = 0;
//            indexbase = 0;
//            indexstride = 0;
//            numfaces = 0;

//        }

//        virtual ~TrimeshPrimitiveManager() {}

//        void lock()
//        {
//            if(m_lock_count>0)
//            {
//                m_lock_count++;
//                return;
//            }
//            m_meshInterface.getLockedReadOnlyVertexIndexBase(
//                &vertexbase,numverts,
//                type, stride,&indexbase, indexstride, numfaces,indicestype,m_part);

//            m_lock_count = 1;
//        }

//        void unlock()
//        {
//            if(m_lock_count == 0) return;
//            if(m_lock_count>1)
//            {
//                --m_lock_count;
//                return;
//            }
//            m_meshInterface.unLockReadOnlyVertexBase(m_part);
//            vertexbase = NULL;
//            m_lock_count = 0;
//        }

//        virtual bool is_trimesh() const
//        {
//            return true;
//        }

//        virtual int get_primitive_count() const
//        {
//            return (int )numfaces;
//        }

//        SIMD_FORCE_INLINE int get_vertex_count() const
//        {
//            return (int )numverts;
//        }

//        SIMD_FORCE_INLINE void get_indices(int face_index,int &i0,int &i1,int &i2) const
//        {
//            if(indicestype == PHY_SHORT)
//            {
//                short * s_indices = (short *)(indexbase + face_index*indexstride);
//                i0 = s_indices[0];
//                i1 = s_indices[1];
//                i2 = s_indices[2];
//            }
//            else
//            {
//                int * i_indices = (int *)(indexbase + face_index*indexstride);
//                i0 = i_indices[0];
//                i1 = i_indices[1];
//                i2 = i_indices[2];
//            }
//        }

//        SIMD_FORCE_INLINE void get_vertex(int vertex_index, btVector3 & vertex) const
//        {
//            if(type == PHY_DOUBLE)
//            {
//                double * dvertices = (double *)(vertexbase + vertex_index*stride);
//                vertex[0] = float(dvertices[0]*m_scale[0]);
//                vertex[1] = float(dvertices[1]*m_scale[1]);
//                vertex[2] = float(dvertices[2]*m_scale[2]);
//            }
//            else
//            {
//                float * svertices = (float *)(vertexbase + vertex_index*stride);
//                vertex[0] = svertices[0]*m_scale[0];
//                vertex[1] = svertices[1]*m_scale[1];
//                vertex[2] = svertices[2]*m_scale[2];
//            }
//        }

//        virtual void get_primitive_box(int prim_index ,btAABB & primbox) const
//        {
//            btPrimitiveTriangle  triangle;
//            get_primitive_triangle(prim_index,triangle);
//            primbox.calc_from_triangle_margin(
//                triangle.m_vertices[0],
//                triangle.m_vertices[1],triangle.m_vertices[2],triangle.m_margin);
//        }

//        virtual void get_primitive_triangle(int prim_index,btPrimitiveTriangle & triangle) const
//        {
//            int indices[3];
//            get_indices(prim_index,indices[0],indices[1],indices[2]);
//            get_vertex(indices[0],triangle.m_vertices[0]);
//            get_vertex(indices[1],triangle.m_vertices[1]);
//            get_vertex(indices[2],triangle.m_vertices[2]);
//            triangle.m_margin = m_margin;
//        }

//        SIMD_FORCE_INLINE void get_bullet_triangle(int prim_index,btTriangleShapeEx & triangle) const
//        {
//            int indices[3];
//            get_indices(prim_index,indices[0],indices[1],indices[2]);
//            get_vertex(indices[0],triangle.m_vertices1[0]);
//            get_vertex(indices[1],triangle.m_vertices1[1]);
//            get_vertex(indices[2],triangle.m_vertices1[2]);
//            triangle.setMargin(m_margin);
//        }

//    };


//protected:
//    TrimeshPrimitiveManager m_primitive_manager;
//public:

//    btGImpactMeshShapePart()
//    {
//        m_box_set.setPrimitiveManager(&m_primitive_manager);
//    }


//    btGImpactMeshShapePart(btStridingMeshInterface * meshInterface,	int part)
//    {
//        m_primitive_manager.m_meshInterface = meshInterface;
//        m_primitive_manager.m_part = part;
//        m_box_set.setPrimitiveManager(&m_primitive_manager);
//    }

//    virtual ~btGImpactMeshShapePart()
//    {
//    }

//    //! if true, then its children must get transforms.
//    virtual bool childrenHasTransform() const
//    {
//        return false;
//    }


//    //! call when reading child shapes
//    virtual void lockChildShapes() const
//    {
//        void * dummy = (void*)(m_box_set.getPrimitiveManager());
//        TrimeshPrimitiveManager * dummymanager = static_cast<TrimeshPrimitiveManager *>(dummy);
//        dummymanager.lock();
//    }

//    virtual void unlockChildShapes()  const
//    {
//        void * dummy = (void*)(m_box_set.getPrimitiveManager());
//        TrimeshPrimitiveManager * dummymanager = static_cast<TrimeshPrimitiveManager *>(dummy);
//        dummymanager.unlock();
//    }

//    //! Gets the number of children
//    virtual int	getNumChildShapes() const
//    {
//        return m_primitive_manager.get_primitive_count();
//    }


//    //! Gets the children
//    virtual btCollisionShape* getChildShape(int index)
//    {
//        (void) index;
//        btAssert(0);
//        return NULL;
//    }



//    //! Gets the child
//    virtual const btCollisionShape* getChildShape(int index) const
//    {
//        (void) index;
//        btAssert(0);
//        return NULL;
//    }

//    //! Gets the children transform
//    virtual btTransform	getChildTransform(int index) const
//    {
//        (void) index;
//        btAssert(0);
//        return btTransform();
//    }

//    //! Sets the children transform
//    /*!
//    \post You must call updateBound() for update the box set.
//    */
//    virtual void setChildTransform(int index, const btTransform & transform)
//    {
//        (void) index;
//        (void) transform;
//        btAssert(0);
//    }


//    //! Obtains the primitive manager
//    virtual const btPrimitiveManagerBase * getPrimitiveManager()  const
//    {
//        return &m_primitive_manager;
//    }

//    SIMD_FORCE_INLINE TrimeshPrimitiveManager * getTrimeshPrimitiveManager()
//    {
//        return &m_primitive_manager;
//    }





//    virtual void	calculateLocalInertia(float mass,ref Vector3 inertia) const;




//    virtual const char*	getName()const
//    {
//        return "GImpactMeshShapePart";
//    }

//    virtual eGIMPACT_SHAPE_TYPE getGImpactShapeType() const
//    {
//        return CONST_GIMPACT_TRIMESH_SHAPE_PART;
//    }

//    //! Determines if this shape has triangles
//    virtual bool needsRetrieveTriangles() const
//    {
//        return true;
//    }

//    //! Determines if this shape has tetrahedrons
//    virtual bool needsRetrieveTetrahedrons() const
//    {
//        return false;
//    }

//    virtual void getBulletTriangle(int prim_index,btTriangleShapeEx & triangle) const
//    {
//        m_primitive_manager.get_bullet_triangle(prim_index,triangle);
//    }

//    virtual void getBulletTetrahedron(int prim_index,btTetrahedronShapeEx & tetrahedron) const
//    {
//        (void) prim_index;
//        (void) tetrahedron;
//        btAssert(0);
//    }



//    SIMD_FORCE_INLINE int getVertexCount() const
//    {
//        return m_primitive_manager.get_vertex_count();
//    }

//    SIMD_FORCE_INLINE void getVertex(int vertex_index, btVector3 & vertex) const
//    {
//        m_primitive_manager.get_vertex(vertex_index,vertex);
//    }

//    SIMD_FORCE_INLINE void setMargin(float margin)
//    {
//        m_primitive_manager.m_margin = margin;
//        postUpdate();
//    }

//    SIMD_FORCE_INLINE float getMargin() const
//    {
//        return m_primitive_manager.m_margin;
//    }

//    virtual void	setLocalScaling(const ref Vector3 scaling)
//    {
//        m_primitive_manager.m_scale = scaling;
//        postUpdate();
//    }

//    virtual const ref Vector3 getLocalScaling() const
//    {
//        return m_primitive_manager.m_scale;
//    }

//    SIMD_FORCE_INLINE int getPart() const
//    {
//        return (int)m_primitive_manager.m_part;
//    }

//    virtual void	processAllTriangles(btTriangleCallback* callback,const ref Vector3 aabbMin,const ref Vector3 aabbMax) const;
//};


////! This class manages a mesh supplied by the btStridingMeshInterface interface.
///*!
//Set of btGImpactMeshShapePart parts
//- Simply create this shape by passing the btStridingMeshInterface to the constructor btGImpactMeshShape, then you must call updateBound() after creating the mesh

//- You can handle deformable meshes with this shape, by calling postUpdate() every time when changing the mesh vertices.

//*/
//class btGImpactMeshShape : public btGImpactShapeInterface
//{
//    btStridingMeshInterface* m_meshInterface;

//protected:
//    btAlignedObjectArray<btGImpactMeshShapePart*> m_mesh_parts;
//    void buildMeshParts(btStridingMeshInterface * meshInterface)
//    {
//        for (int i=0;i<meshInterface.getNumSubParts() ;++i )
//        {
//            btGImpactMeshShapePart * newpart = new btGImpactMeshShapePart(meshInterface,i);
//            m_mesh_parts.push_back(newpart);
//        }
//    }

//    //! use this function for perfofm refit in bounding boxes
//    virtual void calcLocalAABB()
//    {
//        m_localAABB.invalidate();
//        int i = m_mesh_parts.size();
//        while(i--)
//        {
//            m_mesh_parts[i].updateBound();
//            m_localAABB.merge(m_mesh_parts[i].getLocalBox());
//        }
//    }

//public:
//    btGImpactMeshShape(btStridingMeshInterface * meshInterface)
//    {
//        m_meshInterface = meshInterface;
//        buildMeshParts(meshInterface);
//    }

//    virtual ~btGImpactMeshShape()
//    {
//        int i = m_mesh_parts.size();
//        while(i--)
//        {
//            btGImpactMeshShapePart * part = m_mesh_parts[i];
//            delete part;
//        }
//        m_mesh_parts.clear();
//    }


//    btStridingMeshInterface* getMeshInterface()
//    {
//        return m_meshInterface;
//    }

//    const btStridingMeshInterface* getMeshInterface() const
//    {
//        return m_meshInterface;
//    }

//    int getMeshPartCount() const
//    {
//        return m_mesh_parts.size();
//    }

//    btGImpactMeshShapePart * getMeshPart(int index)
//    {
//        return m_mesh_parts[index];
//    }



//    const btGImpactMeshShapePart * getMeshPart(int index) const
//    {
//        return m_mesh_parts[index];
//    }


//    virtual void	setLocalScaling(const ref Vector3 scaling)
//    {
//        localScaling = scaling;

//        int i = m_mesh_parts.size();
//        while(i--)
//        {
//            btGImpactMeshShapePart * part = m_mesh_parts[i];
//            part.setLocalScaling(scaling);
//        }

//        m_needs_update = true;
//    }

//    virtual void setMargin(float margin)
//    {
//        m_collisionMargin = margin;

//        int i = m_mesh_parts.size();
//        while(i--)
//        {
//            btGImpactMeshShapePart * part = m_mesh_parts[i];
//            part.setMargin(margin);
//        }

//        m_needs_update = true;
//    }

//    //! Tells to this object that is needed to refit all the meshes
//    virtual void postUpdate()
//    {
//        int i = m_mesh_parts.size();
//        while(i--)
//        {
//            btGImpactMeshShapePart * part = m_mesh_parts[i];
//            part.postUpdate();
//        }

//        m_needs_update = true;
//    }

//    virtual void	calculateLocalInertia(float mass,ref Vector3 inertia) const;


//    //! Obtains the primitive manager
//    virtual const btPrimitiveManagerBase * getPrimitiveManager()  const
//    {
//        btAssert(0);
//        return NULL;
//    }


//    //! Gets the number of children
//    virtual int	getNumChildShapes() const
//    {
//        btAssert(0);
//        return 0;
//    }


//    //! if true, then its children must get transforms.
//    virtual bool childrenHasTransform() const
//    {
//        btAssert(0);
//        return false;
//    }

//    //! Determines if this shape has triangles
//    virtual bool needsRetrieveTriangles() const
//    {
//        btAssert(0);
//        return false;
//    }

//    //! Determines if this shape has tetrahedrons
//    virtual bool needsRetrieveTetrahedrons() const
//    {
//        btAssert(0);
//        return false;
//    }

//    virtual void getBulletTriangle(int prim_index,btTriangleShapeEx & triangle) const
//    {
//        (void) prim_index; (void) triangle;
//        btAssert(0);
//    }

//    virtual void getBulletTetrahedron(int prim_index,btTetrahedronShapeEx & tetrahedron) const
//    {
//        (void) prim_index; (void) tetrahedron;
//        btAssert(0);
//    }

//    //! call when reading child shapes
//    virtual void lockChildShapes() const
//    {
//        btAssert(0);
//    }

//    virtual void unlockChildShapes() const
//    {
//        btAssert(0);
//    }




//    //! Retrieves the bound from a child
//    /*!
//    */
//    virtual void getChildAabb(int child_index,const btTransform& t,ref Vector3 aabbMin,ref Vector3 aabbMax) const
//    {
//        (void) child_index; (void) t; (void) aabbMin; (void) aabbMax;
//        btAssert(0);
//    }

//    //! Gets the children
//    virtual btCollisionShape* getChildShape(int index)
//    {
//        (void) index;
//        btAssert(0);
//        return NULL;
//    }


//    //! Gets the child
//    virtual const btCollisionShape* getChildShape(int index) const
//    {
//        (void) index;
//        btAssert(0);
//        return NULL;
//    }

//    //! Gets the children transform
//    virtual btTransform	getChildTransform(int index) const
//    {
//        (void) index;
//        btAssert(0);
//        return btTransform();
//    }

//    //! Sets the children transform
//    /*!
//    \post You must call updateBound() for update the box set.
//    */
//    virtual void setChildTransform(int index, const btTransform & transform)
//    {
//        (void) index; (void) transform;
//        btAssert(0);
//    }


//    virtual eGIMPACT_SHAPE_TYPE getGImpactShapeType() const
//    {
//        return CONST_GIMPACT_TRIMESH_SHAPE;
//    }


//    virtual const char*	getName()const
//    {
//        return "GImpactMesh";
//    }

//    virtual void rayTest(const ref Vector3 rayFrom, const ref Vector3 rayTo, btCollisionWorld::RayResultCallback& resultCallback)  const;

//    //! Function for retrieve triangles.
//    /*!
//    It gives the triangles in local space
//    */
//    virtual void	processAllTriangles(btTriangleCallback* callback,const ref Vector3 aabbMin,const ref Vector3 aabbMax) const;

//    virtual	int	calculateSerializeBufferSize() const;

//    ///fills the dataBuffer and returns the struct name (and 0 on failure)
//    virtual	const char*	serialize(void* dataBuffer, btSerializer* serializer) const;

//};

/////do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
//struct	btGImpactMeshShapeData
//{
//    btCollisionShapeData	m_collisionShapeData;

//    btStridingMeshInterfaceData m_meshInterface;

//    btVector3FloatData	m_localScaling;

//    float	m_collisionMargin;

//    int		m_gimpactSubType;
//};

//SIMD_FORCE_INLINE	int	btGImpactMeshShape::calculateSerializeBufferSize() const
//{
//    return sizeof(btGImpactMeshShapeData);
//}



//    }
//}
