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

namespace BulletXNA.BulletCollision
{
    public class DefaultCollisionConstructionInfo
    {
    }


    public class DefaultCollisionConfiguration : ICollisionConfiguration
    {
    ///btCollisionConfiguration allows to configure Bullet collision detection
    ///stack allocator, pool memory allocators
    ///@todo: describe the meaning

        protected int m_persistentManifoldPoolSize;
        protected bool m_useEpaPenetrationAlgorithm;

    //btStackAlloc*	m_stackAlloc;
    //bool	m_ownsStackAllocator;

    //btPoolAllocator*	m_persistentManifoldPool;
    //bool	m_ownsPersistentManifoldPool;


    //btPoolAllocator*	m_collisionAlgorithmPool;
    //bool	m_ownsCollisionAlgorithmPool;

	//default simplex/penetration depth solvers
	protected VoronoiSimplexSolver m_simplexSolver;
	protected IConvexPenetrationDepthSolver m_pdSolver;
	
	//default CreationFunctions, filling the m_doubleDispatch table
    CollisionAlgorithmCreateFunc m_convexConvexCreateFunc;
    CollisionAlgorithmCreateFunc m_convexConcaveCreateFunc;
    CollisionAlgorithmCreateFunc m_swappedConvexConcaveCreateFunc;
    CollisionAlgorithmCreateFunc m_compoundCreateFunc;
    CollisionAlgorithmCreateFunc m_swappedCompoundCreateFunc;
    CollisionAlgorithmCreateFunc m_emptyCreateFunc;
    CollisionAlgorithmCreateFunc m_sphereSphereCF;
    CollisionAlgorithmCreateFunc m_sphereBoxCF;
    CollisionAlgorithmCreateFunc m_boxSphereCF;

    CollisionAlgorithmCreateFunc m_boxBoxCF;
    CollisionAlgorithmCreateFunc m_sphereTriangleCF;
    CollisionAlgorithmCreateFunc m_triangleSphereCF;
    CollisionAlgorithmCreateFunc m_planeConvexCF;
    CollisionAlgorithmCreateFunc m_convexPlaneCF;
    CollisionAlgorithmCreateFunc m_convexAlgo2DCF;
    public DefaultCollisionConfiguration() : this(new DefaultCollisionConstructionInfo())
    {
    }
	public DefaultCollisionConfiguration(DefaultCollisionConstructionInfo constructionInfo)
    {
        m_simplexSolver = BulletGlobals.VoronoiSimplexSolverPool.Get();
        //m_pdSolver = new GjkEpaPenetrationDepthSolver();
        m_pdSolver = new MinkowskiPenetrationDepthSolver();
        m_useEpaPenetrationAlgorithm = true;

	    //default CreationFunctions, filling the m_doubleDispatch table
	    m_convexConvexCreateFunc = new ConvexConvexCreateFunc(m_simplexSolver,m_pdSolver);
	    m_convexConcaveCreateFunc = new ConvexConcaveCreateFunc();
	    m_swappedConvexConcaveCreateFunc = new SwappedConvexConcaveCreateFunc();
	    m_compoundCreateFunc = new CompoundCreateFunc();
	    m_swappedCompoundCreateFunc = new SwappedCompoundCreateFunc();
	    m_emptyCreateFunc = new EmptyCreateFunc();
	
    	m_sphereSphereCF = new SphereSphereCreateFunc();
        m_sphereBoxCF = new SphereBoxCreateFunc();
        m_boxSphereCF = new SwappedSphereBoxCreateFunc();

        m_convexAlgo2DCF = new Convex2dConvex2dCreateFunc(m_simplexSolver, m_pdSolver);
	    m_sphereTriangleCF = new SphereTriangleCreateFunc();
	    m_triangleSphereCF = new SphereTriangleCreateFunc();
	    m_triangleSphereCF.m_swapped = true;
    	
	    m_boxBoxCF = new BoxBoxCreateFunc();

	    //convex versus plane
	    m_convexPlaneCF = new ConvexPlaneCreateFunc();
	    m_planeConvexCF = new ConvexPlaneCreateFunc();
	    m_planeConvexCF.m_swapped = true;
    	
    }

    public virtual void Cleanup()
    {
    }

		///memory pools
    //virtual btPoolAllocator* getPersistentManifoldPool()
    //{
    //    return m_persistentManifoldPool;
    //}

    //virtual btPoolAllocator* getCollisionAlgorithmPool()
    //{
    //    return m_collisionAlgorithmPool;
    //}

    //virtual btStackAlloc*	getStackAllocator()
    //{
    //    return m_stackAlloc;
    //}

    public virtual VoronoiSimplexSolver GetSimplexSolver()
    {
        return m_simplexSolver;
    }

    public virtual CollisionAlgorithmCreateFunc GetCollisionAlgorithmCreateFunc(BroadphaseNativeTypes proxyType0, BroadphaseNativeTypes proxyType1)
    {

	    if ((proxyType0 == BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE) && (proxyType1==BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE))
	    {
		    return	m_sphereSphereCF;
	    }

	    if ((proxyType0 == BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE) && (proxyType1==BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE))
	    {
		    return	m_sphereBoxCF;
	    }

	    if ((proxyType0 == BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE ) && (proxyType1==BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE))
	    {
		    return	m_boxSphereCF;
	    }

        if ((proxyType0 == BroadphaseNativeTypes.CONVEX_2D_SHAPE_PROXYTYPE) && (proxyType1 == BroadphaseNativeTypes.CONVEX_2D_SHAPE_PROXYTYPE))
        {
            return m_convexAlgo2DCF;
        }

	    if ((proxyType0 == BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE ) && (proxyType1==BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE))
	    {
		    return	m_sphereTriangleCF;
	    }

	    if ((proxyType0 == BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE  ) && (proxyType1==BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE))
	    {
		    return	m_triangleSphereCF;
	    } 

	    if ((proxyType0 == BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE) && (proxyType1 == BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE))
	    {
		    return m_boxBoxCF;
	    }
    	
	    if (BroadphaseProxy.IsConvex(proxyType0) && (proxyType1 == BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE))
	    {
		    return m_convexPlaneCF;
	    }

	    if (BroadphaseProxy.IsConvex(proxyType1) && (proxyType0 == BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE))
	    {
		    return m_planeConvexCF;
	    }
	    if (BroadphaseProxy.IsConvex(proxyType0) && BroadphaseProxy.IsConvex(proxyType1))
	    {
		    return m_convexConvexCreateFunc;
	    }

	    if (BroadphaseProxy.IsConvex(proxyType0) && BroadphaseProxy.IsConcave(proxyType1))
	    {
		    return m_convexConcaveCreateFunc;
	    }

	    if (BroadphaseProxy.IsConvex(proxyType1) && BroadphaseProxy.IsConcave(proxyType0))
	    {
		    return m_swappedConvexConcaveCreateFunc;
	    }

	    if (BroadphaseProxy.IsCompound(proxyType0))
	    {
		    return m_compoundCreateFunc;
	    } 
        else
	    {
		    if (BroadphaseProxy.IsCompound(proxyType1))
		    {
			    return m_swappedCompoundCreateFunc;
		    }
	    }

	    //failed to find an algorithm
	    return m_emptyCreateFunc;
    }

	///Use this method to allow to generate multiple contact points between at once, between two objects using the generic convex-convex algorithm.
	///By default, this feature is disabled for best performance.
	///@param numPerturbationIterations controls the number of collision queries. Set it to zero to disable the feature.
	///@param minimumPointsPerturbationThreshold is the minimum number of points in the contact cache, above which the feature is disabled
	///3 is a good value for both params, if you want to enable the feature. This is because the default contact cache contains a maximum of 4 points, and one collision query at the unperturbed orientation is performed first.
	///See Bullet/Demos/CollisionDemo for an example how this feature gathers multiple points.
	///@todo we could add a per-object setting of those parameters, for level-of-detail collision detection.
	    public void	SetConvexConvexMultipointIterations()
        {
            SetConvexConvexMultipointIterations(3,3);
        }
        public void	SetConvexConvexMultipointIterations(int numPerturbationIterations, int minimumPointsPerturbationThreshold)
        {
            ConvexConvexCreateFunc convexConvex = (ConvexConvexCreateFunc)m_convexConvexCreateFunc;
            convexConvex.m_numPerturbationIterations = numPerturbationIterations;
            convexConvex.m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
        }

        public void SetPlaneConvexMultipointIterations()
        {
            SetPlaneConvexMultipointIterations(3, 3);
        }
        public void SetPlaneConvexMultipointIterations(int numPerturbationIterations, int minimumPointsPerturbationThreshold)
        {
            ConvexPlaneCreateFunc cpCF = (ConvexPlaneCreateFunc)m_convexPlaneCF;
            cpCF.m_numPerturbationIterations = numPerturbationIterations;
            cpCF.m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;

            ConvexPlaneCreateFunc pcCF = (ConvexPlaneCreateFunc)m_planeConvexCF;
	        pcCF.m_numPerturbationIterations = numPerturbationIterations;
	        pcCF.m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
        }
    }
}
