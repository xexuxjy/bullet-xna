using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BulletXNA.LinearMath;
using System.Diagnostics;

namespace BulletXNA.BulletCollision.CollisionDispatch
{
    public class CompoundCompoundCollisionAlgorithm : ActivatingCollisionAlgorithm
    {
        HashedSimplePairCache m_childCollisionAlgorithmCache;
        ObjectArray<SimplePair> m_removePairs = new ObjectArray<SimplePair>();

        PersistentManifold m_sharedManifold;
        bool m_ownsManifold;


        int m_compoundShapeRevision0;//to keep track of changes, so that childAlgorithm array can be updated
        int m_compoundShapeRevision1;

        private void RemoveChildAlgorithms()
        {
            ObjectArray<SimplePair> pairs = m_childCollisionAlgorithmCache.GetOverlappingPairArray();

            int numChildren = pairs.Count;
            int i;
            for (i = 0; i < numChildren; i++)
            {
                if (pairs[i].m_userPointer != null)
                {
                    CollisionAlgorithm algo = (CollisionAlgorithm)pairs[i].m_userPointer;
                    algo.Cleanup();
                    m_dispatcher.FreeCollisionAlgorithm(algo);
                }
            }
            m_childCollisionAlgorithmCache.RemoveAllPairs();
        }


        public CompoundCompoundCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1, bool isSwapped)
            : base(ci, body0, body1)
        {
            m_ownsManifold = false;

            m_childCollisionAlgorithmCache = new HashedSimplePairCache();

            //const btCollisionObjectWrapper* col0ObjWrap = body0Wrap;
            Debug.Assert(body0.GetCollisionShape().IsCompound());
            Debug.Assert(body1.GetCollisionShape().IsCompound());


            CompoundShape compoundShape0 = body0.GetCollisionShape() as CompoundShape;
            CompoundShape compoundShape1 = body1.GetCollisionShape() as CompoundShape;

            m_compoundShapeRevision0 = compoundShape0.GetUpdateRevision();
            m_compoundShapeRevision1 = compoundShape1.GetUpdateRevision();

        }

        public override void Cleanup()
        {
            base.Cleanup();
            RemoveChildAlgorithms();
        }
        //virtual ~btCompoundCompoundCollisionAlgorithm();



        public override void ProcessCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
	    Debug.Assert (body0.GetCollisionShape().IsCompound());
	    Debug.Assert (body1.GetCollisionShape().IsCompound());
	    CompoundShape compoundShape0 = body0.GetCollisionShape() as CompoundShape;
	    CompoundShape compoundShape1 = body1.GetCollisionShape() as CompoundShape;

	    ///btCompoundShape might have changed:
	    ////make sure the internal child collision algorithm caches are still valid
	    if ((compoundShape0.GetUpdateRevision() != m_compoundShapeRevision0) || (compoundShape1.GetUpdateRevision() != m_compoundShapeRevision1))
	    {
		    ///clear all
		    RemoveChildAlgorithms();
	    }


	    ///we need to refresh all contact manifolds
	    ///note that we should actually recursively traverse all children, btCompoundShape can nested more then 1 level deep
	    ///so we should add a 'refreshManifolds' in the btCollisionAlgorithm
	    {
		    int i;
		    PersistentManifoldArray manifoldArray = new PersistentManifoldArray();
		    ObjectArray<SimplePair> pairs = m_childCollisionAlgorithmCache.GetOverlappingPairArray();
		    for (i=0;i<pairs.Count;i++)
		    {
			    if (pairs[i].m_userPointer != null)
			    {
				    CollisionAlgorithm algo = (CollisionAlgorithm) pairs[i].m_userPointer;
				    algo.GetAllContactManifolds(manifoldArray);
				    for (int m=0;m<manifoldArray.Count;m++)
				    {
					    if (manifoldArray[m].GetNumContacts() > 0)
					    {
						    resultOut.SetPersistentManifold(manifoldArray[m]);
						    resultOut.RefreshContactPoints();
						    resultOut.SetPersistentManifold(null);
					    }
				    }
				    manifoldArray.Resize(0);
			    }
		    }
	    }


    	Dbvt tree0 = compoundShape0.GetDynamicAabbTree();
	    Dbvt tree1 = compoundShape1.GetDynamicAabbTree();

	    CompoundCompoundLeafCallback callback = new CompoundCompoundLeafCallback(body0,body1,m_dispatcher,dispatchInfo,resultOut,m_childCollisionAlgorithmCache,m_sharedManifold);
	    IndexedMatrix xform=body0.GetWorldTransform().Inverse()*body1.GetWorldTransform();
	    MycollideTT(tree0.m_root,tree1.m_root,xform,callback);

	//printf("#compound-compound child/leaf overlap =%d                      \r",callback.m_numOverlapPairs);

	//remove non-overlapping child pairs

	{
		Debug.Assert(m_removePairs.Count==0);

		//iterate over all children, perform an AABB check inside ProcessChildShape
		ObjectArray<SimplePair> pairs = m_childCollisionAlgorithmCache.GetOverlappingPairArray();
		
		int i;

        PersistentManifoldArray manifoldArray = new PersistentManifoldArray();
        
        IndexedVector3 aabbMin0,aabbMax0,aabbMin1,aabbMax1;        
        
		for (i=0;i<pairs.Count;i++)
		{
			if (pairs[i].m_userPointer != null)
			{
				CollisionAlgorithm algo = (CollisionAlgorithm)pairs[i].m_userPointer;

				{
					IndexedMatrix	orgTrans0;
					CollisionShape childShape0 = null;
					
					IndexedMatrix	newChildWorldTrans0;
					IndexedMatrix	orgInterpolationTrans0;
					childShape0 = compoundShape0.GetChildShape(pairs[i].m_indexA);
					orgTrans0 = body0.GetWorldTransform();
					orgInterpolationTrans0 = body0.GetWorldTransform();
					IndexedMatrix childTrans0 = compoundShape0.GetChildTransform(pairs[i].m_indexA);
					newChildWorldTrans0 = orgTrans0*childTrans0 ;
					childShape0.GetAabb(ref newChildWorldTrans0,out aabbMin0,out aabbMax0);
				}

				{
					IndexedMatrix	orgInterpolationTrans1;
					CollisionShape childShape1 = null;
					IndexedMatrix	orgTrans1;
					IndexedMatrix	newChildWorldTrans1;

					childShape1 = compoundShape1.GetChildShape(pairs[i].m_indexB);
					orgTrans1 = body1.GetWorldTransform();
					orgInterpolationTrans1 = body1.GetWorldTransform();
					IndexedMatrix childTrans1 = compoundShape1.GetChildTransform(pairs[i].m_indexB);
					newChildWorldTrans1 = orgTrans1*childTrans1 ;
					childShape1.GetAabb(ref newChildWorldTrans1,out aabbMin1,out aabbMax1);
				}
				
				

				if (!AabbUtil2.TestAabbAgainstAabb2(ref aabbMin0,ref aabbMax0,ref aabbMin1,ref aabbMax1))
				{
					algo.Cleanup();
					m_dispatcher.FreeCollisionAlgorithm(algo);
					m_removePairs.Add(new SimplePair(pairs[i].m_indexA,pairs[i].m_indexB));
				}
			}
		}
		for (int i=0;i<m_removePairs.Count;i++)
		{
			m_childCollisionAlgorithmCache.RemoveOverlappingPair(m_removePairs[i].m_indexA,m_removePairs[i].m_indexB);
		}
		m_removePairs.Clear();
	}

        }


        public float CalculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut)
        {
            Debug.Assert(false);
            return 0.f;
        }

        public override void GetAllContactManifolds(PersistentManifoldArray manifoldArray)
        {
            int i;
            ObjectArray<SimplePair> pairs = m_childCollisionAlgorithmCache.GetOverlappingPairArray();
            for (i = 0; i < pairs.Count; i++)
            {
                if (pairs[i].m_userPointer != null)
                {
                    ((CollisionAlgorithm)pairs[i].m_userPointer).GetAllContactManifolds(manifoldArray);
                }
            }
        }

        private static ObjectArray<sStkNN> CollideTTStack = new ObjectArray<sStkNN>(Dbvt.DOUBLE_STACKSIZE);

        public static bool MyIntersect(DbvtAabbMm a, DbvtAabbMm b, ref IndexedMatrix xform)
        {
            IndexedVector3 newmin, newmax;
            AabbUtil2.TransformAabb(b.Mins(), b.Maxs(), 0.f, xform, newmin, newmax);
            DbvtAabbMm newb = DbvtAabbMm.FromMM(ref newmin, ref newmax);
            return DbvtAabbMm.Intersect(a, newb);
        }


        public static void MycollideTT(DbvtNode root0,
                                      DbvtNode root1,
                                      IndexedMatrix xform,
                                      CompoundCompoundLeafCallback callback)
        {

            if (root0 != null && root1 != null)
            {
                int depth = 1;
                int treshold = Dbvt.DOUBLE_STACKSIZE - 4;
                ObjectArray<sStkNN> stkStack = new ObjectArray<sStkNN>(Dbvt.DOUBLE_STACKSIZE);
                //stkStack.resize(btDbvt::DOUBLE_STACKSIZE);
                stkStack[0] = new sStkNN(root0, root1);
                do
                {
                    sStkNN p = stkStack[--depth];
                    if (MyIntersect(p.a.volume, p.b.volume, xform))
                    {
                        if (depth > treshold)
                        {
                            stkStack.Resize(stkStack.Count * 2);
                            treshold = stkStack.Count - 4;
                        }
                        if (p.a.IsInternal())
                        {
                            if (p.b.IsInternal())
                            {
                                stkStack[depth++] = new sStkNN(p.a._children[0], p.b._children[0]);
                                stkStack[depth++] = new sStkNN(p.a._children[1], p.b._children[0]);
                                stkStack[depth++] = new sStkNN(p.a._children[0], p.b._children[1]);
                                stkStack[depth++] = new sStkNN(p.a._children[1], p.b._children[1]);
                            }
                            else
                            {
                                stkStack[depth++] = new sStkNN(p.a._children[0], p.b);
                                stkStack[depth++] = new sStkNN(p.a._children[1], p.b);
                            }
                        }
                        else
                        {
                            if (p.b.IsInternal())
                            {
                                stkStack[depth++] = new sStkNN(p.a, p.b._children[0]);
                                stkStack[depth++] = new sStkNN(p.a, p.b._children[1]);
                            }
                            else
                            {
                                callback.Process(p.a, p.b);
                            }
                        }
                    }
                } while (depth > 0);
            }
        }



    }

    public class CompoundCompoundLeafCallback : ICollide
    {
        int m_numOverlapPairs;


        CollisionObject m_compound0ColObjWrap;
        CollisionObject m_compound1ColObjWrap;
        IDispatcher m_dispatcher;
        DispatcherInfo m_dispatchInfo;
        ManifoldResult m_resultOut;


        HashedSimplePairCache m_childCollisionAlgorithmCache;

        PersistentManifold m_sharedManifold;

        CompoundCompoundLeafCallback(CollisionObject compound1ObjWrap,
                                        CollisionObject compound0ObjWrap,
                                        IDispatcher dispatcher,
                                        DispatcherInfo dispatchInfo,
                                        ManifoldResult resultOut,
                                        HashedSimplePairCache childAlgorithmsCache,
                                        PersistentManifold sharedManifold)
        {
            m_compound0ColObjWrap = compound1ObjWrap;
            m_compound1ColObjWrap = compound0ObjWrap;
            m_dispatcher = dispatcher;
            m_dispatchInfo = dispatchInfo;
            m_resultOut = resultOut;
            m_childCollisionAlgorithmCache = childAlgorithmsCache;
            m_sharedManifold = sharedManifold;
            m_numOverlapPairs = 0;
        }




        public override void Process(DbvtNode leaf0, DbvtNode leaf1)
	{
		m_numOverlapPairs++;


		int childIndex0 = leaf0.dataAsInt;
		int childIndex1 = leaf1.dataAsInt;
		

		Debug.Assert(childIndex0>=0);
		Debug.Assert(childIndex1>=0);


		CompoundShape compoundShape0 = m_compound0ColObjWrap.GetCollisionShape() as CompoundShape;
		Debug.Assert(childIndex0<compoundShape0.GetNumChildShapes());

		CompoundShape compoundShape1 = m_compound1ColObjWrap.GetCollisionShape() as CompoundShape;
		Debug.Assert(childIndex1<compoundShape1.GetNumChildShapes());

		CollisionShape childShape0 = compoundShape0.GetChildShape(childIndex0);
		CollisionShape childShape1 = compoundShape1.GetChildShape(childIndex1);

		//backup
		IndexedMatrix	orgTrans0 = m_compound0ColObjWrap.GetWorldTransform();
		IndexedMatrix childTrans0 = compoundShape0.GetChildTransform(childIndex0);
		IndexedMatrix	newChildWorldTrans0 = orgTrans0*childTrans0 ;
		
		IndexedMatrix	orgTrans1 = m_compound1ColObjWrap.GetWorldTransform();
		IndexedMatrix childTrans1 = compoundShape1.GetChildTransform(childIndex1);
		IndexedMatrix	newChildWorldTrans1 = orgTrans1*childTrans1 ;
		

		//perform an AABB check first
		IndexedVector3 aabbMin0,aabbMax0,aabbMin1,aabbMax1;
		childShape0.GetAabb(newChildWorldTrans0,out aabbMin0,out aabbMax0);
		childShape1.GetAabb(newChildWorldTrans1,out aabbMin1,out aabbMax1);
		
		if (gCompoundCompoundChildShapePairCallback)
		{
			if (!gCompoundCompoundChildShapePairCallback(childShape0,childShape1))
				return;
		}

		if (AabbUtil2.TestAabbAgainstAabb2(ref aabbMin0,ref aabbMax0,ref aabbMin1,ref aabbMax1))
		{
			CollisionObject compoundWrap0(this.m_compound0ColObjWrap,childShape0, m_compound0ColObjWrap.GetCollisionObject(),newChildWorldTrans0,-1,childIndex0);
			btCollisionObjectWrapper compoundWrap1(this.m_compound1ColObjWrap,childShape1,m_compound1ColObjWrap.GetCollisionObject(),newChildWorldTrans1,-1,childIndex1);
			

			SimplePair pair = m_childCollisionAlgorithmCache.FindPair(childIndex0,childIndex1);

			CollisionAlgorithm colAlgo = null;

			if (pair != null)
			{
				colAlgo = (CollisionAlgorithm)pair.m_userPointer;
				
			} else
			{
				colAlgo = m_dispatcher.FindAlgorithm(compoundWrap0,compoundWrap1,m_sharedManifold);
				pair = m_childCollisionAlgorithmCache.AddOverlappingPair(childIndex0,childIndex1);
				Debug.Assert(pair!=null);
				pair.m_userPointer = colAlgo;
			}

			Debug.Assert(colAlgo != null);
						
			CollisionObjectWrapper tmpWrap0 = null;
			CollisionObjectWrapper tmpWrap1 = null;

			tmpWrap0 = m_resultOut.GetBody0Wrap();
			tmpWrap1 = m_resultOut.GetBody1Wrap();

			m_resultOut.SetBody0Wrap(compoundWrap0);
			m_resultOut.SetBody1Wrap(compoundWrap1);

			m_resultOut.SetShapeIdentifiersA(-1,childIndex0);
			m_resultOut.SetShapeIdentifiersB(-1,childIndex1);


			colAlgo,ProcessCollision(compoundWrap0,&compoundWrap1,m_dispatchInfo,m_resultOut);
			
			m_resultOut.SetBody0Wrap(tmpWrap0);
			m_resultOut.SetBody1Wrap(tmpWrap1);
		}
	}
    }


}
