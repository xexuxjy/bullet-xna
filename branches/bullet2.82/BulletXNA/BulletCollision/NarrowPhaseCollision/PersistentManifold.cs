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

#define MAINTAIN_PERSISTENCY
#define KEEP_DEEPEST_POINT
//#define DEBUG_PERSISTENCY

using System;
using System.Diagnostics;

using BulletXNA.LinearMath;

namespace BulletXNA.BulletCollision 
{
    ///btPersistentManifold is a contact point cache, it stays persistent as long as objects are overlapping in the broadphase.
    ///Those contact points are created by the collision narrow phase.
    ///The cache can be empty, or hold 1,2,3 or 4 points. Some collision algorithms (GJK) might only add one point at a time.
    ///updates/refreshes old contact points, and throw them away if necessary (distance becomes too large)
    ///reduces the cache to 4 points, when more then 4 points are added, using following rules:
    ///the contact point with deepest penetration is always kept, and it tries to maximuze the area covered by the points
    ///note that some pairs of objects might have more then one contact manifold.
    public class PersistentManifold : TypedObject, IComparable
    {
        /// sort cached points so most isolated points come first
        private int SortCachedPoints(ManifoldPoint pt)
        {
            //calculate 4 possible cases areas, and take biggest area
            //also need to keep 'deepest'

            int maxPenetrationIndex = -1;
#if KEEP_DEEPEST_POINT
            float maxPenetration = pt.GetDistance();
            for (int i = 0; i < m_pointCache.Length; i++)
            {
                if (m_pointCache[i].GetDistance() < maxPenetration)
                {
                    maxPenetrationIndex = i;
                    maxPenetration = m_pointCache[i].GetDistance();
                }
            }
#endif //KEEP_DEEPEST_POINT

            float res0 = 0f, res1 = 0f, res2 = 0f, res3 = 0f;
            if (maxPenetrationIndex != 0)
            {
                IndexedVector3 a0 = pt.GetLocalPointA() - m_pointCache[1].GetLocalPointA();
                IndexedVector3 b0 = m_pointCache[3].GetLocalPointA() - m_pointCache[2].GetLocalPointA();
                IndexedVector3 cross = IndexedVector3.Cross(a0, b0);
                res0 = cross.LengthSquared();
            }
            if (maxPenetrationIndex != 1)
            {
                IndexedVector3 a1 = pt.GetLocalPointA() - m_pointCache[0].GetLocalPointA();
                IndexedVector3 b1 = m_pointCache[3].GetLocalPointA() - m_pointCache[2].GetLocalPointA();
                IndexedVector3 cross = IndexedVector3.Cross(a1, b1);
                res1 = cross.LengthSquared();
            }

            if (maxPenetrationIndex != 2)
            {
                IndexedVector3 a2 = pt.GetLocalPointA() - m_pointCache[0].GetLocalPointA();
                IndexedVector3 b2 = m_pointCache[3].GetLocalPointA() - m_pointCache[1].GetLocalPointA();
                IndexedVector3 cross = IndexedVector3.Cross(a2, b2);
                res2 = cross.LengthSquared();
            }

            if (maxPenetrationIndex != 3)
            {
                IndexedVector3 a3 = pt.GetLocalPointA() - m_pointCache[0].GetLocalPointA();
                IndexedVector3 b3 = m_pointCache[2].GetLocalPointA() - m_pointCache[1].GetLocalPointA();
                IndexedVector3 cross = IndexedVector3.Cross(a3, b3);
                res3 = cross.LengthSquared();
            }

            IndexedVector4 maxvec = new IndexedVector4(res0, res1, res2, res3);
            int biggestarea = MathUtil.ClosestAxis(ref maxvec);

#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugPersistentManifold)
            {
                BulletGlobals.g_streamWriter.WriteLine("sortCachedPoints [{0}]", biggestarea);
            }
#endif

            return biggestarea;

        }

        public int FindContactPoint(ref ManifoldPoint unUsed, int numUnused, ref ManifoldPoint pt)
        {
            return 0;
        }

        public PersistentManifold()
            : base((int)ContactManifoldTypes.BT_PERSISTENT_MANIFOLD_TYPE)
        {
        }

        public PersistentManifold(Object body0, Object body1, int foo, float contactBreakingThreshold, float contactProcessingThreshold)
            : base((int)ContactManifoldTypes.BT_PERSISTENT_MANIFOLD_TYPE)
        {
            m_body0 = body0;
            m_body1 = body1;
            m_contactBreakingThreshold = contactBreakingThreshold;
            m_contactProcessingThreshold = contactProcessingThreshold;
            m_cachedPoints = 0;
        }

        public void Initialise(Object body0, Object body1, int foo, float contactBreakingThreshold, float contactProcessingThreshold)
        {
            m_body0 = body0;
            m_body1 = body1;
            m_contactBreakingThreshold = contactBreakingThreshold;
            m_contactProcessingThreshold = contactProcessingThreshold;
            m_cachedPoints = 0;
        }

        public Object GetBody0()
        {
            return m_body0;
        }
        public Object GetBody1()
        {
            return m_body1;
        }

        public void SetBodies(Object body0, Object body1)
        {
            m_body0 = body0;
            m_body1 = body1;
        }

        public void ClearUserCache(ref ManifoldPoint pt)
        {
            Object oldPtr = pt.m_userPersistentData;
            if (oldPtr != null)
            {
#if DEBUG            
                if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugPersistentManifold)
                {

                    int occurance = 0;
                    for (int i = 0; i < m_cachedPoints; i++)
                    {
                        if (m_pointCache[i].m_userPersistentData == oldPtr)
                        {
                            occurance++;
                            if (occurance > 1)
                            {
                                BulletGlobals.g_streamWriter.WriteLine("error in clearUserCache\n");
                            }
                        }
                    }
                    //Debug.Assert(occurance<=0);
                }
#endif                
                if (pt.m_userPersistentData != null && gContactDestroyedCallback != null)
                {
                    gContactDestroyedCallback.Callback(pt.m_userPersistentData);
                    pt.m_userPersistentData = null;
                }

		        DebugPersistency();
            }
            BulletGlobals.ManifoldPointPool.Free(pt);
            pt = null;
        }

	    public void	DebugPersistency()
        {
#if DEBUG        
			if(BulletGlobals.g_streamWriter != null && BulletGlobals.debugPersistentManifold)
			{
				BulletGlobals.g_streamWriter.WriteLine("DebugPersistency : numPoints {0}", m_cachedPoints);
				for (int i = 0; i < m_cachedPoints; i++)
				{
					BulletGlobals.g_streamWriter.WriteLine(String.Format("m_pointCache[{0}]", i));
					MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "WorldA", m_pointCache[i].GetPositionWorldOnA());
					MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "WorldB", m_pointCache[i].GetPositionWorldOnB());
					MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "NormalB", m_pointCache[i].GetNormalWorldOnB());
				}
			}
#endif			
        }

        public int GetNumContacts()
        {
            return m_cachedPoints;
        }

        public ManifoldPoint GetContactPoint(int index)
        {
            Debug.Assert(index < m_cachedPoints);
            return m_pointCache[index];
        }

        ///@todo: get this margin from the current physics / collision environment
        public float GetContactBreakingThreshold()
        {
            return m_contactBreakingThreshold;
        }

        public float GetContactProcessingThreshold()
        {
            return m_contactProcessingThreshold;
        }

        public int GetCacheEntry(ManifoldPoint newPoint)
        {
            //float shortestDist = GetContactBreakingThreshold() * GetContactBreakingThreshold();
            float shortestDist = GetContactBreakingThreshold(); 
            shortestDist *= shortestDist;
            int size = GetNumContacts();
            int nearestPoint = -1;
            for (int i = 0; i < size; i++)
            {
                IndexedVector3 diffA = m_pointCache[i].GetLocalPointA() - newPoint.GetLocalPointA();
                float distToManiPoint = diffA.LengthSquared();
                if (distToManiPoint < shortestDist)
                {
                    shortestDist = distToManiPoint;
                    nearestPoint = i;
                }
            }
#if DEBUG            
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugPersistentManifold)
            {
                BulletGlobals.g_streamWriter.WriteLine("getCacheEntry [{0}]", nearestPoint);
            }
#endif

            return nearestPoint;
        }

        public int AddManifoldPoint(ManifoldPoint newPoint)
        {
            Debug.Assert(ValidContactDistance(newPoint));

            int insertIndex = GetNumContacts();
            if (insertIndex == MANIFOLD_CACHE_SIZE)
            {
                if (MANIFOLD_CACHE_SIZE >= 4)
                {
                    //sort cache so best points come first, based on area
                    insertIndex = SortCachedPoints(newPoint);
                }
                else
                {
                    insertIndex = 0;
                }
                ClearUserCache(ref m_pointCache[insertIndex]);

            }
            else
            {
                m_cachedPoints++;
            }
            if (insertIndex < 0)
            {
                insertIndex = 0;
            }

            //Debug.Assert(m_pointCache[insertIndex].GetUserPersistentData() == null);
            m_pointCache[insertIndex] = newPoint;

#if DEBUG
            if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugPersistentManifold)
            {
                BulletGlobals.g_streamWriter.WriteLine("addManifoldPoint[{0}][{1}]",insertIndex,m_cachedPoints);
                MathUtil.PrintContactPoint(BulletGlobals.g_streamWriter, newPoint);
            }
#endif

            return insertIndex;
        }

        public void RemoveContactPoint(int index)
        {
            ClearUserCache(ref m_pointCache[index]);

            int lastUsedIndex = GetNumContacts() - 1;
            //		m_pointCache[index] = m_pointCache[lastUsedIndex];
            if (index != lastUsedIndex)
            {
                //m_pointCache[index].Copy(m_pointCache[lastUsedIndex])
                m_pointCache[index] = m_pointCache[lastUsedIndex];
                m_pointCache[lastUsedIndex] = null; //?
                //get rid of duplicated userPersistentData pointer
                //m_pointCache[lastUsedIndex].Reset();
            }

            //Debug.Assert(m_pointCache[lastUsedIndex].GetUserPersistentData() == null);
            m_cachedPoints--;
        }

        public void ReplaceContactPoint(ManifoldPoint newPoint, int insertIndex)
        {
            Debug.Assert(ValidContactDistance(newPoint));


#if MAINTAIN_PERSISTENCY
            int lifeTime = m_pointCache[insertIndex].GetLifeTime();
            float appliedImpulse = m_pointCache[insertIndex].m_appliedImpulse;
            float appliedLateralImpulse1 = m_pointCache[insertIndex].m_appliedImpulseLateral1;
            float appliedLateralImpulse2 = m_pointCache[insertIndex].m_appliedImpulseLateral2;

            Debug.Assert(lifeTime >= 0);
            Object cache = m_pointCache[insertIndex].GetUserPersistentData();

            BulletGlobals.ManifoldPointPool.Free(m_pointCache[insertIndex]);

            m_pointCache[insertIndex] = newPoint;

            m_pointCache[insertIndex].SetUserPersistentData(cache);
            m_pointCache[insertIndex].SetAppliedImpulse(appliedImpulse);
            m_pointCache[insertIndex].SetAppliedImpulseLateral1(appliedLateralImpulse1);
            m_pointCache[insertIndex].SetAppliedImpulseLateral2(appliedLateralImpulse2);

            m_pointCache[insertIndex].m_constraintRow[0].m_accumImpulse = appliedImpulse;
            m_pointCache[insertIndex].m_constraintRow[1].m_accumImpulse = appliedLateralImpulse1;
            m_pointCache[insertIndex].m_constraintRow[2].m_accumImpulse = appliedLateralImpulse2;

            m_pointCache[insertIndex].SetLifeTime(lifeTime);
#else
		    ClearUserCache(ref m_pointCache[insertIndex]);
		    m_pointCache[insertIndex] = newPoint;
#endif
        }

        public bool ValidContactDistance(ManifoldPoint pt)
        {
            return pt.m_distance1 <= GetContactBreakingThreshold();
        }

        /// calculated new worldspace coordinates and depth, and reject points that exceed the collision margin
        public void RefreshContactPoints(ref IndexedMatrix trA, ref IndexedMatrix trB)
        {
#if DEBUG        
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugPersistentManifold)
            {
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "refreshContactPoints trA", trA._origin);
                MathUtil.PrintVector3(BulletGlobals.g_streamWriter, "refreshContactPoints trB", trB._origin);
            }
#endif

            /// first refresh worldspace positions and distance
            int numContacts = GetNumContacts() - 1;
            for (int i = numContacts; i >= 0; i--)
            {
                ManifoldPoint manifoldPoint = m_pointCache[i];
                IndexedVector3 pointA = trA * manifoldPoint.GetLocalPointA();
                IndexedVector3 pointB = trB * manifoldPoint.GetLocalPointB();

                manifoldPoint.SetPositionWorldOnA(ref pointA);
                manifoldPoint.SetPositionWorldOnB(ref pointB);

                manifoldPoint.SetDistance(IndexedVector3.Dot((manifoldPoint.GetPositionWorldOnA() - manifoldPoint.GetPositionWorldOnB()), manifoldPoint.GetNormalWorldOnB()));
                manifoldPoint.SetLifeTime(manifoldPoint.GetLifeTime() + 1);
                m_pointCache[i] = manifoldPoint;

            }

            /// then 
            float distance2d;
            IndexedVector3 projectedDifference, projectedPoint;
            for (int i = numContacts; i >= 0; i--)
            {
                ManifoldPoint manifoldPoint = m_pointCache[i];
                //contact becomes invalid when signed distance exceeds margin (projected on contactnormal direction)
                if (!ValidContactDistance(manifoldPoint))
                {
                    RemoveContactPoint(i);
                }
                else
                {
                    //contact also becomes invalid when relative movement orthogonal to normal exceeds margin
                    projectedPoint = manifoldPoint.GetPositionWorldOnA() - manifoldPoint.GetNormalWorldOnB() * manifoldPoint.GetDistance();
                    projectedDifference = manifoldPoint.GetPositionWorldOnB() - projectedPoint;
                    distance2d = IndexedVector3.Dot(projectedDifference, projectedDifference);
                    if (distance2d > GetContactBreakingThreshold() * GetContactBreakingThreshold())
                    {
                        RemoveContactPoint(i);
                    }
                    else
                    {
                        //contact point processed callback
                        if (gContactProcessedCallback != null)
                        {
                            gContactProcessedCallback.Callback(manifoldPoint, m_body0, m_body1);
                        }
                    }
                }
            }
            DebugPersistency();
        }


        public void ClearManifold()
        {
            for (int i = 0; i < m_cachedPoints; i++)
            {
                ClearUserCache(ref m_pointCache[i]);
            }
            m_cachedPoints = 0;
#if DEBUG            
			if (BulletGlobals.g_streamWriter != null && BulletGlobals.debugPersistentManifold)
            {
                BulletGlobals.g_streamWriter.WriteLine("clearManifold");
            }
#endif            
        }

        private ManifoldPoint[] m_pointCache = new ManifoldPoint[MANIFOLD_CACHE_SIZE];

        /// this two body pointers can point to the physics rigidbody class.
        /// void* will allow any rigidbody class
        private Object m_body0;
        private Object m_body1;
        private int m_cachedPoints;

        public int m_companionIdA;
        public int m_companionIdB;
	 
        public int m_index1a;

        private float m_contactBreakingThreshold;
        private float m_contactProcessingThreshold;
        private const int MANIFOLD_CACHE_SIZE = 4;

        public static IContactDestroyedCallback gContactDestroyedCallback = null;
        public static IContactProcessedCallback gContactProcessedCallback = null;

        #region IComparable Members

        public int CompareTo(object obj)
        {
            if (obj is PersistentManifold)
            {
                PersistentManifold otherManifold = (PersistentManifold)obj;
                return getIslandId(this) - getIslandId(otherManifold);
            }
            else
            {
                throw new ArgumentException("Object is not a PersistentManifold");
            }
        }

        #endregion

        private static int getIslandId(PersistentManifold lhs)
        {
            CollisionObject rcolObj0 = lhs.GetBody0() as CollisionObject;
            int islandId = rcolObj0.GetIslandTag();
            if (islandId >= 0)
                return rcolObj0.GetIslandTag();

            CollisionObject rcolObj1 = lhs.GetBody1() as CollisionObject;
            return rcolObj1.GetIslandTag();
        }
    }

    public interface IContactDestroyedCallback
    {
        bool Callback(Object userPersistentData);
    }

    public interface IContactProcessedCallback
    {
        bool Callback(ManifoldPoint point, Object body0, Object body1);
    }

    public enum ContactManifoldTypes
    {
        MIN_CONTACT_MANIFOLD_TYPE = 1024,
        BT_PERSISTENT_MANIFOLD_TYPE 
    }
}
