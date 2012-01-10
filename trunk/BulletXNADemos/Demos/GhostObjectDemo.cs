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

 Code of original appBasicDemo modified by Flix. (http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=6195)
*/


using BulletXNA.BulletCollision;
using BulletXNA.LinearMath;
using BulletXNA.BulletDynamics;
using BulletXNA;
using Microsoft.Xna.Framework;
using System;

namespace BulletXNADemos.Demos
{
    public class GhostObjectDemo : DemoApplication
    {


        protected override void Initialize()
        {
            base.Initialize();
            SetCameraDistance(50.0f);

            ///collision configuration contains default setup for memory, collision setup
            m_collisionConfiguration = new DefaultCollisionConfiguration();
            //m_collisionConfiguration.setConvexConvexMultipointIterations();

            ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
            m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);

            m_broadphase = new DbvtBroadphase();

            ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
            m_constraintSolver = new SequentialImpulseConstraintSolver();

            m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

            IndexedVector3 gravity = new IndexedVector3(0, -10, 0);
            m_dynamicsWorld.SetGravity(ref gravity);


            // NEW => btGhostPairCallback =================================
            m_ghostPairCallback = new GhostPairCallback();
            m_dynamicsWorld.GetBroadphase().GetOverlappingPairCache().SetInternalGhostPairCallback(m_ghostPairCallback);	// Needed once to enable ghost objects inside Bullet

            // NEW => btGhostObject =======================================
            m_ghostObject = new GhostObject();
            CollisionShape shape = new BoxShape(new IndexedVector3(5f));	// As far as I know only the world aabb of the shape will be used (i.e. a box always parallel to the x,y,z axis of variable size)
            m_collisionShapes.Add(shape);
            m_ghostObject.SetCollisionShape(shape);
            m_ghostObject.SetCollisionFlags(CollisionFlags.CF_NO_CONTACT_RESPONSE);		// We can choose to make it "solid" if we want...
            m_dynamicsWorld.AddCollisionObject(m_ghostObject, CollisionFilterGroups.SensorTrigger, CollisionFilterGroups.AllFilter & ~CollisionFilterGroups.SensorTrigger);
            //m_ghostObject.setWorldTransform(btTransform(btQuaternion::getIdentity(),btVector3(0,5,-15)));
            IndexedMatrix im = IndexedMatrix.CreateFromQuaternion(quatDeg45Y);
            im._origin = new IndexedVector3(0, 5, -15);
            m_ghostObject.SetWorldTransform(im);

            // NEW => btPairCachingGhostObject ============================
            m_pairCachingGhostObject = new PairCachingGhostObject();
            shape = new ConeShape(7.0f, 14.0f);
            m_collisionShapes.Add(shape);
            m_pairCachingGhostObject.SetCollisionShape(shape);
            m_pairCachingGhostObject.SetCollisionFlags(CollisionFlags.CF_NO_CONTACT_RESPONSE);	// We can choose to make it "solid" if we want...
            m_dynamicsWorld.AddCollisionObject(m_pairCachingGhostObject, CollisionFilterGroups.SensorTrigger, CollisionFilterGroups.AllFilter & ~CollisionFilterGroups.SensorTrigger);
            //m_pairCachingGhostObject.setWorldTransform(btTransform(btQuaternion::getIdentity(),btVector3(0,5,15)));
            im._origin = new IndexedVector3(0, 7, 15);
            m_pairCachingGhostObject.SetWorldTransform(im);
            //=============================================================

            ///create a few basic rigid bodies
            CollisionShape groundShape = new BoxShape(new IndexedVector3(50));

            m_collisionShapes.Add(groundShape);

            IndexedMatrix groundTransform = IndexedMatrix.Identity;
            groundTransform._origin = new IndexedVector3(0, -50, 0);

            float mass = 0.0f;
            LocalCreateRigidBody(mass, groundTransform, groundShape);

            // spawn some cubes (code pasted from appBasicDemo...)
            if(true)
            {
                //create a few dynamic rigidbodies
                CollisionShape colShape = new BoxShape(new IndexedVector3(SCALING, SCALING, SCALING));
                //btCollisionShape* colShape = new btSphereShape(btScalar(1.));
                //CollisionShape colShape = new CylinderShape(new IndexedVector3(1f, 1, 1f));
                m_collisionShapes.Add(colShape);

                /// Create Dynamic Objects
                IndexedMatrix startTransform = IndexedMatrix.Identity;

                mass = 1f;

                //rigidbody is dynamic if and only if mass is non zero, otherwise static
                bool isDynamic = mass != 0f;

                IndexedVector3 localInertia = IndexedVector3.Zero;
                if (isDynamic)
                {
                    colShape.CalculateLocalInertia(mass, out localInertia);
                }
                float start_x = START_POS_X - ARRAY_SIZE_X / 2;
                float start_y = START_POS_Y;
                float start_z = START_POS_Z - ARRAY_SIZE_Z / 2;

                for (int k = 0; k < ARRAY_SIZE_Y; k++)
                {
                    for (int i = 0; i < ARRAY_SIZE_X; i++)
                    {
                        for (int j = 0; j < ARRAY_SIZE_Z; j++)
                        {
                            startTransform._origin = (new IndexedVector3(2.0f * i + start_x, 20 + 2.0f * k + start_y, 2.0f * j + start_z) * SCALING);

                            //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
                            DefaultMotionState myMotionState = new DefaultMotionState(startTransform, IndexedMatrix.Identity);
                            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, colShape, localInertia);
                            RigidBody body = new RigidBody(rbInfo);
                            //body.setContactProcessingThreshold(colShape.getContactBreakingThreshold());
                            body.SetActivationState(ActivationState.ISLAND_SLEEPING);

                            m_dynamicsWorld.AddRigidBody(body);
                            body.SetActivationState(ActivationState.ISLAND_SLEEPING);
                            body.SetUserPointer(String.Format("Box X{0} Y{1} Z{2}", k, i, j));
                        }
                    }
                }
            }

            ClientResetScene();

        }

        public override void ClientMoveAndDisplay(GameTime gameTime)
        {
	        // OPTIONAL: We simply move our ghost objects around (without rotating them.....)==============
	        if (m_ghostObject != null || m_pairCachingGhostObject != null)	
            {
                rad+=0.005f;	// Bad (depends on PC speed)
		        float  sinRad = (float)Math.Sin(rad);
		        float  cosRad = (float)Math.Cos(rad);
		        if (m_ghostObject != null)	
                {
                    IndexedMatrix im = IndexedMatrix.CreateFromQuaternion(quatDeg45Y);
                    im._origin = new IndexedVector3(15*cosRad,5,-15*sinRad);
			        m_ghostObject.SetWorldTransform(ref im);
		        }
		        if (m_pairCachingGhostObject != null)	
                {
                    IndexedMatrix im = IndexedMatrix.CreateFromQuaternion(quatDeg45Y);
                    im._origin = new IndexedVector3(-15*cosRad,7,15*sinRad);
			        m_pairCachingGhostObject.SetWorldTransform(ref im);
		        }	
	        }



            base.ClientMoveAndDisplay(gameTime);
	        // NEW => Retrives the content from all ghost objects in the world and call ProcessObectsInsideGhostObjects(...) for each ====		
	        if (m_dynamicsWorld != null)
	        {
                ObjectArray < CollisionObject > objsInsidePairCachingGhostObject = new ObjectArray<CollisionObject>();	// We might want this to be a member variable...							
		        ObjectArray < CollisionObject >  pObjsInsideGhostObject = null;		// We will store a reference of the current array in this pointer
		        ObjectArray < CollisionObject > objs  = m_dynamicsWorld.GetCollisionObjectArray();
		        for (int i=0,sz=objs.Count;i<sz;i++)	
                {
			        CollisionObject o = objs[i];
			        GhostObject go = GhostObject.Upcast(o);
			        if (go != null)	
                    {
				        objsInsidePairCachingGhostObject.Resize(0);
				        PairCachingGhostObject pgo = go as PairCachingGhostObject;	// No upcast functionality...
				        if (pgo != null)	
                        {
					        GetCollidingObjectsInsidePairCachingGhostObject((DiscreteDynamicsWorld)m_dynamicsWorld,pgo,objsInsidePairCachingGhostObject);
					        pObjsInsideGhostObject = objsInsidePairCachingGhostObject;
				        }
				        else 
                        {
					        pObjsInsideGhostObject = go.GetOverlappingPairs();	// It's better not to try and copy the whole array, but to keep a reference to it!
					        // Side Note: btAlignedObjectArray < btCollisionObject* > objs = go.getOverlappingPairs(); (at the moment) makes my program crash on my system...
					        // Nevermind, that was the wrong way of doing it: btAlignedObjectArray < btCollisionObject* >& objs = go.getOverlappingPairs(); is much better.
				        }	
				        // Here pObjsInsideGhostObject should be valid.
				
				        ProcessObectsInsideGhostObjects(pObjsInsideGhostObject,pgo != null);
			        }
		
		        }	
	        }

        }




        // This static method is useful for the demo only. It's called by "void GhostObjectsDemo::clientMoveAndDisplay()".
        // Basically we must find a way to "display" the objects inside ghost objects to the user.
        // We choose to apply a vertical impulse to them (just because it's easier).
        // (The strength of the impulse is different depending on the type of ghost object)
        static void ProcessObectsInsideGhostObjects(ObjectArray<CollisionObject> objs, bool isPairCachingGhostObject)
        {
            foreach (CollisionObject co in objs)
            {
                RigidBody b = RigidBody.Upcast(co);
                if (b != null)
                {
                    b.Activate();
                    IndexedVector3 impulse = isPairCachingGhostObject ? new IndexedVector3(0, 0.5f, 0) : new IndexedVector3(0, 0.25f, 0);
                    b.ApplyCentralImpulse(ref impulse);
                }
            }
        }




        // Portable static method: prerequisite call: m_dynamicsWorld.getBroadphase().getOverlappingPairCache().setInternalGhostPairCallback(new btGhostPairCallback()); 
        public static void GetCollidingObjectsInsidePairCachingGhostObject(DiscreteDynamicsWorld m_dynamicsWorld, PairCachingGhostObject m_pairCachingGhostObject, ObjectArray<CollisionObject> collisionArrayOut)
        {
            bool addOnlyObjectsWithNegativeDistance = true;	// With "false" things don't change much, and the code is a bit faster and cleaner...


            collisionArrayOut.Resize(0);
            if (m_pairCachingGhostObject == null || m_dynamicsWorld == null) return;

            //#define USE_PLAIN_COLLISION_WORLD // We dispatch all collision pairs of the ghost object every step (slow)
#if USE_PLAIN_COLLISION_WORLD
	//======================================================================================================
	// I thought this line was no longer needed, but it seems to be necessary (and I believe it's an expensive call):
	m_dynamicsWorld.getDispatcher().dispatchAllCollisionPairs(m_pairCachingGhostObject.getOverlappingPairCache(), m_dynamicsWorld.getDispatchInfo(), m_dynamicsWorld.getDispatcher());
	// Maybe the call can be automatically triggered by some other Bullet call (I'm almost sure I could comment it out in another demo I made long ago...)
	// So by now the general rule is: in real projects, simply comment it out and see if it works!
	//======================================================================================================
	// UPDATE: in dynamic worlds, the line above can be commented out and the broadphase pair can be retrieved through the call to findPair(...) below.
	// In collision worlds probably the above line is needed only if collision detection for all the bodies hasn't been made... This is something
	// I'm still not sure of... the general rule is to try to comment out the line above and try to use findPair(...) and see if it works whenever possible....
	//======================================================================================================
#endif //USE_PLAIN_COLLISION_WORLD

            ObjectArray<BroadphasePair> collisionPairs = m_pairCachingGhostObject.GetOverlappingPairCache().GetOverlappingPairArray();
            int numObjects = collisionPairs.Count;
            ObjectArray<PersistentManifold> m_manifoldArray = new ObjectArray<PersistentManifold>();
            bool added;
            for (int i = 0; i < numObjects; i++)
            {
                m_manifoldArray.Resize(0);

#if USE_PLAIN_COLLISION_WORLD
		const btBroadphasePair& collisionPair = collisionPairs[i];
		if (collisionPair.m_algorithm) collisionPair.m_algorithm.getAllContactManifolds(m_manifoldArray);
		else {	// THIS SHOULD NEVER HAPPEN, AND IF IT DOES, PLEASE RE-ENABLE the "call" a few lines above...
			printf("No collisionPair.m_algorithm - probably m_dynamicsWorld.getDispatcher().dispatchAllCollisionPairs(...) must be missing.\n");	
		}	
#else // USE_PLAIN_COLLISION_WORLD
                BroadphasePair cPair = collisionPairs[i];
                //unless we manually perform collision detection on this pair, the contacts are in the dynamics world paircache:
                BroadphasePair collisionPair = m_dynamicsWorld.GetPairCache().FindPair(cPair.m_pProxy0, cPair.m_pProxy1);
                if (collisionPair == null)
                {
                    continue;

                }
                if (collisionPair.m_algorithm != null)
                {
                    collisionPair.m_algorithm.GetAllContactManifolds(m_manifoldArray);
                }
                else
                {	// THIS SHOULD NEVER HAPPEN, AND IF IT DOES, PLEASE RE-ENABLE the "call" a few lines above...
                    //printf("No collisionPair.m_algorithm - probably m_dynamicsWorld.getDispatcher().dispatchAllCollisionPairs(...) must be missing.\n");	
                }
#endif //USE_PLAIN_COLLISION_WORLD

                added = false;
                for (int j = 0; j < m_manifoldArray.Count; j++)
                {
                    PersistentManifold manifold = m_manifoldArray[j];
                    // Here we are in the narrowphase, but can happen that manifold.getNumContacts()==0:
                    if (addOnlyObjectsWithNegativeDistance)
                    {
                        for (int p = 0, numContacts = manifold.GetNumContacts(); p < numContacts; p++)
                        {
                            ManifoldPoint pt = manifold.GetContactPoint(p);
                            if (pt.GetDistance() < 0.0)
                            {
                                // How can I be sure that the colObjs are all distinct ? I use the "added" flag.
                                collisionArrayOut.Add((CollisionObject)(manifold.GetBody0() == m_pairCachingGhostObject ? manifold.GetBody1() : manifold.GetBody0()));
                                added = true;
                                break;
                            }
                        }
                        if (added)
                        {
                            break;
                        }
                    }
                    else if (manifold.GetNumContacts() > 0)
                    {
                        collisionArrayOut.Add((CollisionObject)(manifold.GetBody0() == m_pairCachingGhostObject ? manifold.GetBody1() : manifold.GetBody0()));
                        break;
                    }
                }
            }
        }




        static void Main(string[] args)
        {
            using (GhostObjectDemo game = new GhostObjectDemo())
            {
                game.Run();
            }
        }
        // test - just 8 objects.
        public const int ARRAY_SIZE_X = 3;
        public const int ARRAY_SIZE_Y = 3;
        public const int ARRAY_SIZE_Z = 3;

        //maximum number of objects (and allow user to shoot additional boxes)
        public const int MAX_PROXIES = (ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z + 1024);

        ///scaling of the objects (0.1 = 20 centimeter boxes )
        public const float SCALING = 1.5f;

        public const int START_POS_X = -5;
        public const int START_POS_Y = -5;
        public const int START_POS_Z = -3;

        GhostPairCallback m_ghostPairCallback = null;				// Needed once to enable ghost objects inside Bullet
        GhostObject m_ghostObject = null;							// simple aabb ghost object (keeps track of the objects whose aabbs intersect its own collision shape aabb: this is called "broadphase stage collision detection")
        PairCachingGhostObject m_pairCachingGhostObject = null;		// full shape ghost object (keeps track of the objects whose collision shape intersect its own collision shape: this is called "narrowphase stage collision detection")
        // "Portable" method; prerequisite call: m_dynamicsWorld.getBroadphase().getOverlappingPairCache().setInternalGhostPairCallback(m_ghostPairCallback); 
        //static void GetCollidingObjectsInsidePairCachingGhostObject(btDiscreteDynamicsWorld* m_dynamicsWorld,btPairCachingGhostObject* m_pairCachingGhostObject,btAlignedObjectArray < btCollisionObject* >& collisionArrayOut);

        private Quaternion quatDeg45Y = Quaternion.CreateFromAxisAngle(new Vector3(0, 1, 0), MathUtil.SIMD_HALF_PI * 0.5f);

        float rad = 0;

    }
}
