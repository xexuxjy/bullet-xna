/*

C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

using System.IO;
using BulletXNA;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace BulletXNADemos.Demos
{
    public class RagDollDemo : DemoApplication
    {

        public const float CONSTRAINT_DEBUG_SIZE = 0.2f;
        public class RagDoll
        {
	        public enum BODYPART
	        {
		        PELVIS = 0,
		        SPINE,
		        HEAD,

		        LEFT_UPPER_LEG,
		        LEFT_LOWER_LEG,

		        RIGHT_UPPER_LEG,
		        RIGHT_LOWER_LEG,

		        LEFT_UPPER_ARM,
		        LEFT_LOWER_ARM,

		        RIGHT_UPPER_ARM,
		        RIGHT_LOWER_ARM,

		        COUNT
	        };

	        public enum JOINT
	        {
		        PELVIS_SPINE = 0,
		        SPINE_HEAD,

		        LEFT_HIP,
		        LEFT_KNEE,

		        RIGHT_HIP,
		        RIGHT_KNEE,

		        LEFT_SHOULDER,
		        LEFT_ELBOW,

		        RIGHT_SHOULDER,
		        RIGHT_ELBOW,

		        COUNT
	        };

	        DynamicsWorld m_ownerWorld;
            RagDollDemo m_ragDollDemo;
            CollisionShape[] m_shapes = new CollisionShape[(int)BODYPART.COUNT];
            RigidBody[] m_bodies = new RigidBody[(int)BODYPART.COUNT];
            TypedConstraint[] m_joints = new TypedConstraint[(int)JOINT.COUNT];

            public RagDoll()
            {
            }

        	public RagDoll (RagDollDemo ragDollDemo,DynamicsWorld ownerWorld, ref IndexedVector3 positionOffset,StreamWriter streamWriter)
		
	        {
                m_ownerWorld = ownerWorld;
                m_ragDollDemo = ragDollDemo;
		        // Setup the geometry
                m_shapes[(int)BODYPART.PELVIS] = new CapsuleShape(0.15f, 0.20f);
                m_shapes[(int)BODYPART.SPINE] = new CapsuleShape(0.15f, 0.28f);
                m_shapes[(int)BODYPART.HEAD] = new CapsuleShape(0.10f, 0.05f);
                m_shapes[(int)BODYPART.LEFT_UPPER_LEG] = new CapsuleShape(0.07f, 0.45f);
                m_shapes[(int)BODYPART.LEFT_LOWER_LEG] = new CapsuleShape(0.05f, 0.37f);
                m_shapes[(int)BODYPART.RIGHT_UPPER_LEG] = new CapsuleShape(0.07f, 0.45f);
                m_shapes[(int)BODYPART.RIGHT_LOWER_LEG] = new CapsuleShape(0.05f, 0.37f);
                m_shapes[(int)BODYPART.LEFT_UPPER_ARM] = new CapsuleShape(0.05f, 0.33f);
                m_shapes[(int)BODYPART.LEFT_LOWER_ARM] = new CapsuleShape(0.04f, 0.25f);
                m_shapes[(int)BODYPART.RIGHT_UPPER_ARM] = new CapsuleShape(0.05f, 0.33f);
                m_shapes[(int)BODYPART.RIGHT_LOWER_ARM] = new CapsuleShape(0.04f, 0.25f);

		        // Setup all the rigid bodies
		        IndexedMatrix offset = IndexedMatrix.CreateTranslation(positionOffset);

		        IndexedMatrix transform = IndexedMatrix.CreateTranslation(new IndexedVector3(0,1,0));

				IndexedMatrix adjusted = offset*transform;

                m_bodies[(int)BODYPART.PELVIS] = m_ragDollDemo.LocalCreateRigidBody(1f, adjusted, m_shapes[(int)BODYPART.PELVIS],true);
				m_bodies[(int)BODYPART.PELVIS].SetUserPointer("PELVIS");
                transform = IndexedMatrix.CreateTranslation(new IndexedVector3(0,1.2f,0));
				adjusted = offset * transform;
				m_bodies[(int)BODYPART.SPINE] = m_ragDollDemo.LocalCreateRigidBody(1f, adjusted, m_shapes[(int)BODYPART.SPINE], true);
				m_bodies[(int)BODYPART.SPINE].SetUserPointer("SPINE");

                transform = IndexedMatrix.CreateTranslation(new IndexedVector3(0,1.6f,0));
				adjusted = offset * transform;
				m_bodies[(int)BODYPART.HEAD] = m_ragDollDemo.LocalCreateRigidBody(1f, adjusted, m_shapes[(int)BODYPART.HEAD], true);
				m_bodies[(int)BODYPART.HEAD].SetUserPointer("HEAD");

				transform = IndexedMatrix.CreateTranslation(new IndexedVector3(-0.18f, 0.65f, 0));
				adjusted = offset * transform
                    ;
				m_bodies[(int)BODYPART.LEFT_UPPER_LEG] = m_ragDollDemo.LocalCreateRigidBody(1f, adjusted, m_shapes[(int)BODYPART.LEFT_UPPER_LEG], true);
				m_bodies[(int)BODYPART.LEFT_UPPER_LEG].SetUserPointer("LEFTUPPERLEG");

				transform = IndexedMatrix.CreateTranslation(new IndexedVector3(-0.18f, 0.2f, 0));
				adjusted = offset *transform;
				m_bodies[(int)BODYPART.LEFT_LOWER_LEG] = m_ragDollDemo.LocalCreateRigidBody(1f, adjusted, m_shapes[(int)BODYPART.LEFT_LOWER_LEG], true);
				m_bodies[(int)BODYPART.LEFT_LOWER_LEG].SetUserPointer("LEFTLOWERLEG");

				transform = IndexedMatrix.CreateTranslation(new IndexedVector3(0.18f, 0.65f, 0));
				adjusted = offset * transform;
				m_bodies[(int)BODYPART.RIGHT_UPPER_LEG] = m_ragDollDemo.LocalCreateRigidBody(1f, adjusted, m_shapes[(int)BODYPART.RIGHT_UPPER_LEG], true);
				m_bodies[(int)BODYPART.RIGHT_UPPER_LEG].SetUserPointer("RIGHTUPPERLEG");

				transform = IndexedMatrix.CreateTranslation(new IndexedVector3(0.18f, 0.2f, 0));
				adjusted = offset*transform;
				m_bodies[(int)BODYPART.RIGHT_LOWER_LEG] = m_ragDollDemo.LocalCreateRigidBody(1f, adjusted, m_shapes[(int)BODYPART.RIGHT_LOWER_LEG], true);
				m_bodies[(int)BODYPART.RIGHT_LOWER_LEG].SetUserPointer("RIGHTLOWERLEG");

				transform = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
				transform._origin = new IndexedVector3(-0.35f, 1.45f, 0);
				adjusted = offset *transform;
				m_bodies[(int)BODYPART.LEFT_UPPER_ARM] = m_ragDollDemo.LocalCreateRigidBody(1f, adjusted, m_shapes[(int)BODYPART.LEFT_UPPER_ARM], true);
				m_bodies[(int)BODYPART.LEFT_UPPER_ARM].SetUserPointer("LEFTUPPERARM");

				transform = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
				transform._origin = new IndexedVector3(-0.7f, 1.45f, 0);
				adjusted = offset * transform;
				m_bodies[(int)BODYPART.LEFT_LOWER_ARM] = m_ragDollDemo.LocalCreateRigidBody(1f, adjusted, m_shapes[(int)BODYPART.LEFT_LOWER_ARM], true);
				m_bodies[(int)BODYPART.LEFT_LOWER_ARM].SetUserPointer("LEFTLOWERARM");

				transform = MathUtil.SetEulerZYX(0, 0, -MathUtil.SIMD_HALF_PI);
				transform._origin = new IndexedVector3(0.35f, 1.45f, 0);
				adjusted = offset * transform;
				m_bodies[(int)BODYPART.RIGHT_UPPER_ARM] = m_ragDollDemo.LocalCreateRigidBody(1f, adjusted, m_shapes[(int)BODYPART.RIGHT_UPPER_ARM], true);
				m_bodies[(int)BODYPART.RIGHT_UPPER_ARM].SetUserPointer("RIGHTUPPERARM");

				transform = MathUtil.SetEulerZYX(0, 0, -MathUtil.SIMD_HALF_PI);
				transform._origin = new IndexedVector3(0.7f, 1.45f, 0);
				adjusted = offset * transform;
				m_bodies[(int)BODYPART.RIGHT_LOWER_ARM] = m_ragDollDemo.LocalCreateRigidBody(1f, adjusted, m_shapes[(int)BODYPART.RIGHT_LOWER_ARM], true);
				m_bodies[(int)BODYPART.RIGHT_LOWER_ARM].SetUserPointer("RIGHTLOWERARM");

		        // Setup some damping on the m_bodies
		        for (int i = 0; i < (int)BODYPART.COUNT; ++i)
		        {
					if (m_bodies[i] != null)
					{
                        //m_bodies[i].SetDamping(0.05f, 0.85f);
                        m_bodies[i].SetDamping(0.5f, 0.85f);
						m_bodies[i].SetDeactivationTime(0.8f);
						m_bodies[i].SetSleepingThresholds(1.6f, 2.5f);
					}
                }

		        // Now setup the constraints
		        HingeConstraint hingeC;
		        ConeTwistConstraint coneC;

		        IndexedMatrix localA = IndexedMatrix.Identity;
                IndexedMatrix localB = IndexedMatrix.Identity;

                localA = MathUtil.SetEulerZYX(0, MathUtil.SIMD_HALF_PI, 0);
                localA._origin = new IndexedVector3(0.0f, 0.15f, 0.0f);
                localB = MathUtil.SetEulerZYX(0, MathUtil.SIMD_HALF_PI, 0);
                localB._origin = new IndexedVector3(0.0f, -0.15f, 0.0f);
                hingeC = new HingeConstraint(m_bodies[(int)BODYPART.PELVIS], m_bodies[(int)BODYPART.SPINE], ref localA, ref localB);
		        hingeC.SetLimit(-MathUtil.SIMD_QUARTER_PI, MathUtil.SIMD_HALF_PI);
                m_joints[(int)JOINT.PELVIS_SPINE] = hingeC;
                m_joints[(int)JOINT.PELVIS_SPINE].m_debugName = "PELVIS_SPINE";
		        hingeC.SetDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

                m_ownerWorld.AddConstraint(m_joints[(int)JOINT.PELVIS_SPINE], true);


                localA = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
                localA._origin = new IndexedVector3(0.0f, 0.30f, 0.0f);
                localB = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
                localB._origin = new IndexedVector3(0.0f, -0.14f, 0.0f);
                coneC = new ConeTwistConstraint(m_bodies[(int)BODYPART.SPINE], m_bodies[(int)BODYPART.HEAD], ref localA, ref localB);
		        coneC.SetLimit(MathUtil.SIMD_QUARTER_PI, MathUtil.SIMD_QUARTER_PI, MathUtil.SIMD_HALF_PI);
                m_joints[(int)JOINT.SPINE_HEAD] = coneC;
                m_joints[(int)JOINT.SPINE_HEAD].m_debugName = "SPINE_HEAD";

		        coneC.SetDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

                m_ownerWorld.AddConstraint(m_joints[(int)JOINT.SPINE_HEAD], true);


				localA = IndexedMatrix.Identity;
				localB = IndexedMatrix.Identity;
				localA = MathUtil.SetEulerZYX(0, 0, -MathUtil.SIMD_QUARTER_PI * 5);
				localA._origin = new IndexedVector3(-0.18f, -0.10f, 0.0f);
				localB = MathUtil.SetEulerZYX(0, 0, -MathUtil.SIMD_QUARTER_PI * 5);
				localB._origin = new IndexedVector3(0.0f, 0.225f, 0.0f);
				coneC = new ConeTwistConstraint(m_bodies[(int)BODYPART.PELVIS], m_bodies[(int)BODYPART.LEFT_UPPER_LEG], ref localA, ref localB);
				coneC.SetLimit(MathUtil.SIMD_QUARTER_PI, MathUtil.SIMD_QUARTER_PI, 0);
				m_joints[(int)JOINT.LEFT_HIP] = coneC;
                m_joints[(int)JOINT.LEFT_HIP].m_debugName = "LEFT_HIP";

				coneC.SetDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

				m_ownerWorld.AddConstraint(m_joints[(int)JOINT.LEFT_HIP], true);

				localA = MathUtil.SetEulerZYX(0f, MathUtil.SIMD_HALF_PI, 0f);
				localA._origin = new IndexedVector3(0.0f, -0.225f, 0.0f);
				localB = MathUtil.SetEulerZYX(0, MathUtil.SIMD_HALF_PI, 0);
				localB._origin = new IndexedVector3(0.0f, 0.185f, 0.0f);
				hingeC = new HingeConstraint(m_bodies[(int)BODYPART.LEFT_UPPER_LEG], m_bodies[(int)BODYPART.LEFT_LOWER_LEG], ref localA, ref localB);
				hingeC.SetLimit(0, MathUtil.SIMD_HALF_PI);
				m_joints[(int)JOINT.LEFT_KNEE] = hingeC;
                m_joints[(int)JOINT.LEFT_KNEE].m_debugName = "LEFT_KNEE";

				hingeC.SetDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

				m_ownerWorld.AddConstraint(m_joints[(int)JOINT.LEFT_KNEE], true);


				localA = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_QUARTER_PI);
				localA._origin = new IndexedVector3(0.18f, -0.10f, 0.0f);
				localB = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_QUARTER_PI);
				localB._origin = new IndexedVector3(0.0f, 0.225f, 0.0f);
				coneC = new ConeTwistConstraint(m_bodies[(int)BODYPART.PELVIS], m_bodies[(int)BODYPART.RIGHT_UPPER_LEG], ref localA, ref localB);
				coneC.SetLimit(MathUtil.SIMD_QUARTER_PI, MathUtil.SIMD_QUARTER_PI, 0);
				m_joints[(int)JOINT.RIGHT_HIP] = coneC;
                m_joints[(int)JOINT.RIGHT_HIP].m_debugName = "RIGHT_HIP";

				coneC.SetDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

				m_ownerWorld.AddConstraint(m_joints[(int)JOINT.RIGHT_HIP], true);

				localA = MathUtil.SetEulerZYX(0, MathUtil.SIMD_HALF_PI, 0);
				localA._origin = new IndexedVector3(0.0f, -0.225f, 0.0f);
				localB = MathUtil.SetEulerZYX(0, MathUtil.SIMD_HALF_PI, 0);
				localB._origin = new IndexedVector3(0.0f, 0.185f, 0.0f);
				hingeC = new HingeConstraint(m_bodies[(int)BODYPART.RIGHT_UPPER_LEG], m_bodies[(int)BODYPART.RIGHT_LOWER_LEG], ref localA, ref localB);
				hingeC.SetLimit(0, MathUtil.SIMD_HALF_PI);
				m_joints[(int)JOINT.RIGHT_KNEE] = hingeC;
                m_joints[(int)JOINT.RIGHT_KNEE].m_debugName = "RIGHT_KNEE";

				hingeC.SetDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

				m_ownerWorld.AddConstraint(m_joints[(int)JOINT.RIGHT_KNEE], true);


				localA = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_PI);
				localA._origin = new IndexedVector3(-0.2f, 0.15f, 0.0f);
				localB = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
				localB._origin = new IndexedVector3(0.0f, -0.18f, 0.0f);
				coneC = new ConeTwistConstraint(m_bodies[(int)BODYPART.SPINE], m_bodies[(int)BODYPART.LEFT_UPPER_ARM], ref localA, ref localB);
				coneC.SetLimit(MathUtil.SIMD_HALF_PI, MathUtil.SIMD_HALF_PI, 0);
				coneC.SetDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

				m_joints[(int)JOINT.LEFT_SHOULDER] = coneC;
                m_joints[(int)JOINT.LEFT_SHOULDER].m_debugName = "LEFT_SHOULDER";

				m_ownerWorld.AddConstraint(m_joints[(int)JOINT.LEFT_SHOULDER], true);

				localA = MathUtil.SetEulerZYX(0, MathUtil.SIMD_HALF_PI, 0);
				localA._origin = new IndexedVector3(0.0f, 0.18f, 0.0f);
				localB = MathUtil.SetEulerZYX(0, MathUtil.SIMD_HALF_PI, 0);
				localB._origin = new IndexedVector3(0.0f, -0.14f, 0.0f);
				hingeC = new HingeConstraint(m_bodies[(int)BODYPART.LEFT_UPPER_ARM], m_bodies[(int)BODYPART.LEFT_LOWER_ARM], ref localA, ref localB);
				//		hingeC.setLimit(-MathUtil.SIMD_HALF_PI), 0));
				hingeC.SetLimit(0, MathUtil.SIMD_HALF_PI);
				m_joints[(int)JOINT.LEFT_ELBOW] = hingeC;
                m_joints[(int)JOINT.LEFT_ELBOW].m_debugName = "LEFT_ELBOW";

				hingeC.SetDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

				m_ownerWorld.AddConstraint(m_joints[(int)JOINT.LEFT_ELBOW], true);

				localA = MathUtil.SetEulerZYX(0, 0, 0);
				localA._origin = new IndexedVector3(0.2f, 0.15f, 0.0f);
				localB = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
				localB._origin = new IndexedVector3(0.0f, -0.18f, 0.0f);
				coneC = new ConeTwistConstraint(m_bodies[(int)BODYPART.SPINE], m_bodies[(int)BODYPART.RIGHT_UPPER_ARM], ref localA, ref localB);
				coneC.SetLimit(MathUtil.SIMD_HALF_PI, MathUtil.SIMD_HALF_PI, 0);
				m_joints[(int)JOINT.RIGHT_SHOULDER] = coneC;
                m_joints[(int)JOINT.RIGHT_SHOULDER].m_debugName = "RIGHT_SHOULDER";

				coneC.SetDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

				m_ownerWorld.AddConstraint(m_joints[(int)JOINT.RIGHT_SHOULDER], true);

				localA = MathUtil.SetEulerZYX(0, MathUtil.SIMD_HALF_PI, 0);
				localA._origin = new IndexedVector3(0.0f, 0.18f, 0.0f);
				localB = MathUtil.SetEulerZYX(0, MathUtil.SIMD_HALF_PI, 0);
				localB._origin = new IndexedVector3(0.0f, -0.14f, 0.0f);
				hingeC = new HingeConstraint(m_bodies[(int)BODYPART.RIGHT_UPPER_ARM], m_bodies[(int)BODYPART.RIGHT_LOWER_ARM], ref localA, ref localB);
				//		hingeC.setLimit(-MathUtil.SIMD_HALF_PI), 0));
				hingeC.SetLimit(0, MathUtil.SIMD_HALF_PI);
				m_joints[(int)JOINT.RIGHT_ELBOW] = hingeC;
                m_joints[(int)JOINT.RIGHT_ELBOW].m_debugName = "RIGHT_ELBOW";

				hingeC.SetDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

                m_ownerWorld.AddConstraint(m_joints[(int)JOINT.RIGHT_ELBOW], true); 
				
				
	        }

	        public virtual void Cleanup()
	        {
		        int i;

		        // Remove all constraints
		        for ( i = 0; i < (int)JOINT.COUNT; ++i)
		        {
					if(m_joints[i] != null)
					{
						m_ownerWorld.RemoveConstraint(m_joints[i]);
						m_joints[i].Cleanup();
						m_joints[i] = null;
					}
		        }

		        // Remove all bodies and shapes
		        for ( i = 0; i < (int)BODYPART.COUNT; ++i)
		        {
					if(m_bodies[i] != null)
					{
						m_ownerWorld.RemoveRigidBody(m_bodies[i]);
        			
						//delete m_bodies[i].getMotionState();

						m_bodies[i].Cleanup(); 
						m_bodies[i] = null;
						m_shapes[i].Cleanup(); 
						m_shapes[i] = null;
					}
		        }
	        }
        }




        public override void InitializeDemo()

        {
	        // Setup the basic world

	        SetTexturing(true);
	        SetShadows(true);

	        SetCameraDistance(5.0f);

	        m_collisionConfiguration = new DefaultCollisionConfiguration();

	        m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);

	        IndexedVector3 worldAabbMin = new IndexedVector3(-10000,-10000,-10000);
	        IndexedVector3 worldAabbMax = new IndexedVector3(10000,10000,10000);
            m_broadphase = new AxisSweep3Internal(ref worldAabbMin, ref worldAabbMax, 0xfffe, 0xffff, 16384, null, true, m_dispatcher);
            //m_broadphase = new SimpleBroadphase(1000, null);
	        m_constraintSolver = new SequentialImpulseConstraintSolver();

            m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);
	        //m_dynamicsWorld.getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	        //m_dynamicsWorld.getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;



	        // Setup a big ground box
	        {
		        CollisionShape groundShape = new BoxShape(new IndexedVector3(200.0f,10.0f,200.0f));
		        m_collisionShapes.Add(groundShape);
		        IndexedMatrix groundTransform = IndexedMatrix.CreateTranslation(0,-10,0);

		        CollisionObject fixedGround = new CollisionObject();
		        fixedGround.SetCollisionShape(groundShape);
		        fixedGround.SetWorldTransform(ref groundTransform);
                fixedGround.SetUserPointer("Ground");
		        m_dynamicsWorld.AddCollisionObject(fixedGround);
	        }

	        // Spawn one ragdoll
	        IndexedVector3 startOffset = new IndexedVector3(1,0.5f,0);


            //string filename = @"c:\users\man\bullet\xna-ragdoll-constraints-output.txt";
            //FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
            //BulletGlobals.g_streamWriter = new StreamWriter(filestream);


			SpawnRagdoll(ref startOffset, BulletGlobals.g_streamWriter);
            //startOffset = new IndexedVector3(-1,0.5f,0);
            //spawnRagdoll(ref startOffset);

	        ClientResetScene();		
        }

        public void SpawnRagdoll(ref IndexedVector3 startOffset,StreamWriter streamWriter)
        {
	        RagDoll ragDoll = new RagDoll (this,m_dynamicsWorld, ref startOffset,streamWriter);
	        m_ragdolls.Add(ragDoll);
        }	

        public override void KeyboardCallback(Keys key,int x,int y,GameTime gameTime,bool released,ref KeyboardState newState,ref KeyboardState oldState)
        {
	        switch (key)
	        {
	        case Keys.E:
		        {
		        IndexedVector3 startOffset = new IndexedVector3(0,2,0);
		        SpawnRagdoll(ref startOffset,BulletGlobals.g_streamWriter);
		        break;
		        }
	        default:
		        base.KeyboardCallback(key, x, y,gameTime,released,ref newState,ref oldState);
                break;
	        }

        	
        }



        	public ObjectArray<RagDoll> m_ragdolls = new ObjectArray<RagDoll>();

            static void Main(string[] args)
            {
                using (RagDollDemo game = new RagDollDemo())
                {
                    game.Run();
                }
            }
    }


}
