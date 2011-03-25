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
using BulletXNA.BullettCollision.CollisionShapes;
using BulletXNA.BulletCollision.CollisionDispatch;
using BulletXNA.BulletCollision.BroadphaseCollision;
using BulletXNA.BulletDynamics.ConstraintSolver;
using Microsoft.Xna.Framework;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.BulletDynamics.Dynamics;
using BulletXNA.BullettDynamics.Dynamics;
using BulletXNA.BullettCollision.CollisionDispatch;
using BulletXNA;
using System.IO;

namespace BulletXNADemos.Demos
{
    public class BasicDemo : DemoApplication
    {
        public BasicDemo()
        {
        }
        
        public override void InitializeDemo()
        {
            SetCameraDistance(SCALING * 50f);

			//string filename = @"C:\users\man\bullett\xna-basic-output.txt";
			//FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
			//BulletGlobals.g_streamWriter = new StreamWriter(filestream);

	        ///collision configuration contains default setup for memory, collision setup
	        m_collisionConfiguration = new DefaultCollisionConfiguration();

	        ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	        m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);

	        m_broadphase = new DbvtBroadphase();
            IOverlappingPairCache pairCache = null;
            //pairCache = new SortedOverlappingPairCache();

            m_broadphase = new SimpleBroadphase(1000, pairCache);

	        ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	        SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
            m_constraintSolver = sol;

            m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

            Vector3 gravity = new Vector3(0, -10, 0);
	        m_dynamicsWorld.SetGravity(ref gravity);

	        ///create a few basic rigid bodies
            Vector3 halfExtents = new Vector3(50, 50, 50);
            //Vector3 halfExtents = new Vector3(10, 10, 10);
            CollisionShape groundShape = new BoxShape(ref halfExtents);
            //CollisionShape groundShape = new StaticPlaneShape(Vector3.Up, 50);
        	
	        m_collisionShapes.Add(groundShape);

            Matrix groundTransform = Matrix.CreateTranslation(new Vector3(0, -50, 0));
            //Matrix groundTransform = Matrix.CreateTranslation(new Vector3(0,-10,0));
	        float mass = 0f;
            LocalCreateRigidBody(mass, ref groundTransform, groundShape);
	        {
		        //create a few dynamic rigidbodies
                CollisionShape colShape = new BoxShape(new Vector3(SCALING, SCALING, SCALING));
		        //btCollisionShape* colShape = new btSphereShape(btScalar(1.));
                //CollisionShape colShape = new CylinderShape(new Vector3(1f, 1, 1f));
		        m_collisionShapes.Add(colShape);

		        /// Create Dynamic Objects
		        Matrix startTransform = Matrix.Identity;

		        mass = 1f;

		        //rigidbody is dynamic if and only if mass is non zero, otherwise static
		        bool isDynamic = mass != 0f;

		        Vector3 localInertia = Vector3.Zero;
		        if (isDynamic)
                {
			        colShape.CalculateLocalInertia(mass,ref localInertia);
                }
		        float start_x = START_POS_X - ARRAY_SIZE_X/2;
		        float start_y = START_POS_Y;
		        float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		        for (int k=0;k<ARRAY_SIZE_Y;k++)
		        {
			        for (int i=0;i<ARRAY_SIZE_X;i++)
			        {
				        for(int j = 0;j<ARRAY_SIZE_Z;j++)
				        {
                            startTransform.Translation = (new Vector3(2.0f * i + start_x, 20 + 2.0f * k + start_y, 2.0f * j + start_z) * SCALING);

                            //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
                            DefaultMotionState myMotionState = new DefaultMotionState(startTransform, Matrix.Identity);
                            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, colShape, localInertia);
                            RigidBody body = new RigidBody(rbInfo);
                            //body->setContactProcessingThreshold(colShape->getContactBreakingThreshold());
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

        // test - just 8 objects.
        public const int ARRAY_SIZE_X = 2;
        public const int ARRAY_SIZE_Y = 4;
        public const int ARRAY_SIZE_Z = 2;

        //maximum number of objects (and allow user to shoot additional boxes)
        public const int MAX_PROXIES = (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024);

        ///scaling of the objects (0.1 = 20 centimeter boxes )
        public const float SCALING = 1f;

        public const int START_POS_X  = -5;
        public const int START_POS_Y  = -5;
        public const int START_POS_Z  = -3;

        static void Main(string[] args)
        {
            using (BasicDemo game = new BasicDemo())
            {
                game.Run();
            }
        }

    }
}
