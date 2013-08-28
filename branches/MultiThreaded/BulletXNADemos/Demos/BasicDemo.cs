
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
using BulletXNA;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using BulletXNA.LinearMath;
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
            //maxiterations = 10;
            SetCameraDistance(SCALING * 50f);

            //string filename = @"E:\users\man\bullet\xna-basic-output-1.txt";
            //FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
            //BulletGlobals.g_streamWriter = new StreamWriter(filestream);

	        ///collision configuration contains default setup for memory, collision setup
	        m_collisionConfiguration = new DefaultCollisionConfiguration();

	        ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	        m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);


            IndexedVector3 worldMin = new IndexedVector3(-1000, -1000, -1000);
            IndexedVector3 worldMax = -worldMin;
            m_broadphase = new AxisSweep3Internal(ref worldMin, ref worldMax, 0xfffe, 0xffff, 16384, null, false, m_dispatcher);

            //m_broadphase = new DbvtBroadphase();
            IOverlappingPairCache pairCache = null;
            //pairCache = new SortedOverlappingPairCache();

            //m_broadphase = new SimpleBroadphase(1000, pairCache);

	        ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	        SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
            m_constraintSolver = sol;

            m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

            IndexedVector3 gravity = new IndexedVector3(0, -10, 0);
	        m_dynamicsWorld.SetGravity(ref gravity);

	        ///create a few basic rigid bodies
            IndexedVector3 halfExtents = new IndexedVector3(50, 50, 50);
            //IndexedVector3 halfExtents = new IndexedVector3(10, 10, 10);
            CollisionShape groundShape = new BoxShape(ref halfExtents);
            //CollisionShape groundShape = new StaticPlaneShape(new IndexedVector3(0,1,0), 50);
        	
	        m_collisionShapes.Add(groundShape);

            IndexedMatrix groundTransform = IndexedMatrix.CreateTranslation(new IndexedVector3(0, -50, 0));
            //IndexedMatrix groundTransform = IndexedMatrix.CreateTranslation(new IndexedVector3(0,-10,0));
	        float mass = 0f;
            LocalCreateRigidBody(mass, ref groundTransform, groundShape);
	        {
		        //create a few dynamic rigidbodies
                CollisionShape colShape = new BoxShape(new IndexedVector3(SCALING, SCALING, SCALING));
                //CollisionShape colShape = BuildCorner();
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
		        float start_x = START_POS_X - ARRAY_SIZE_X/2;
		        float start_y = START_POS_Y;
		        float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		        for (int k=0;k<ARRAY_SIZE_Y;k++)
		        {
			        for (int i=0;i<ARRAY_SIZE_X;i++)
			        {
				        for(int j = 0;j<ARRAY_SIZE_Z;j++)
				        {
                            startTransform._origin = (new IndexedVector3(2.0f * i + start_x, 20 + 2.0f * k + start_y, 2.0f * j + start_z) * SCALING);

                            //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
                            DefaultMotionState myMotionState = new DefaultMotionState(startTransform, IndexedMatrix.Identity);
                            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, colShape, localInertia);
                            RigidBody body = new RigidBody(rbInfo);
                            //body->setContactProcessingThreshold(colShape->getContactBreakingThreshold());
                            //body.SetActivationState(ActivationState.ISLAND_SLEEPING);

                            m_dynamicsWorld.AddRigidBody(body);
                            //body.SetActivationState(ActivationState.ISLAND_SLEEPING);
                            body.SetUserPointer(String.Format("Box X{0} Y{1} Z{2}", k, i, j));
				        }
			        }
		        }
	        }

            //ClientResetScene();
        }


        public CollisionShape BuildCorner()
        {
            // slope.
            IndexedVector3[] vertices = new IndexedVector3[]{new IndexedVector3(0,0,0),new IndexedVector3(1,0,0),new IndexedVector3(0,0,1),new IndexedVector3(1,0,1),
                            new IndexedVector3(0,1,0),new IndexedVector3(1,1,0),new IndexedVector3(0,1,1),new IndexedVector3(1,1,1)};



            //int[] indices = new int[] { 0, 4, 5, 4, 6, 7, 7, 5, 4, 0, 4, 6, 6, 2, 0, 2, 6, 7, 4,5,0,2,2,7,5};
            //int[] indices = new int[] { 0, 4, 5, 4, 6, 7, 7, 5, 4,  6, 4,0,0,2,6, 7, 6, 2, 4, 5, 0, 2, 2, 7, 5 };
            int[] indices = new int[] { 1,4,5,
                1,5,7,
                7,3,1,
                3,7,6,
                7,5,4,
                4,6,7,
                4,1,3,
                3,6,4
            };

            int vertStride = 1;
            int indexStride = 3;

            ObjectArray<IndexedVector3> vertexArray = new ObjectArray<IndexedVector3>();
            for (int i = 0; i < vertices.Length; ++i)
            {
                vertexArray.Add(vertices[i]);
            }

            ObjectArray<int> intArray = new ObjectArray<int>();
            for (int i = 0; i < indices.Length; ++i)
            {
                intArray.Add(indices[i]);
            }
            TriangleIndexVertexArray indexVertexArray = new TriangleIndexVertexArray(indices.Length/3, intArray, indexStride, vertexArray.Count, vertexArray, vertStride);
            TriangleMeshShape triangleMesh = new TriangleMeshShape(indexVertexArray);
            //TriangleMeshShape triangleMesh = new BvhTriangleMeshShape(indexVertexArray,true,true);
            return triangleMesh;




        }


        // test - just 8 objects.
        public const int ARRAY_SIZE_X = 3;
        public const int ARRAY_SIZE_Y = 3;
        public const int ARRAY_SIZE_Z = 3 ;

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
