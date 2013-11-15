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

using System.Collections.Generic;
using BulletXNA;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using BulletXNA.LinearMath;

namespace BulletXNADemos.Demos
{
    public class Box2DDemo : DemoApplication
    {
        public override void InitializeDemo()
        {
            SetTexturing(true);
            SetShadows(true);

            SetCameraDistance(SCALING*50.0f);
            m_cameraTargetPosition = IndexedVector3.Zero;

            //string filename = @"E:\users\man\bullet\xna-box2d-output.txt";
            //FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
            //BulletGlobals.g_streamWriter = new StreamWriter(filestream);


            ///collision configuration contains default setup for memory, collision setup
            m_collisionConfiguration = new DefaultCollisionConfiguration();
            //m_collisionConfiguration.setConvexConvexMultipointIterations();

            ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
            m_dispatcher = new	CollisionDispatcher(m_collisionConfiguration);

            VoronoiSimplexSolver simplex = new VoronoiSimplexSolver();
            MinkowskiPenetrationDepthSolver pdSolver = new MinkowskiPenetrationDepthSolver();

            CollisionAlgorithmCreateFunc convexAlgo2d = new Convex2dConvex2dCreateFunc(simplex, pdSolver);

			m_dispatcher.RegisterCollisionCreateFunc((int)BroadphaseNativeTypes.CONVEX_2D_SHAPE_PROXYTYPE, (int)BroadphaseNativeTypes.CONVEX_2D_SHAPE_PROXYTYPE, convexAlgo2d);
			m_dispatcher.RegisterCollisionCreateFunc((int)BroadphaseNativeTypes.BOX_2D_SHAPE_PROXYTYPE, (int)BroadphaseNativeTypes.CONVEX_2D_SHAPE_PROXYTYPE, convexAlgo2d);
			m_dispatcher.RegisterCollisionCreateFunc((int)BroadphaseNativeTypes.CONVEX_2D_SHAPE_PROXYTYPE, (int)BroadphaseNativeTypes.BOX_2D_SHAPE_PROXYTYPE, convexAlgo2d);
            m_dispatcher.RegisterCollisionCreateFunc((int)BroadphaseNativeTypes.BOX_2D_SHAPE_PROXYTYPE, (int)BroadphaseNativeTypes.BOX_2D_SHAPE_PROXYTYPE, new Box2dBox2dCreateFunc());

            //m_broadphase = new DbvtBroadphase();
            m_broadphase = new SimpleBroadphase(1000,null);

			///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
			m_constraintSolver = new SequentialImpulseConstraintSolver();

			m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);
			//m_dynamicsWorld.getSolverInfo().m_erp = 1.f;
			//m_dynamicsWorld.getSolverInfo().m_numIterations = 4;

			//m_dynamicsWorld.setGravity(new IndexedVector3(0,-10,0));

			///create a few basic rigid bodies
			CollisionShape groundShape = new BoxShape(new IndexedVector3(150,50,150));
		//	btCollisionShape* groundShape = new btStaticPlaneShape(IndexedVector3(0,1,0),50);
	
			m_collisionShapes.Add(groundShape);

			IndexedMatrix groundTransform = IndexedMatrix.CreateTranslation(0,-43,0);
			LocalCreateRigidBody(0,groundTransform,groundShape);

			{
				//create a few dynamic rigidbodies
				// Re-using the same collision is better for memory usage and performance

				float u= 1*SCALING-0.04f;
				IList<IndexedVector3> points = new List<IndexedVector3>();
                points.Add(new IndexedVector3(0, u, 0));
                points.Add(new IndexedVector3(-u,-u,0));
                points.Add(new IndexedVector3(u,-u,0));
				ConvexShape boxShape= new Convex2dShape(new BoxShape(new IndexedVector3(SCALING,SCALING,0.04f)));
				//btCollisionShape* colShape = new btBox2dShape(IndexedVector3(SCALING*1,SCALING*1,0.04));


				ConvexShape triangleShape= new Convex2dShape(new ConvexHullShape(points,3));
				IndexedVector3 extents = new IndexedVector3(SCALING, SCALING, 0.04f);
				ConvexShape cylinderShape= new Convex2dShape(new CylinderShapeZ(ref extents));
				

				//btUniformScalingShape* colShape = new btUniformScalingShape(convexColShape,1.f);
				boxShape.SetMargin(0.03f);
				//btCollisionShape* colShape = new btSphereShape(float(1.));
				m_collisionShapes.Add(boxShape);
				m_collisionShapes.Add(triangleShape);
				
				/// Create Dynamic Objects
				IndexedMatrix startTransform = IndexedMatrix.Identity;

				float mass = 1.0f;;

				//rigidbody is dynamic if and only if mass is non zero, otherwise static
				bool isDynamic = (mass != 0.9f);

				IndexedVector3 localInertia = IndexedVector3.Zero;
				if (isDynamic)
				{
					boxShape.CalculateLocalInertia(mass, out localInertia);
				}

		//		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		//		float start_y = START_POS_Y;
		//		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

				IndexedVector3 x = new IndexedVector3(-ARRAY_SIZE_X, 8f,-20f);
				//IndexedVector3 y = IndexedVector3.Zero;
				IndexedVector3 y = new IndexedVector3(0,0,0);
				IndexedVector3 deltaX = new IndexedVector3(SCALING*1, SCALING*2,0f);
				IndexedVector3 deltaY = new IndexedVector3(SCALING*2, 0.0f,0f);

				for (int i = 0; i < ARRAY_SIZE_X; ++i)
				{
					y = x;

					for (int  j = i; j < ARRAY_SIZE_Y; ++j)
					{
							startTransform._origin = (y-new IndexedVector3(-10,0,0));

					
							//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
							DefaultMotionState myMotionState = new DefaultMotionState(startTransform,IndexedMatrix.Identity);
							RigidBodyConstructionInfo rbInfo;
							switch (j%3)
							{
		#if true
							case 0:
                                    rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, boxShape, localInertia);
                                    //rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, triangleShape, localInertia);
								break;
							case 1:
								rbInfo = new RigidBodyConstructionInfo(mass,myMotionState,triangleShape,localInertia);
								break;
		#endif
							default:
								rbInfo = new RigidBodyConstructionInfo(mass,myMotionState,cylinderShape,localInertia);
                                break;
							}
							RigidBody body = new RigidBody(rbInfo);
							//body.setContactProcessingThreshold(colShape.getContactBreakingThreshold());
							body.SetActivationState(ActivationState.ISLAND_SLEEPING);
							body.SetLinearFactor(new IndexedVector3(1,1,0));
							body.SetAngularFactor(new IndexedVector3(0,0,1));

							m_dynamicsWorld.AddRigidBody(body);
							body.SetActivationState(ActivationState.ISLAND_SLEEPING);
                            //if (BulletGlobals.g_streamWriter != null)
                            //{
                            //    BulletGlobals.g_streamWriter.WriteLine("localCreateRigidBody [{0}] startTransform", body.m_debugBodyId);
                            //    MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, startTransform);
                            //    BulletGlobals.g_streamWriter.WriteLine("");
                            //}


					//	y += -0.8*deltaY;
						y += deltaY;
					}

					x += deltaX;
				}

			}


			ClientResetScene();
		}

		const int ARRAY_SIZE_X  = 5;
		const int ARRAY_SIZE_Y =5 ;
		const int ARRAY_SIZE_Z =5;

		//maximum number of objects (and allow user to shoot additional boxes)
		const int MAX_PROXIES =(ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024);

		///scaling of the objects (0.1 = 20 centimeter boxes )
		const float SCALING = 1f;
		const int START_POS_X =-5;
		const int START_POS_Y =20;
		const int START_POS_Z =-3;

        static void Main(string[] args)
        {
            using (Box2DDemo game = new Box2DDemo())
            {
                game.Run();
            }
        }

	}

}
