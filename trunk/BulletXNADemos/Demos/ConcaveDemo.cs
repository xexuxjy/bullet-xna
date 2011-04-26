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

#define USE_TRIMESH_SHAPE
using System;
using BulletXNA;
using BulletXNA.BulletCollision.BroadphaseCollision;
using BulletXNA.BulletCollision.CollisionDispatch;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.BulletCollision.NarrowPhaseCollision;
using BulletXNA.BulletDynamics.ConstraintSolver;
using BulletXNA.BulletDynamics.Dynamics;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;

namespace BulletXNADemos.Demos
{
    public class ConcaveDemo : DemoApplication
	{

		//-----------------------------------------------------------------------------------------------

		public override void InitializeDemo()
		{
			//string filename = @"C:\users\man\bullett\xna-concave-output.txt";
			//FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
			//BulletGlobals.g_streamWriter = new StreamWriter(filestream);

            
            m_animatedMesh = true;
			base.InitializeDemo();
			int totalTriangles = 2 * (NUM_VERTS_X - 1) * (NUM_VERTS_Y - 1);

            gVertices = new ObjectArray<Vector3>(totalVerts);
			int indicesTotal = totalTriangles * 3;
            gIndices = new ObjectArray<int>(indicesTotal);

            //BulletGlobals.gContactAddedCallback = new CustomMaterialCombinerCallback();

			SetVertexPositions(waveheight,0f);

			int vertStride = 1;
			int indexStride = 3;

			int index=0;
			for (int i=0;i<NUM_VERTS_X-1;i++)
			{
				for (int j=0;j<NUM_VERTS_Y-1;j++)
				{
                    gIndices[index++] = j * NUM_VERTS_X + i;
                    gIndices[index++] = j * NUM_VERTS_X + i + 1;
                    gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;

                    gIndices[index++] = j * NUM_VERTS_X + i;
                    gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;
                    gIndices[index++] = (j + 1) * NUM_VERTS_X + i;
				}
			}

            if (BulletGlobals.g_streamWriter != null)
            {
                index = 0;
                BulletGlobals.g_streamWriter.WriteLine("setIndexPositions");
                for (int i = 0; i < gIndices.Count; i++)
                {
                    BulletGlobals.g_streamWriter.WriteLine(String.Format("{0} {1}", i, gIndices[i]));
                }
            }


			TriangleIndexVertexArray indexVertexArrays = new TriangleIndexVertexArray(totalTriangles,
				gIndices,indexStride,totalVerts,gVertices,vertStride);

			bool useQuantizedAabbCompression = true;


            OptimizedBvh bvh = new OptimizedBvh();
	        Vector3 aabbMin = new Vector3(-1000,-1000,-1000);
	        Vector3 aabbMax = new Vector3(1000,1000,1000);

            m_trimeshShape = new BvhTriangleMeshShape(indexVertexArrays, useQuantizedAabbCompression, ref aabbMin, ref aabbMax, true);
            //CollisionShape trimeshShape = new TriangleMeshShape(indexVertexArrays);

            Vector3 scaling = Vector3.One;
            //m_trimeshShape.SetOptimizedBvh(bvh, ref scaling);
            
			//BulletWorldImporter import = new BulletWorldImporter(0);//don't store info into the world
			//if (import.loadFile("myShape.bullet"))
			//{
			//    int numBvh = import.getNumBvhs();
			//    if (numBvh != 0)
			//    {
			//        OptimizedBvh bvh = import.getBvhByIndex(0);
			//        Vector3 aabbMin = new Vector3(-1000,-1000,-1000);
			//        Vector3 aabbMax = new Vector3(1000,1000,1000);
			
			//        trimeshShape = new indexVertexArrays,useQuantizedAabbCompression,ref aabbMin,ref aabbMax,false);
			//        Vector3 scaling = Vector3.One;
			//        trimeshShape.setOptimizedBvh(bvh, ref scaling);
			//        //trimeshShape  = new btBvhTriangleMeshShape(m_indexVertexArrays,useQuantizedAabbCompression,aabbMin,aabbMax);
			//        //trimeshShape.setOptimizedBvh(bvh);
			
			//    }
			//    int numShape = import.getNumCollisionShapes();
			//    if (numShape != 0)
			//    {
			//        trimeshShape = (BvhTriangleMeshShape)import.getCollisionShapeByIndex(0);
					
			//        //if you know the name, you can also try to get the shape by name:
			//        String meshName = import.getNameForPointer(trimeshShape);
			//        if (meshName != null)
			//        {
			//            trimeshShape = (BvhTriangleMeshShape)import.getCollisionShapeByName(meshName);
			//        }
					
			//    }
			//}


            //CollisionShape groundShape = trimeshShape;//m_trimeshShape;
            CollisionShape groundShape = m_trimeshShape;//m_trimeshShape;

            //groundShape = new TriangleShape(new Vector3(0,` 0, 100), new Vector3(100, 0, 0),new Vector3(-100, 0, -100));
            //groundShape = new StaticPlaneShape(Vector3.Up, 0f);
            //groundShape = new BoxShape(new Vector3(100f, 0.1f, 100f));
            Vector3 up = new Vector3(0.4f,1,0);
            up.Normalize();
            //groundShape = new StaticPlaneShape(up, 0f);
            //groundShape = new TriangleMeshShape(indexVertexArrays);
            
			m_collisionConfiguration = new DefaultCollisionConfiguration();
			m_dispatcher = new	CollisionDispatcher(m_collisionConfiguration);

			Vector3 worldMin = new Vector3(-1000,-1000,-1000);
			Vector3 worldMax = new Vector3(1000,1000,1000);
            //m_broadphase = new AxisSweep3Internal(ref worldMin, ref worldMax, 0xfffe, 0xffff, 16384, null, false);
            m_broadphase = new DbvtBroadphase();
			m_constraintSolver = new SequentialImpulseConstraintSolver();
			m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

			float mass = 0f;
			Matrix startTransform = Matrix.CreateTranslation(new Vector3(2,-2,0));

            //CompoundShape colShape = new CompoundShape();
            //Vector3 halfExtents = new Vector3(4, 1, 1);
            //CollisionShape cylinderShape = new CylinderShapeX(ref halfExtents);
            //CollisionShape boxShape = new BoxShape(new Vector3(4, 1, 1));
            //Matrix localTransform = Matrix.Identity;
            //colShape.addChildShape(ref localTransform, boxShape);
            //Quaternion orn = Quaternion.CreateFromYawPitchRoll(MathUtil.SIMD_HALF_PI, 0f, 0f);
            //localTransform = Matrix.CreateFromQuaternion(orn);
            //colShape.addChildShape(ref localTransform, cylinderShape);

            ////BoxShape colShape = new BoxShape(new Vector3(1, 1, 1));


            //int numCollideObjects = 1;
            //m_collisionShapes.Add(colShape);
            //{
            //    for (int i = 0; i < numCollideObjects; i++)
            //    {
            //        startTransform.Translation = new Vector3(4,10+i*2,1);
            //        localCreateRigidBody(1, ref startTransform,colShape);
            //    }
            //}

			CollisionShape boxShape = new BoxShape(new Vector3(1, 1, 1));
			//CollisionShape boxShape = new SphereShape(1);
            //CollisionShape boxShape = new SphereShape(1);
            //CollisionShape boxShape = new CapsuleShapeZ(0.5f, 1);
            m_collisionShapes.Add(boxShape);
            for (int i = 0; i < 1; i++)
            {
                startTransform.Translation = new Vector3(2f * i, 5, 1);
                LocalCreateRigidBody(1, ref startTransform, boxShape);
            }


			startTransform = Matrix.Identity;
			staticBody = LocalCreateRigidBody(mass, ref startTransform,groundShape);

			staticBody.SetCollisionFlags(staticBody.GetCollisionFlags() | CollisionFlags.CF_KINEMATIC_OBJECT);//STATIC_OBJECT);

			//enable custom material callback
			staticBody.SetCollisionFlags(staticBody.GetCollisionFlags() | CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK);

            //clientResetScene();
		}

		//-----------------------------------------------------------------------------------------------

		public override void  ClientMoveAndDisplay(Microsoft.Xna.Framework.GameTime gameTime)
		{
 			 base.ClientMoveAndDisplay(gameTime);
			float dt = GetDeltaTimeMicroseconds() * 0.000001f;

			if (m_animatedMesh)
			{
				m_offset+=dt;

                SetVertexPositions(waveheight, m_offset);
				
				int i;
				int j;
				Vector3 aabbMin = MathUtil.MAX_VECTOR;
				Vector3 aabbMax = MathUtil.MIN_VECTOR;

				for ( i=NUM_VERTS_X/2-3;i<NUM_VERTS_X/2+2;i++)
				{
					for (j=NUM_VERTS_X/2-3;j<NUM_VERTS_Y/2+2;j++)
					{
					
						MathUtil.VectorMin(gVertices[i+j*NUM_VERTS_X],ref aabbMin);
						MathUtil.VectorMax(gVertices[i+j*NUM_VERTS_X],ref aabbMax);

                        float sin = (float)Math.Sin((float)i + m_offset);
                        float cos = (float)Math.Cos((float)j + m_offset);


						gVertices[i+j*NUM_VERTS_X] = new Vector3((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
							//0.f,
							waveheight * sin * cos,
							(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);

						MathUtil.VectorMin(gVertices[i + j * NUM_VERTS_X], ref aabbMin);
						MathUtil.VectorMax(gVertices[i + j * NUM_VERTS_X], ref aabbMax);

					}
				}

				m_trimeshShape.PartialRefitTree(ref aabbMin,ref aabbMax);

				//clear all contact points involving mesh proxy. Note: this is a slow/unoptimized operation.
				m_dynamicsWorld.GetBroadphase().GetOverlappingPairCache().CleanProxyFromPairs(staticBody.GetBroadphaseHandle(),GetDynamicsWorld().GetDispatcher());
			}
		}

		//-----------------------------------------------------------------------------------------------

		///User can override this material combiner by implementing gContactAddedCallback and setting body0.m_collisionFlags |= btCollisionObject::customMaterialCallback;
		public static float CalculateCombinedFriction(float friction0,float friction1)
		{
			float friction = friction0 * friction1;

			const float MAX_FRICTION  = 10.0f;
			if (friction < -MAX_FRICTION)
				friction = -MAX_FRICTION;
			if (friction > MAX_FRICTION)
				friction = MAX_FRICTION;
			return friction;
		}

		public static float calculateCombinedRestitution(float restitution0, float restitution1)
		{
			return restitution0 * restitution1;
		}

		//-----------------------------------------------------------------------------------------------

		public class CustomMaterialCombinerCallback : IContactAddedCallback
		{
			#region IContactAddedCallback Members

			public bool Callback(ManifoldPoint cp, CollisionObject colObj0, int partId0, int index0, CollisionObject colObj1, int partId1, int index1)
			{
				float friction0 = colObj0.GetFriction();
				float friction1 = colObj1.GetFriction();
				float restitution0 = colObj0.GetRestitution();
				float restitution1 = colObj1.GetRestitution();

				if ((colObj0.GetCollisionFlags() & CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK) != 0)
				{
					friction0 = 1.0f;//partId0,index0
					restitution0 = 0.0f;
				}
				if ((colObj1.GetCollisionFlags() & CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK) != 0)
				{
					if ((index1&1) != 0)
					{
						friction1 = 1.0f;//partId1,index1
					} 
					else
					{
						friction1 = 0f;
					}
					restitution1 = 0f;
				}

				cp.m_combinedFriction = CalculateCombinedFriction(friction0,friction1);
				cp.m_combinedRestitution = calculateCombinedRestitution(restitution0,restitution1);

				//this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
				return true;
			}

			#endregion
		}

		//-----------------------------------------------------------------------------------------------

		public void	SetVertexPositions(float waveheight, float offset)
		{
			for (int i=0;i<NUM_VERTS_X;i++)
			{
				for (int j=0;j<NUM_VERTS_Y;j++)
				{
					gVertices[i+j*NUM_VERTS_X] = new Vector3((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
						//0.f,
						waveheight*(float)Math.Sin((float)i+offset)*(float)Math.Cos((float)j+offset),
						(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
				}
			}
            if (BulletGlobals.g_streamWriter != null)
            {
                BulletGlobals.g_streamWriter.WriteLine("setVertexPositions");
                for (int i = 0; i < gVertices.Count; ++i)
                {
                    MathUtil.PrintVector3(BulletGlobals.g_streamWriter, gVertices[i]);
                }
            }
		}
		
		//-----------------------------------------------------------------------------------------------

		public static ObjectArray<Vector3> gVertices = null;
        public static ObjectArray<int> gIndices = null;
		public static BvhTriangleMeshShape m_trimeshShape = null;
		public static RigidBody staticBody = null;
		public const float waveheight = 5.0f;
		//public const float waveheight = 0.0f;
		public const float TRIANGLE_SIZE = 8.0f;
		public const int NUM_VERTS_X = 6;
		public const int NUM_VERTS_Y = 6;
		public bool m_animatedMesh;
		public const int totalVerts = NUM_VERTS_X * NUM_VERTS_Y;
        private float m_offset = 0f;

		static void Main(string[] args)
		{
			using (ConcaveDemo game = new ConcaveDemo())
			{
				game.Run();
			}
		}



	}
}
