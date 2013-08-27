
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
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework;

namespace BulletXNADemos.Demos
{
    public class CornerDemo : DemoApplication
    {
        public CornerDemo()
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
            m_broadphase = new AxisSweep3Internal(ref worldMin, ref worldMax, 0xfffe, 0xffff, 16384, null, false,m_dispatcher);

            IOverlappingPairCache pairCache = null;
            //pairCache = new SortedOverlappingPairCache();
            pairCache = new HashedOverlappingPairCache();

            m_broadphase = new DbvtBroadphase(pairCache,m_dispatcher);


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
            float mass = 0f;

            float topY = 2.5f;
            float bottomY = 0.5f;

            float diff = 10f;
            float left = -(diff / 2f);
            float right = -left;


            LocalCreateRigidBody(mass, ref groundTransform, groundShape);
            {
                /// Create Dynamic Objects
                IndexedMatrix startTransform = IndexedMatrix.Identity;

                mass = 0f;

                //rigidbody is dynamic if and only if mass is non zero, otherwise static
                bool isDynamic = mass != 0f;


                RigidBody rb = null;

                //startTransform._origin = new IndexedVector3(left, topY, 0);
                //collisionTopLeftCorner = BuildCorner(vertices, topLeft);
                //rb = LocalCreateRigidBody(0f, startTransform, collisionTopLeftCorner);
                //rb.SetUserPointer("TopLeftCorner");

                //startTransform._origin = new IndexedVector3(right, topY, 0);
                //collisionTopRightCorner = BuildCorner(vertices, topRight);
                //rb = LocalCreateRigidBody(0f, startTransform, collisionTopRightCorner);
                //rb.SetUserPointer("TopRightCorner");

                startTransform._origin = new IndexedVector3(left, bottomY, 0);
                collisionBottomLeftCorner = BuildCorner(vertices, bottomLeft);
                rb = LocalCreateRigidBody(0f, startTransform, collisionBottomLeftCorner);
                rb.SetUserPointer("BottomLeftCorner");

                startTransform._origin = new IndexedVector3(right, bottomY, 0);
                collisionBottomRightCorner = BuildCorner(vertices, bottomRight);
                rb = LocalCreateRigidBody(0f, startTransform, collisionBottomRightCorner);
                rb.SetUserPointer("BottomRightCorner");


                startTransform._origin = IndexedVector3.Zero;

                m_playerSphere = LocalCreateRigidBody(1f, startTransform, new SphereShape(0.25f));
                m_playerSphere.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
            }



            BulletGlobals.gDebugDraw.SetDebugMode(BulletXNA.LinearMath.DebugDrawModes.DBG_DrawAabb | BulletXNA.LinearMath.DebugDrawModes.DBG_DrawNormals | BulletXNA.LinearMath.DebugDrawModes.DBG_DrawContactPoints);
            m_dynamicsWorld.SetDebugDrawer(BulletGlobals.gDebugDraw);

            //ClientResetScene();
        }

        protected override void Update(GameTime gameTime)
        {
            base.Update(gameTime);
            //DbvtBroadphase dbvt = m_broadphase as DbvtBroadphase;
            //if (dbvt != null)
            //{
            //    dbvt.Visualise();
            //}
        }


        public static CollisionShape BuildCorner(IndexedVector3[] vertices, int[] indices)
        {
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
            TriangleIndexVertexArray indexVertexArray = new TriangleIndexVertexArray(indices.Length / 3, intArray, indexStride, vertexArray.Count, vertexArray, vertStride);
            TriangleMeshShape triangleMesh = new TriangleMeshShape(indexVertexArray);
            return triangleMesh;
        }

        public override void KeyboardCallback(Keys key, int x, int y, GameTime gameTime, bool released, ref KeyboardState newState, ref KeyboardState oldState)
        {
            float elapsedTime = (float)gameTime.ElapsedGameTime.TotalSeconds;
            float multiplier = 100;

            elapsedTime *= multiplier;
            
            if (key == Keys.N)
            {
                //m_playerSphere.SetActivationState(ActivationState.ACTIVE_TAG);
                IndexedVector3 iv = IndexedVector3.Left * elapsedTime;
                m_playerSphere.ApplyCentralImpulse(ref iv);

            }
            else if (key == Keys.M)
            {
                //m_playerSphere.SetActivationState(ActivationState.ACTIVE_TAG);
                IndexedVector3 iv = IndexedVector3.Right * elapsedTime;
                m_playerSphere.ApplyCentralImpulse(ref iv);
            }
            else if (key != Keys.N && key != Keys.M)
            {
                base.KeyboardCallback(key, x, y, gameTime, released, ref newState, ref oldState);
            }

        }


        // slope.
        IndexedVector3[] vertices = new IndexedVector3[]{new IndexedVector3(-0.5f,-0.5f,-0.5f),new IndexedVector3(0.5f,-0.5f,-0.5f),new IndexedVector3(-0.5f,-0.5f,0.5f),new IndexedVector3(0.5f,-0.5f,0.5f),
                        new IndexedVector3(-0.5f,0.5f,-0.5f),new IndexedVector3(0.5f,0.5f,-0.5f),new IndexedVector3(-0.5f,0.5f,0.5f),new IndexedVector3(0.5f,0.5f,0.5f)};


        int[] topLeft = new int[] { 0, 4, 5, 4, 6, 7, 7, 5, 4, 6, 4, 0, 0, 2, 6, 7, 6, 2, 5, 7, 2, 2, 0, 5 };
        int[] topRight = new int[] { 1, 4, 5, 1, 5, 7, 7, 3, 1, 3, 7, 6, 7, 5, 4, 4, 6, 7, 4, 1, 3, 3, 6, 4 };
        int[] bottomLeft = new int[] { 1, 0, 4, 1, 3, 2, 2, 0, 1, 0, 2, 6, 6, 4, 0, 6, 2, 3, 4, 6, 3, 3, 1, 4 };
        int[] bottomRight = new int[] { 0, 5, 1, 1, 5, 7, 7, 3, 1, 7, 2, 3, 0, 1, 3, 3, 2, 0, 5, 0, 2, 2, 7, 5 };

        public static CollisionShape collisionTopLeftCorner;
        public static CollisionShape collisionBottomLeftCorner;
        public static CollisionShape collisionTopRightCorner;
        public static CollisionShape collisionBottomRightCorner;

        public RigidBody m_playerSphere;




        ///scaling of the objects (0.1 = 20 centimeter boxes )
        public const float SCALING = 1f;

        static void Main(string[] args)
        {
            using (CornerDemo game = new CornerDemo())
            {
                game.Run();
            }
        }

    }
}
