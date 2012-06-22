
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

namespace BulletXNADemos.Demos
{
    public class PyramidDemo : DemoApplication
    {
        public PyramidDemo()
        {
        }

        public override void InitializeDemo()
        {
            maxiterations = 500;
            SetCameraDistance(SCALING * 50f);

            //string filename = @"E:\users\man\bullet\xna-basic-output-1.txt";
            //FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
            //BulletGlobals.g_streamWriter = new StreamWriter(filestream);

            ///collision configuration contains default setup for memory, collision setup
            m_collisionConfiguration = new DefaultCollisionConfiguration();

            ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
            m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);

            m_broadphase = new DbvtBroadphase();
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

            IndexedMatrix groundTransform = IndexedMatrix.CreateTranslation(new IndexedVector3(0, -52, 0));
            //IndexedMatrix groundTransform = IndexedMatrix.CreateTranslation(new IndexedVector3(0,-10,0));
            float mass = 0f;
            LocalCreateRigidBody(mass, ref groundTransform, groundShape);

            CreateScene5(20);

            ClientResetScene();
        }

        private void CreateScene5(int dim)
        {
            BoxShape boxShape = new BoxShape(new IndexedVector3(0.5f));
            float mass = 1.0f;
            for (int x = 0; x < dim; x++)
            {
                for (int e = x; e < dim; e++)
                {
                    //IndexedVector3 pos = new IndexedVector3(e - 0.5f * x, x * 1.01f - 14, 25);
                    IndexedVector3 pos = new IndexedVector3(e - 0.5f * x, x * 1.0f, 0);
                    RigidBody rb = LocalCreateRigidBody(mass, IndexedMatrix.CreateTranslation(pos), boxShape);
                    //m_dynamicsWorld.AddRigidBody(rb);
                }
            }
        }

        public override void ClientMoveAndDisplay(Microsoft.Xna.Framework.GameTime gameTime)
        {
            if (numiterations == 2)
            {
                int ibreak = 0;
            }
            base.ClientMoveAndDisplay(gameTime);
        }

        // test - just 8 objects.
        public const int ARRAY_SIZE_X = 5;
        public const int ARRAY_SIZE_Y = 5;
        public const int ARRAY_SIZE_Z = 5;

        //maximum number of objects (and allow user to shoot additional boxes)
        public const int MAX_PROXIES = (ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z + 1024);

        ///scaling of the objects (0.1 = 20 centimeter boxes )
        public const float SCALING = 1f;

        public const int START_POS_X = -5;
        public const int START_POS_Y = -5;
        public const int START_POS_Z = -3;

        static void Main(string[] args)
        {
            using (PyramidDemo game = new PyramidDemo())
            {
                game.Run();
            }
        }

    }
}
