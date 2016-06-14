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

using BulletXNA;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using BulletXNA.LinearMath;

namespace BulletXNADemos.Demos
{
    public class IslandStressDemo : DemoApplication
    {
        const float SCALING = 1.0f;
        public override void InitializeDemo()
        {
            base.InitializeDemo();
            SetCameraDistance(SCALING * 50f);

            //string filename = @"C:\users\man\bullett\xna-largemesh-output.txt";
            //FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
            //BulletGlobals.g_streamWriter = new StreamWriter(filestream);

            ///collision configuration contains default setup for memory, collision setup
            m_collisionConfiguration = new DefaultCollisionConfiguration();

            ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
            m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);

            m_broadphase = new DbvtBroadphase();
            IOverlappingPairCache pairCache = null;
            //pairCache = new SortedOverlappingPairCache();

            //m_broadphase = new SimpleBroadphase(10000, pairCache);

            ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
            SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
            m_constraintSolver = sol;

            m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

            IndexedVector3 gravity = new IndexedVector3(0, -10, 0);
            m_dynamicsWorld.SetGravity(ref gravity);

            m_profileManager = new BasicProfileManager();
            BulletGlobals.g_profileManager = m_profileManager;
            m_profileIterator = m_profileManager.getIterator();

            ///create a few basic rigid bodies
            IndexedVector3 halfExtents = new IndexedVector3(50, 50, 50);
            //IndexedVector3 halfExtents = new IndexedVector3(10, 10, 10);
            //CollisionShape groundShape = new BoxShape(ref halfExtents);
            CollisionShape groundShape = new StaticPlaneShape(new IndexedVector3(0,1,0), 0);
            LocalCreateRigidBody(0f, IndexedMatrix.Identity, groundShape);
            //CollisionShape groundShape = BuildLargeMesh();
            m_collisionShapes.Add(groundShape);
            CollisionShape sphereShape = new SphereShape(0.2f);
            int size = 16;// 5; // 16
            for (int i = 0; i < size; ++i)
            {
                for (int j = 0; j < size; ++j)
                {
                    IndexedMatrix m = IndexedMatrix.CreateTranslation(new IndexedVector3(i, 1, j));
                    RigidBody rb = LocalCreateRigidBody(1f, m, sphereShape);
                    rb.SetActivationState(ActivationState.ISLAND_SLEEPING);
                }
            }

            ClientResetScene();

        }


        static void Main(string[] args)
        {
            using (IslandStressDemo game = new IslandStressDemo())
            {
                game.Run();
            }
        }

    }
}
