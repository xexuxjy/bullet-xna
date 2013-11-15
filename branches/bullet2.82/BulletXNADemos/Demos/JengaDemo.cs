
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
    public class JengaDemo : DemoApplication
    {
        public JengaDemo()
        {
        }

        public override void InitializeDemo()
        {
            SetCameraDistance(30f);

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

            m_broadphase = new SimpleBroadphase(1000, pairCache);

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
                int numBlocksTall = 10;//18; //How many 'stories' tall.
                float blockWidth = 6f; //Total width/length of the tower.
                float blockHeight = 2f;


                //create a few dynamic rigidbodies
                IndexedVector3 extents = new IndexedVector3(blockWidth/3, blockHeight, blockWidth);
                IndexedVector3 boxHalfExtents = extents * 0.5f;

                CollisionShape colShape = new BoxShape(boxHalfExtents);
                //btCollisionShape* colShape = new btSphereShape(btScalar(1.));
                //CollisionShape colShape = new CylinderShape(new IndexedVector3(1f, 1, 1f));
                //m_collisionShapes.Add(colShape);

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



                //The default number of iterations is 10, which works fine, but this demo
                //is all about stability (it's jenga!).  Increase the iterations a bit.
                //Even though it's using twice as many iterations, it will early-out
                //before reaching the limit MOST of the time.
                //It's still pretty playable at around 7-8 max iterations, though.

                IndexedMatrix transform = IndexedMatrix.Identity;
                for (int i = 0; i < numBlocksTall; i++)
                {
                    if (i % 2 == 0)
                    {
                        for (int j = 0; j < 3; j++)
                        {
                            transform = IndexedMatrix.Identity;
                            IndexedVector3 position = new IndexedVector3(j * (blockWidth/3) - blockWidth/3,
                                                blockHeight / 2 + i * (blockHeight),
                                                0);
                            //position += boxHalfExtents;
                            transform._origin = position;
                            RigidBody rb = LocalCreateRigidBody(mass, transform, colShape);
                            rb.SetActivationState(ActivationState.ISLAND_SLEEPING);
                        }
                    }
                    else
                    {
                        transform = IndexedMatrix.CreateRotationY(MathUtil.SIMD_HALF_PI);
                        for (int j = 0; j < 3; j++)
                        {
                            IndexedVector3 position = new IndexedVector3(0,
                                            blockHeight / 2 + (i) * (blockHeight),
                                            j * (blockWidth / 3) - blockWidth / 3f);
                            transform._origin = position;
                            //position += boxHalfExtents;
                            transform._origin = position;
                            RigidBody rb = LocalCreateRigidBody(mass, transform, colShape);
                            rb.SetActivationState(ActivationState.ISLAND_SLEEPING);
                        }
                    }
                }
                //game.Camera.Position = new Vector3(0, 5, 15);


            }

            ClientResetScene();
        }


        static void Main(string[] args)
        {
            using (JengaDemo game = new JengaDemo())
            {
                game.Run();
            }
        }

    }
}
