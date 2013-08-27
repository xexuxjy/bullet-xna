
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
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace BulletXNADemos.Demos
{
    public class Pachinko : DemoApplication
    {
        public Pachinko()
        {
        }
        
        public override void InitializeDemo()
        {
            //maxiterations = 10;
            SetCameraDistance(100f);
            m_cameraTargetPosition = new IndexedVector3();
            m_cameraPosition = new IndexedVector3(0, 0, -m_cameraDistance);


            //string filename = @"E:\users\man\bullet\xna-basic-output-1.txt";
            //FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
            //BulletGlobals.g_streamWriter = new StreamWriter(filestream);

	        ///collision configuration contains default setup for memory, collision setup
	        m_collisionConfiguration = new DefaultCollisionConfiguration();

	        ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	        m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);

            //m_broadphase = new DbvtBroadphase();
            IOverlappingPairCache pairCache = null;
            //pairCache = new SortedOverlappingPairCache();

            //m_broadphase = new SimpleBroadphase(1000, pairCache);
            m_broadphase = new DbvtBroadphase(pairCache,m_dispatcher);

            IndexedVector3 worldAabbMin = new IndexedVector3(-200, -200, -200);
            IndexedVector3 worldAabbMax = -worldAabbMin;
            //m_broadphase = new AxisSweep3Internal(ref worldAabbMin, ref worldAabbMax, 0xfffe, 0xffff, 16384, null, true);

	        ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	        SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
            m_constraintSolver = sol;

            m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

            IndexedVector3 gravity = new IndexedVector3(0, -10, 0);
	        m_dynamicsWorld.SetGravity(ref gravity);

            m_profileManager = new BasicProfileManager();
            BulletGlobals.g_profileManager = m_profileManager;
            m_profileIterator = m_profileManager.getIterator();

			BuildBoard();

        }

        protected override void Update(GameTime gameTime)
        {
            base.Update(gameTime);
            m_updateCount++;
            if (m_updateCount % m_frequency == 0)
            {
                //DropBall();
            }
        }


		    private void BuildBoard()
		    {
                float width = 40;
                float height = 40;

			    IndexedVector3 boardBackExtents = new IndexedVector3(width,height,2) /2f;
			    IndexedVector3 boardSideExtents = new IndexedVector3(1,height,2) /2f;
			    IndexedVector3 boardBarExtents = new IndexedVector3(width,1,2)/2f;
			    IndexedVector3 pinExtent = new IndexedVector3(0.5f,1f,2f);
			
			
			    IndexedVector3 boardCenter = new IndexedVector3(0,0,0);
			
			    // build frame.objects
			
			    BoxShape boardBack = new BoxShape(boardBackExtents);
			    BoxShape boardSide = new BoxShape(boardSideExtents);
			    BoxShape boardBar = new BoxShape(boardBarExtents);
			    StaticPlaneShape boardFront = new StaticPlaneShape(new IndexedVector3(0,0,-1),-(boardBackExtents.Z+boardSideExtents.Z));
			
			    CollisionShape pinShape = new CapsuleShapeZ(pinExtent.Y,pinExtent.Z);
                //pinShape = new BoxShape(pinExtent);

                //pinShape = new SphereShape(pinExtent.Y);

                m_cameraUp = Vector3.Up;
			
			    // now RB's
                Matrix m = Matrix.Identity;
                IndexedMatrix trans = m;
                trans._origin = boardCenter;
			
			    float mass = 0f;
			
                //LocalCreateRigidBody(mass,trans,boardBack);
                //LocalCreateRigidBody(mass, trans, boardFront);


                IndexedVector3 leftSide = new IndexedVector3(boardCenter.X - boardBackExtents.X + boardSideExtents.X, boardCenter.Y, boardCenter.Z + boardSideExtents.Z);
                IndexedVector3 rightSide = new IndexedVector3(boardCenter.X + boardBackExtents.X - boardSideExtents.X, boardCenter.Y, boardCenter.Z + boardSideExtents.Z);

                IndexedVector3 topBar = new IndexedVector3(boardCenter.X, boardCenter.Y + boardBackExtents.Y - boardBarExtents.Y, boardCenter.Z + boardSideExtents.Z);
                IndexedVector3 bottomBar = new IndexedVector3(boardCenter.X, boardCenter.Y - boardBackExtents.Y + boardBarExtents.Y, boardCenter.Z + boardSideExtents.Z);


                trans._origin = leftSide;
                LocalCreateRigidBody(mass, trans, boardSide);
                trans._origin = rightSide;
                LocalCreateRigidBody(mass, trans, boardSide);
                trans._origin = topBar;
                LocalCreateRigidBody(mass, trans, boardBar);
                trans._origin = bottomBar;
                LocalCreateRigidBody(mass, trans, boardBar);




                // now place the pins? (simple offset grid to start)

                int numPinsX = 8;
                int numPinsY = 8;

                // fixme
                IndexedVector3 pinTopLeft = new IndexedVector3(-boardBackExtents.X + 4, boardBackExtents.Y - 5, boardCenter.Z + boardSideExtents.Z);
                IndexedVector3 pinSpacer = new IndexedVector3(pinExtent.Y * 4f, -pinExtent.Y * 4f, 0);


                
                float ballRadius = 0.9f;
                float fudge = 3f;
                m_dropSphereShape = new SphereShape(ballRadius);

                m_ballDropSpot = new Vector3(1f, pinTopLeft.Y + 1, boardCenter.Z + boardBackExtents.Z+ballRadius);
                //new SphereShape(0.95f);


                m_debugDraw.SetDebugMode(m_debugDraw.GetDebugMode() | DebugDrawModes.DBG_DrawAabb);

                trans._origin = pinTopLeft;

                LocalCreateRigidBody(mass, trans, pinShape);


                for (int i = 0; i < numPinsX; ++i)
                {
                    for (int j = 0; j < numPinsY; ++j)
                    {
                        IndexedVector3 pos = new IndexedVector3(pinSpacer.X * i, (pinSpacer.Y * j), pinSpacer.Z);
                        // stagger rows.
                        if (j % 2 == 1)
                        {
                            pos.X += pinSpacer.X/2f;
                        }

                        trans._origin = pinTopLeft + pos;
                        LocalCreateRigidBody(mass, trans, pinShape);
                    }
                }
			
		    }

        public void DropBall()
        {
            RigidBody rb = LocalCreateRigidBody(1f, IndexedMatrix.CreateTranslation(m_ballDropSpot), m_dropSphereShape);
            rb.SetLinearFactor(new IndexedVector3(1, 1, 0));




        }

        public override void KeyboardCallback(Microsoft.Xna.Framework.Input.Keys key, int x, int y, GameTime gameTime, bool released, ref Microsoft.Xna.Framework.Input.KeyboardState newState, ref Microsoft.Xna.Framework.Input.KeyboardState oldState)
        {
            base.KeyboardCallback(key, x, y, gameTime, released, ref newState, ref oldState);
            if (key == Keys.V && released)
            {
                DropBall();
            }
        }


        static void Main(string[] args)
        {
            using (Pachinko game = new Pachinko())
            {
                game.Run();
            }
        }

        private IndexedVector3 m_ballDropSpot;
        private SphereShape m_dropSphereShape = null;
        private int m_updateCount = 0;
        private int m_frequency = 30;
    }
}
