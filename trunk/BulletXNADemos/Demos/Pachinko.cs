
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
    public class Pachinko : DemoApplication
    {
        public Pachinko()
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
			
			BuildBoard();

        }


		private void BuildBoard()
		{
			IndexedVector3 boardBackExtents = new IndexedVector(40,40,1) /2f;
			IndexedVector3 boardSideExtent = new IndexedVector(1,40,1) /2f;
			IndexedVector3 boardBarExtent = new IndexedVector(40,1,1)/2f;
			IndexedVector3 pinExtent = new IndexedVector(0.5f,0.5f,1f);
			
			
			IndexedVector3 boardCenter = new IndexedVector3(0,0,0);
			
			// build frame.objects
			
			BoxShape boardBack = new BoxShape(boardBackExtents);
			BoxShape boardSide = new BoxShape(boardSideExtents);
			BoxShape boardBar = new BoxShape(boardTopExtents);
			
			CapsuleShape pinShape = new CapsuleShape(pinExtent);
			
			
			
			// now RB's
			IndexedMatrix trans = IndexedMatrix.CreateTranslation(boardCenter);
			
			float mass = 0f;
			
			LocalCreateRigidBody(mass,trans,boardBack);
			IndexedVector leftSide = new IndexedVector(boardCenter.X-boardBackExtents.X+boardSideExtent.X,boardCenter.Y,boardCenter.Z);
			IndexedVector rightSide = new IndexedVector(boardCenter.X+boardBackExtents.X-boardSideExtent.X,boardCenter.Y,boardCenter.Z);

			IndexedVector topBar = new IndexedVector(boardCenter.X,boardCenter.Y+boardBackExtent.Y - boardBarExtent.Y,boardCenter.Z);
			IndexedVector bottomBar = new IndexedVector(boardCenter.X,boardCenter.Y-boardBackExtent.Y + boardBarExtent.Y,boardCenter.Z);
			
			
			trans = IndexedMatrix.CreateTranslation(leftSide);
			LocalCreateRigidBody(mass,trans,boardSide);
			trans = IndexedMatrix.CreateTranslation(RightSide);
			LocalCreateRigidBody(mass,trans,boardSide);
			trans = IndexedMatrix.CreateTranslation(topBar);
			LocalCreateRigidBody(mass,trans,boardBar);
			trans = IndexedMatrix.CreateTranslation(bottomBar);
			LocalCreateRigidBody(mass,trans,boardBar);
			
			// now place the pins? (simple offset grid to start)
			
			int numPinsX = 8;
			int numPinsY = 8;
			
			// fixme
			IndexedVector3 pinTopLeft = new IndexedVector3(0,0,0);
			IndexedVector3 pinSpacer = new IndexedVector3(1,1,0);
			
			
			for(int i=0;i<numPinsX;++i)
			{
				for(int j=0;j<numPinsY;++j)
				{
					trans = IndexedMatrix.CreateTranslation(topLeft+new IndexedVector3(pinSpacer.X*i,pinSpacer.Y*j,pinSpacer.Z));
					LocalCreateRigidBody(mass,trans,pinShape);
				}
			}
			
		}



        static void Main(string[] args)
        {
            using (Pachinko game = new Pachinko())
            {
                game.Run();
            }
        }

    }
}
