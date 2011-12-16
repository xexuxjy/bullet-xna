
///*
// * C# / XNA  port of Bullet (c) 2011 Mark Neale <xexuxjy@hotmail.com>
// *
// * Bullet Continuous Collision Detection and Physics Library
// * Copyright (c) 2003-2008 Erwin Coumans  http://www.bulletphysics.com/
// *
// * This software is provided 'as-is', without any express or implied warranty.
// * In no event will the authors be held liable for any damages arising from
// * the use of this software.
// * 
// * Permission is granted to anyone to use this software for any purpose, 
// * including commercial applications, and to alter it and redistribute it
// * freely, subject to the following restrictions:
// * 
// * 1. The origin of this software must not be misrepresented; you must not
// *    claim that you wrote the original software. If you use this software
// *    in a product, an acknowledgment in the product documentation would be
// *    appreciated but is not required.
// * 2. Altered source versions must be plainly marked as such, and must not be
// *    misrepresented as being the original software.
// * 3. This notice may not be removed or altered from any source distribution.
// */

//using System;
//using BulletXNA;
//using BulletXNA.BulletCollision;
//using BulletXNA.BulletDynamics;
//using Microsoft.Xna.Framework;
//using BulletXNA.LinearMath;
//using System.IO;

//namespace BulletXNADemos.Demos
//{
//    public class FractureDemo : DemoApplication
//    {
//        public override void InitializeDemo()
//        {
//    SetCameraDistance(SCALING*20.0f);

//    ///collision configuration contains default setup for memory, collision setup
//    m_collisionConfiguration = new DefaultCollisionConfiguration();
//    //m_collisionConfiguration.setConvexConvexMultipointIterations();

//    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
//    m_dispatcher = new	CollisionDispatcher(m_collisionConfiguration);

//    m_broadphase = new DbvtBroadphase();

//    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
//    SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
//    m_solver = sol;

//    //m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

//    FractureDynamicsWorld fractureWorld = new FractureDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
//    m_dynamicsWorld = fractureWorld;

//    m_ShootBoxInitialSpeed=100; 

//    //m_splitImpulse removes the penetration resolution from the applied impulse, otherwise objects might fracture due to deep penetrations.
//    m_dynamicsWorld.GetSolverInfo().m_splitImpulse = true;

//    {
//        ///create a few basic rigid bodies
//        CollisionShape groundShape = new BoxShape(new IndexedVector3(50,1,50));
//    ///	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),0);
//        m_collisionShapes.Add(groundShape);
//        IndexedMatrix groundTransform = IndexedMatrix.Identity;
//        LocalCreateRigidBody(0.0f,ref groundTransform,groundShape);
//    }

//    {
//        ///create a few basic rigid bodies
//        CollisionShape shape = new BoxShape(new IndexedVector3(1,1,1));
//        m_collisionShapes.Add(shape);
//        IndexedMatrix tr = IndexedMatrix.Identity;
//        tr._origin = new IndexedVector3(5,2,0);
//        LocalCreateRigidBody(0.0f,ref tr,shape);
//    }



//    {
//        //create a few dynamic rigidbodies
//        // Re-using the same collision is better for memory usage and performance

//        CollisionShape colShape = new BoxShape(new IndexedVector3(SCALING*1,SCALING*1,SCALING*1));
//        //btCollisionShape* colShape = new btCapsuleShape(SCALING*0.4,SCALING*1);
//        //btCollisionShape* colShape = new btSphereShape(float(1.));
//        m_collisionShapes.Add(colShape);

//        /// Create Dynamic Objects
//        IndexedMatrix startTransform= IndexedMatrix.Identity;

//        float	mass = 1.0f;

//        //rigidbody is dynamic if and only if mass is non zero, otherwise static
//        bool isDynamic = (mass != 0.0f);

//        Vector3 localInertia(0,0,0);
//        if (isDynamic)
//        {
//            colShape.calculateLocalInertia(mass,localInertia);
//        }


//        int gNumObjects = 10;

//        for (int i=0;i<gNumObjects;i++)
//        {
//            btTransform trans;
//            trans.setIdentity();

//            btVector3 pos(i*2*CUBE_HALF_EXTENTS ,20,0);
//            trans.setOrigin(pos);

//            //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
//            btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
//            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
//            btFractureBody* body = new btFractureBody(rbInfo, m_dynamicsWorld);
//            body.setLinearVelocity(btVector3(0,-10,0));

//            m_dynamicsWorld.addRigidBody(body);


//        }

//    }



//    fractureWorld.stepSimulation(1./60.,0);
//    fractureWorld.glueCallback();

//        }

//    }
//}
