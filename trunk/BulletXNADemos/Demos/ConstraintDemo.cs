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

#define P2P

using BulletXNA;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using Microsoft.Xna.Framework;
using BulletXNA.LinearMath;
using System.IO;

namespace BulletXNADemos.Demos
{
    public class ConstraintDemo : DemoApplication
    {

        const float CUBE_HALF_EXTENTS = 1.0f;
        IndexedMatrix sliderTransform = IndexedMatrix.Identity;
        IndexedVector3 lowerSliderLimit = new IndexedVector3(-10,0,0);
        IndexedVector3 hiSliderLimit = new IndexedVector3(10,0,0);

        RigidBody d6body0 = null;
        HingeConstraint spDoorHinge = null;
        HingeConstraint spHingeDynAB = null;
        Generic6DofConstraint spSlider6Dof = null;

        const bool s_bTestConeTwistMotor = false;

        public override void InitializeDemo()
        {
            //string filename = @"E:\users\man\bullet\xna-constraint-output.txt";
            //FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
            //BulletGlobals.g_streamWriter = new StreamWriter(filestream);

            //maxiterations = 100;

	        m_collisionConfiguration = new DefaultCollisionConfiguration();
	        m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);
	        IndexedVector3 worldMin = new IndexedVector3(-1000,-1000,-1000);
	        IndexedVector3 worldMax = new IndexedVector3(1000,1000,1000);
            m_broadphase = new AxisSweep3Internal(ref worldMin, ref worldMax, 0xfffe, 0xffff, 16384, null, false);
	        m_constraintSolver = new SequentialImpulseConstraintSolver();
	        m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_constraintSolver,m_collisionConfiguration);
            m_dynamicsWorld.SetDebugDrawer(m_debugDraw);

            SetCameraDistance(26f);

            //CollisionShape groundShape = new BoxShape(new IndexedVector3(50f, 40f, 50f));
            CollisionShape groundShape = new StaticPlaneShape(new IndexedVector3(0, 1, 0), 40);

            m_collisionShapes.Add(groundShape);
            IndexedMatrix groundTransform = IndexedMatrix.Identity;
            groundTransform._origin = new IndexedVector3(0, -56, 0);
            RigidBody groundBody = LocalCreateRigidBody(0, ref groundTransform, groundShape);

            CollisionShape shape = new BoxShape(new IndexedVector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS));
            m_collisionShapes.Add(shape);
            IndexedMatrix trans = IndexedMatrix.Identity;
            trans._origin = new IndexedVector3(0, 20, 0);

	        float mass = 1f;

#if true
	    //point to point constraint with a breaking threshold
	    {
		    trans = IndexedMatrix.Identity;
		    trans._origin = new IndexedVector3(1,30,-5);
		    LocalCreateRigidBody( mass,trans,shape);
		    trans._origin = new IndexedVector3(0,0,-5);

		    RigidBody body0 = LocalCreateRigidBody( mass,trans,shape);
		    
            trans._origin = new IndexedVector3(2*CUBE_HALF_EXTENTS,20,0);
		    mass = 1.0f;
		    RigidBody body1 = null;//localCreateRigidBody( mass,trans,shape);
		    IndexedVector3 pivotInA = new IndexedVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,0);
		    TypedConstraint p2p = new Point2PointConstraint(body0,ref pivotInA);
		    m_dynamicsWorld.AddConstraint(p2p);
		    p2p.SetBreakingImpulseThreshold(10.2f);
		    p2p.SetDbgDrawSize(5.0f);
	    }
#endif



#if true
        //point to point constraint (ball socket)
			//SEEMS OK
	        {
                //trans = IndexedMatrix.Identity;
		        RigidBody body0 = LocalCreateRigidBody( mass,ref trans,shape);
		        trans._origin = new IndexedVector3(2*CUBE_HALF_EXTENTS,20,0);

		        mass = 1f;
		        RigidBody body1 = null;//localCreateRigidBody( mass,trans,shape);

		        IndexedVector3 pivotInA = new IndexedVector3(CUBE_HALF_EXTENTS,-CUBE_HALF_EXTENTS,-CUBE_HALF_EXTENTS);
		        IndexedVector3 axisInA = new IndexedVector3(0,0,1);

                IndexedVector3 pivotInB = body1 != null ? body1.GetCenterOfMassTransform().Inverse() * (body0.GetCenterOfMassTransform() * (pivotInA)) : pivotInA;
                IndexedVector3 axisInB = body1 != null ? (body1.GetCenterOfMassTransform()._basis.Inverse() * (body1.GetCenterOfMassTransform()._basis * axisInA)) :
                body0.GetCenterOfMassTransform()._basis * axisInA;
#if P2P
		TypedConstraint p2p = new Point2PointConstraint(body0,ref pivotInA);
		//btTypedConstraint* p2p = new btPoint2PointConstraint(*body0,*body1,pivotInA,pivotInB);
		//btTypedConstraint* hinge = new btHingeConstraint(*body0,*body1,pivotInA,pivotInB,axisInA,axisInB);
		m_dynamicsWorld.AddConstraint(p2p);
		p2p.SetDbgDrawSize(5.0f);
#else

		        HingeConstraint hinge = new HingeConstraint(body0,ref pivotInA,ref axisInA,false);
        		
		        float	targetVelocity = 1f;
		        float	maxMotorImpulse = 1.0f;
		        hinge.EnableAngularMotor(true,targetVelocity,maxMotorImpulse);

		        m_dynamicsWorld.AddConstraint(hinge);//p2p);
		        hinge.SetDbgDrawSize(5f);
        #endif
	        }
#endif

#if true
            //create a slider, using the generic D6 constraint
			// SEEMS OK
	        {
		        mass = 1f;
		        IndexedVector3 sliderWorldPos = new IndexedVector3(0,10,0);
		        IndexedVector3 sliderAxis = new IndexedVector3(1,0,0);
		        float angle=0f;//SIMD_RADS_PER_DEG * 10.f;
		        IndexedBasisMatrix sliderOrientation = new IndexedBasisMatrix(new IndexedQuaternion(sliderAxis,angle));
		        trans = IndexedMatrix.Identity;
		        trans._origin = sliderWorldPos;
		        //trans.setBasis(sliderOrientation);
		        sliderTransform = trans;

		        d6body0 = LocalCreateRigidBody( mass,ref trans,shape);
		        d6body0.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		        RigidBody fixedBody1 = LocalCreateRigidBody(0,ref trans,null);
		        m_dynamicsWorld.AddRigidBody(fixedBody1);

		        IndexedMatrix frameInA, frameInB;
		        frameInA = IndexedMatrix.Identity;
		        frameInB = IndexedMatrix.Identity;
                frameInA._origin = new IndexedVector3(0, 5, 0);
                frameInB._origin = new IndexedVector3(0, 5, 0);

        //		bool useLinearReferenceFrameA = false;//use fixed frame B for linear llimits
		        bool useLinearReferenceFrameA = true;//use fixed frame A for linear llimits
		        spSlider6Dof = new Generic6DofConstraint(fixedBody1, d6body0,ref frameInA,ref frameInB,useLinearReferenceFrameA);
                spSlider6Dof.SetLinearLowerLimit(ref lowerSliderLimit);
                spSlider6Dof.SetLinearUpperLimit(ref hiSliderLimit);

		        //range should be small, otherwise singularities will 'explode' the constraint
                IndexedVector3 angularLower = new IndexedVector3(-1.5f,0,0);
                IndexedVector3 angularUpper = -angularLower;
                spSlider6Dof.SetAngularLowerLimit(ref angularLower);
                spSlider6Dof.SetAngularUpperLimit(ref angularUpper);
        //		slider.setAngularLowerLimit(IndexedVector3(0,0,0));
        //		slider.setAngularUpperLimit(IndexedVector3(0,0,0));
                spSlider6Dof.SetAngularLowerLimit(new IndexedVector3(-MathUtil.SIMD_PI, 0, 0));
                spSlider6Dof.SetAngularUpperLimit(new IndexedVector3(1.5f, 0, 0));

                spSlider6Dof.GetTranslationalLimitMotor().m_enableMotor[0] = true;
                spSlider6Dof.GetTranslationalLimitMotor().m_targetVelocity.X = -5.0f;
                spSlider6Dof.GetTranslationalLimitMotor().m_maxMotorForce.X = 0.1f;


                m_dynamicsWorld.AddConstraint(spSlider6Dof);
                spSlider6Dof.SetDbgDrawSize(5f);

	        }
#endif
#if true
	        { // create a door using hinge constraint attached to the world
		        CollisionShape pDoorShape = new BoxShape(new IndexedVector3(2.0f, 5.0f, 0.2f));
		        m_collisionShapes.Add(pDoorShape);
		        IndexedMatrix doorTrans = IndexedMatrix.Identity;
		        doorTrans._origin = new IndexedVector3(-5.0f, -2.0f, 0.0f);
		        RigidBody pDoorBody = LocalCreateRigidBody( 1.0f, ref doorTrans, pDoorShape);
		        pDoorBody.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		        IndexedVector3 btPivotA = new IndexedVector3( 10f+2.1f, -2.0f, 0.0f ); // right next to the door slightly outside
		        IndexedVector3 btAxisA = new IndexedVector3( 0.0f, 1.0f, 0.0f ); // pointing upwards, aka Y-axis

		        spDoorHinge = new HingeConstraint( pDoorBody, ref btPivotA, ref btAxisA,false );

                spDoorHinge.SetLimit(-MathUtil.SIMD_PI * 0.25f, MathUtil.SIMD_PI * 0.25f);
		        m_dynamicsWorld.AddConstraint(spDoorHinge);
		        spDoorHinge.SetDbgDrawSize(5.0f);

	        }
#endif
#if true
            { // create a generic 6DOF constraint
				// SEEMS OK - But debug draw a bit wrong?
		        IndexedMatrix tr = IndexedMatrix.Identity;
		        tr._origin = new IndexedVector3(10f, 6f, 0f);
                //tr.getBasis().setEulerZYX(0,0,0);
        //		RigidBody pBodyA = localCreateRigidBody( mass, tr, shape);
		        RigidBody pBodyA = LocalCreateRigidBody( 0.0f, ref tr, shape);
        //		RigidBody pBodyA = localCreateRigidBody( 0.0, tr, 0);
		        pBodyA.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

                tr = IndexedMatrix.Identity;
		        tr._origin = new IndexedVector3(0f, 6f, 0f);		        
                //tr.getBasis().setEulerZYX(0,0,0);
		        RigidBody pBodyB = LocalCreateRigidBody(mass, ref tr, shape);
		        pBodyB.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

		        IndexedMatrix frameInA, frameInB;
		        frameInA = IndexedMatrix.CreateTranslation(-5,0,0);
		        frameInB = IndexedMatrix.CreateTranslation(5,0,0);

		        Generic6DofConstraint pGen6DOF = new Generic6DofConstraint(pBodyA, pBodyB, ref frameInA, ref frameInB, true);
        //		btGeneric6DofConstraint* pGen6DOF = new btGeneric6DofConstraint(*pBodyA, *pBodyB, frameInA, frameInB, false);
		        IndexedVector3 linearLower = new IndexedVector3(-10, -2, -1);
                pGen6DOF.SetLinearLowerLimit(ref linearLower);
                IndexedVector3 linearUpper = new IndexedVector3(10,2,1);
		        pGen6DOF.SetLinearUpperLimit(ref linearUpper);
                // ? why again?
                //linearLower = new IndexedVector3(-10,0,0);
                //pGen6DOF.setLinearLowerLimit(ref linearLower);
        //		pGen6DOF.setLinearUpperLimit(IndexedVector3(10., 0., 0.));
        //		pGen6DOF.setLinearLowerLimit(IndexedVector3(0., 0., 0.));
        //		pGen6DOF.setLinearUpperLimit(IndexedVector3(0., 0., 0.));

        //		pGen6DOF.getTranslationalLimitMotor().m_enableMotor[0] = true;
        //		pGen6DOF.getTranslationalLimitMotor().m_targetVelocity[0] = 5.0f;
        //		pGen6DOF.getTranslationalLimitMotor().m_maxMotorForce[0] = 0.1f;


        //		pGen6DOF.setAngularLowerLimit(IndexedVector3(0., SIMD_HALF_PI*0.9, 0.));
        //		pGen6DOF.setAngularUpperLimit(IndexedVector3(0., -SIMD_HALF_PI*0.9, 0.));
        //		pGen6DOF.setAngularLowerLimit(IndexedVector3(0., 0., -SIMD_HALF_PI));
        //		pGen6DOF.setAngularUpperLimit(IndexedVector3(0., 0., SIMD_HALF_PI));

                IndexedVector3 angularLower = new IndexedVector3(-MathUtil.SIMD_HALF_PI * 0.5f, -0.75f, -MathUtil.SIMD_HALF_PI * 0.8f);
                IndexedVector3 angularUpper = -angularLower;
		        pGen6DOF.SetAngularLowerLimit(ref angularLower);
		        pGen6DOF.SetAngularUpperLimit(ref angularUpper);
        //		pGen6DOF.setAngularLowerLimit(IndexedVector3(0.f, -0.75, SIMD_HALF_PI * 0.8f));
        //		pGen6DOF.setAngularUpperLimit(IndexedVector3(0.f, 0.75, -SIMD_HALF_PI * 0.8f));
        //		pGen6DOF.setAngularLowerLimit(IndexedVector3(0.f, -SIMD_HALF_PI * 0.8f, SIMD_HALF_PI * 1.98f));
        //		pGen6DOF.setAngularUpperLimit(IndexedVector3(0.f, SIMD_HALF_PI * 0.8f,  -SIMD_HALF_PI * 1.98f));

        		
        		
        //		pGen6DOF.setAngularLowerLimit(IndexedVector3(-0.75,-0.5, -0.5));
        //		pGen6DOF.setAngularUpperLimit(IndexedVector3(0.75,0.5, 0.5));
        //		pGen6DOF.setAngularLowerLimit(IndexedVector3(-0.75,0., 0.));
        //		pGen6DOF.setAngularUpperLimit(IndexedVector3(0.75,0., 0.));

		        m_dynamicsWorld.AddConstraint(pGen6DOF, true);
		        pGen6DOF.SetDbgDrawSize(5.0f);
	        }
#endif
#if true
            { // create a ConeTwist constraint

		        IndexedMatrix tr = IndexedMatrix.CreateTranslation(-10,5,0);

		        RigidBody pBodyA = LocalCreateRigidBody( 1.0f, ref tr, shape);
		        pBodyA.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

		        tr = IndexedMatrix.CreateTranslation(-10,-5,0);

		        RigidBody pBodyB = LocalCreateRigidBody(0.0f, ref tr, shape);

		        IndexedMatrix frameInA, frameInB;
                frameInA = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
                frameInA._origin = new IndexedVector3(0, -5, 0);
				frameInB = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
                frameInB._origin = new IndexedVector3(0, 5, 0);

		        ConeTwistConstraint pCT = new ConeTwistConstraint(pBodyA, pBodyB, ref frameInA, ref frameInB);
		        pCT.SetLimit(MathUtil.SIMD_QUARTER_PI, MathUtil.SIMD_QUARTER_PI, MathUtil.SIMD_PI * 0.8f, 1.0f,0.3f,1.0f); // soft limit == hard limit
		        m_dynamicsWorld.AddConstraint(pCT, true);
		        pCT.SetDbgDrawSize(5.0f);
	        }
#endif
#if true
            { // Hinge connected to the world, with motor (to hinge motor with new and old constraint solver)
				// WORKS OK
		        IndexedMatrix tr = IndexedMatrix.Identity;
		        RigidBody pBody = LocalCreateRigidBody( 1.0f, ref tr, shape);
		        pBody.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		        IndexedVector3 btPivotA = new IndexedVector3( 10.0f, 0.0f, 0.0f );
		        IndexedVector3 btAxisA = new IndexedVector3( 0.0f, 0.0f, 1.0f );

		        HingeConstraint pHinge = new HingeConstraint(pBody, ref btPivotA, ref btAxisA,false);
        //		pHinge.enableAngularMotor(true, -1.0, 0.165); // use for the old solver
		        pHinge.EnableAngularMotor(true, -1.0f, 1.65f); // use for the new SIMD solver
		        m_dynamicsWorld.AddConstraint(pHinge);
		        pHinge.SetDbgDrawSize(5.0f);
	        }
#endif
#if true
            {
		// WORKS OK
		// create a universal joint using generic 6DOF constraint
		// create two rigid bodies
		// static bodyA (parent) on top:
		IndexedMatrix tr = IndexedMatrix.CreateTranslation(20,4,0);
		RigidBody pBodyA = LocalCreateRigidBody( 0.0f, ref tr, shape);
		pBodyA.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		// dynamic bodyB (child) below it :
		tr = IndexedMatrix.CreateTranslation(20,0,0);
		RigidBody pBodyB = LocalCreateRigidBody(1.0f, ref tr, shape);
		pBodyB.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		// add some (arbitrary) data to build constraint frames
		IndexedVector3 parentAxis = new IndexedVector3(1.0f, 0.0f, 0.0f); 
		IndexedVector3 childAxis = new IndexedVector3(0.0f, 0.0f, 1.0f);
        IndexedVector3 anchor = new IndexedVector3(20.0f, 2.0f, 0.0f);

		UniversalConstraint pUniv = new UniversalConstraint(pBodyA, pBodyB, ref anchor, ref parentAxis, ref childAxis);
        pUniv.SetLowerLimit(-MathUtil.SIMD_HALF_PI * 0.5f, -MathUtil.SIMD_HALF_PI * 0.5f);
        pUniv.SetUpperLimit(MathUtil.SIMD_HALF_PI * 0.5f, MathUtil.SIMD_HALF_PI * 0.5f);
		// add constraint to world
		m_dynamicsWorld.AddConstraint(pUniv, true);
		// draw constraint frames and limits for debugging
		pUniv.SetDbgDrawSize(5.0f);
	}
#endif

#if true
            // WORKS OK
	{ // create a generic 6DOF constraint with springs 

		IndexedMatrix tr = IndexedMatrix.CreateTranslation(-20f,16f,0f);
        //tr.setIdentity();
        //tr.setOrigin(btVector3(btScalar(-20.), btScalar(16.), btScalar(0.)));
        //tr.getBasis().setEulerZYX(0,0,0);
		RigidBody pBodyA = LocalCreateRigidBody( 0.0f, ref tr, shape);
		pBodyA.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

        //tr.setIdentity();
        //tr.setOrigin(btVector3(btScalar(-10.), btScalar(16.), btScalar(0.)));
        //tr.getBasis().setEulerZYX(0,0,0);
        tr = IndexedMatrix.CreateTranslation(-10,16,0);
		RigidBody pBodyB = LocalCreateRigidBody(1.0f, ref tr, shape);
		pBodyB.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

        IndexedMatrix frameInA = IndexedMatrix.CreateTranslation(10f,0f,0f);
        IndexedMatrix frameInB = IndexedMatrix.CreateTranslation(0f,0f,0f);

		Generic6DofSpringConstraint pGen6DOFSpring = new Generic6DofSpringConstraint(pBodyA, pBodyB, ref frameInA, ref frameInB, true);
		pGen6DOFSpring.SetLinearUpperLimit(new IndexedVector3(5f, 0f, 0f));
		pGen6DOFSpring.SetLinearLowerLimit(new IndexedVector3(-5f, 0f, 0f));

		pGen6DOFSpring.SetAngularLowerLimit(new IndexedVector3(0f, 0f, -1.5f));
		pGen6DOFSpring.SetAngularUpperLimit(new IndexedVector3(0f, 0f, 1.5f));

		m_dynamicsWorld.AddConstraint(pGen6DOFSpring, true);
		pGen6DOFSpring.SetDbgDrawSize(5.0f);
		
		pGen6DOFSpring.EnableSpring(0, true);
		pGen6DOFSpring.SetStiffness(0, 39.478f);
		pGen6DOFSpring.SetDamping(0, 0.5f);
		pGen6DOFSpring.EnableSpring(5, true);
		pGen6DOFSpring.SetStiffness(5, 39.478f);
		pGen6DOFSpring.SetDamping(0, 0.3f);
		pGen6DOFSpring.SetEquilibriumPoint();
	}
#endif
#if true
    {
		// WORKS OK
		// create a Hinge2 joint
		// create two rigid bodies
		// static bodyA (parent) on top:
		IndexedMatrix tr = IndexedMatrix.CreateTranslation(-20f,4f,0f);
        
        RigidBody pBodyA = LocalCreateRigidBody( 0.0f, ref tr, shape);
		pBodyA.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		// dynamic bodyB (child) below it :
		tr = IndexedMatrix.CreateTranslation(-20f,0f,0f);
        RigidBody pBodyB = LocalCreateRigidBody(1.0f, ref tr, shape);
		pBodyB.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		// add some data to build constraint frames
		IndexedVector3 parentAxis = new IndexedVector3(0.0f, 1.0f, 0.0f); 
		IndexedVector3 childAxis = new IndexedVector3(1.0f, 0.0f, 0.0f);
        IndexedVector3 anchor = new IndexedVector3(-20.0f, 0.0f, 0.0f);
		Hinge2Constraint pHinge2 = new Hinge2Constraint(pBodyA, pBodyB, ref anchor, ref parentAxis, ref childAxis);
		pHinge2.SetLowerLimit(-MathUtil.SIMD_HALF_PI * 0.5f);
        pHinge2.SetUpperLimit(MathUtil.SIMD_HALF_PI * 0.5f);
		// add constraint to world
		m_dynamicsWorld.AddConstraint(pHinge2, true);
		// draw constraint frames and limits for debugging
		pHinge2.SetDbgDrawSize(5.0f);
	}
#endif
#if true
    { 
			// WORKS OK
		// create a Hinge joint between two dynamic bodies
		// create two rigid bodies
		// static bodyA (parent) on top:
		IndexedMatrix tr = IndexedMatrix.CreateTranslation(-20f,-2f,0f);
		RigidBody pBodyA = LocalCreateRigidBody( 1.0f, ref tr, shape);
		pBodyA.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		// dynamic bodyB:
		tr = IndexedMatrix.CreateTranslation(-30f,-2f,0f);
		RigidBody pBodyB = LocalCreateRigidBody(10.0f, ref tr, shape);
		pBodyB.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		// add some data to build constraint frames
		IndexedVector3 axisA = new IndexedVector3(0.0f, 1.0f, 0.0f); 
		IndexedVector3 axisB = new IndexedVector3(0.0f, 1.0f, 0.0f); 
		IndexedVector3 pivotA = new IndexedVector3(-5.0f, 0.0f, 0.0f);
        IndexedVector3 pivotB = new IndexedVector3(5.0f, 0.0f, 0.0f);
		
        spHingeDynAB = new HingeConstraint(pBodyA, pBodyB, ref pivotA, ref pivotB, ref axisA, ref axisB);
        spHingeDynAB.SetLimit(-MathUtil.SIMD_HALF_PI * 0.5f, MathUtil.SIMD_HALF_PI * 0.5f);
		// add constraint to world
		m_dynamicsWorld.AddConstraint(spHingeDynAB, true);
		// draw constraint frames and limits for debugging
		spHingeDynAB.SetDbgDrawSize(5.0f);
	}
#endif

#if true
    { // 6DOF connected to the world, with motor
		IndexedMatrix tr = IndexedMatrix.CreateTranslation(10,-15,0);
		RigidBody pBody = LocalCreateRigidBody( 1.0f, ref tr, shape);
		pBody.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		IndexedMatrix frameB = IndexedMatrix.Identity;
		Generic6DofConstraint pGen6Dof = new Generic6DofConstraint(pBody, ref frameB, false );
		m_dynamicsWorld.AddConstraint(pGen6Dof);
		pGen6Dof.SetDbgDrawSize(5.0f);

		pGen6Dof.SetAngularLowerLimit(new IndexedVector3(0,0,0));
		pGen6Dof.SetAngularUpperLimit(new IndexedVector3(0,0,0));
		pGen6Dof.SetLinearLowerLimit(new IndexedVector3(-10.0f, 0, 0));
		pGen6Dof.SetLinearUpperLimit(new IndexedVector3(10.0f, 0, 0));

		pGen6Dof.GetTranslationalLimitMotor().m_enableMotor[0] = true;
		pGen6Dof.GetTranslationalLimitMotor().m_targetVelocity[0] = 5.0f;
		pGen6Dof.GetTranslationalLimitMotor().m_maxMotorForce[0] = 0.1f;
	}
#endif
        }

        public override void Cleanup()
        {

            base.Cleanup();
        }

        static void Main(string[] args)
        {
            using (ConstraintDemo game = new ConstraintDemo())
            {
                game.Run();
            }
        }
    }
}