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
using BulletXNA.BulletCollision.CollisionDispatch;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.BulletDynamics.ConstraintSolver;
using BulletXNA.BulletDynamics.Dynamics;
using Microsoft.Xna.Framework;

namespace BulletXNADemos.Demos
{
    public class ConstraintDemo : DemoApplication
    {

        const float CUBE_HALF_EXTENTS = 1.0f;
        Matrix sliderTransform = Matrix.Identity;
        Vector3 lowerSliderLimit = new Vector3(-10,0,0);
        Vector3 hiSliderLimit = new Vector3(10,0,0);

        RigidBody d6body0 = null;
        HingeConstraint spDoorHinge = null;
        HingeConstraint spHingeDynAB = null;
        Generic6DofConstraint spSlider6Dof = null;

        const bool s_bTestConeTwistMotor = false;

        public override void InitializeDemo()
        {
	        m_collisionConfiguration = new DefaultCollisionConfiguration();
	        m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);
	        Vector3 worldMin = new Vector3(-1000,-1000,-1000);
	        Vector3 worldMax = new Vector3(1000,1000,1000);
            m_broadphase = new AxisSweep3Internal(ref worldMin, ref worldMax, 0xfffe, 0xffff, 16384, null, false);
	        m_constraintSolver = new SequentialImpulseConstraintSolver();
	        m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_constraintSolver,m_collisionConfiguration);
            m_dynamicsWorld.SetDebugDrawer(m_debugDraw);

            SetCameraDistance(26f);

            //CollisionShape groundShape = new BoxShape(new Vector3(50f, 40f, 50f));
            CollisionShape groundShape = new StaticPlaneShape(new Vector3(0, 1, 0), 40);

            m_collisionShapes.Add(groundShape);
            Matrix groundTransform = Matrix.Identity;
            groundTransform.Translation = new Vector3(0, -56, 0);
            RigidBody groundBody = LocalCreateRigidBody(0, ref groundTransform, groundShape);

            CollisionShape shape = new BoxShape(new Vector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS));
            m_collisionShapes.Add(shape);
            Matrix trans = Matrix.Identity;
            trans.Translation = new Vector3(0, 20, 0);

	        float mass = 1f;
        #if true
	        //point to point constraint (ball socket)
			//SEEMS OK
	        {
		        RigidBody body0 = LocalCreateRigidBody( mass,ref trans,shape);
		        trans.Translation = new Vector3(2*CUBE_HALF_EXTENTS,20,0);

		        mass = 1f;
		        RigidBody body1 = null;//localCreateRigidBody( mass,trans,shape);

		        Vector3 pivotInA = new Vector3(CUBE_HALF_EXTENTS,-CUBE_HALF_EXTENTS,-CUBE_HALF_EXTENTS);
		        Vector3 axisInA = new Vector3(0,0,1);

		        Vector3 pivotInB = body1 != null ? Vector3.Transform((Vector3.Transform(pivotInA,body0.GetCenterOfMassTransform())),Matrix.Invert(body1.GetCenterOfMassTransform())) : pivotInA;
		        Vector3 axisInB = body1 != null ? Vector3.Transform((Vector3.TransformNormal(axisInA,body1.GetCenterOfMassTransform())),Matrix.Invert(MathUtil.BasisMatrix(body1.GetCenterOfMassTransform()))) : 
                    Vector3.TransformNormal( axisInA,body0.GetCenterOfMassTransform());

		        HingeConstraint hinge = new HingeConstraint(body0,ref pivotInA,ref axisInA,false);
        		
		        float	targetVelocity = 1f;
		        float	maxMotorImpulse = 1.0f;
		        hinge.EnableAngularMotor(true,targetVelocity,maxMotorImpulse);

		        m_dynamicsWorld.AddConstraint(hinge);//p2p);
		        hinge.SetDbgDrawSize(5f);

	        }
        #endif

#if true
	        //create a slider, using the generic D6 constraint
			// SEEMS OK
	        {
		        mass = 1f;
		        Vector3 sliderWorldPos = new Vector3(0,10,0);
		        Vector3 sliderAxis = new Vector3(1,0,0);
		        float angle=0f;//SIMD_RADS_PER_DEG * 10.f;
		        Matrix  sliderOrientation = Matrix.CreateFromQuaternion(Quaternion.CreateFromAxisAngle(sliderAxis ,angle));
		        trans = Matrix.Identity;
		        trans.Translation = sliderWorldPos;
		        //trans.setBasis(sliderOrientation);
		        sliderTransform = trans;

		        d6body0 = LocalCreateRigidBody( mass,ref trans,shape);
		        d6body0.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		        RigidBody fixedBody1 = LocalCreateRigidBody(0,ref trans,null);
		        m_dynamicsWorld.AddRigidBody(fixedBody1);

		        Matrix frameInA, frameInB;
		        frameInA = Matrix.Identity;
		        frameInB = Matrix.Identity;
                frameInA.Translation = new Vector3(0, 5, 0);
                frameInB.Translation = new Vector3(0, 5, 0);

        //		bool useLinearReferenceFrameA = false;//use fixed frame B for linear llimits
		        bool useLinearReferenceFrameA = true;//use fixed frame A for linear llimits
		        spSlider6Dof = new Generic6DofConstraint(fixedBody1, d6body0,ref frameInA,ref frameInB,useLinearReferenceFrameA);
                spSlider6Dof.SetLinearLowerLimit(ref lowerSliderLimit);
                spSlider6Dof.SetLinearUpperLimit(ref hiSliderLimit);

		        //range should be small, otherwise singularities will 'explode' the constraint
                Vector3 angularLower = new Vector3(-1.5f,0,0);
                Vector3 angularUpper = -angularLower;
                spSlider6Dof.SetAngularLowerLimit(ref angularLower);
                spSlider6Dof.SetAngularUpperLimit(ref angularUpper);
        //		slider.setAngularLowerLimit(Vector3(0,0,0));
        //		slider.setAngularUpperLimit(Vector3(0,0,0));

                spSlider6Dof.GetTranslationalLimitMotor().m_enableMotor[0] = true;
                spSlider6Dof.GetTranslationalLimitMotor().m_targetVelocity.X = -5.0f;
                spSlider6Dof.GetTranslationalLimitMotor().m_maxMotorForce.X = 0.1f;


                m_dynamicsWorld.AddConstraint(spSlider6Dof);
                spSlider6Dof.SetDbgDrawSize(5f);

	        }
#endif
#if true
	        { // create a door using hinge constraint attached to the world
		        CollisionShape pDoorShape = new BoxShape(new Vector3(2.0f, 5.0f, 0.2f));
		        m_collisionShapes.Add(pDoorShape);
		        Matrix doorTrans = Matrix.Identity;
		        doorTrans.Translation = new Vector3(-5.0f, -2.0f, 0.0f);
		        RigidBody pDoorBody = LocalCreateRigidBody( 1.0f, ref doorTrans, pDoorShape);
		        pDoorBody.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		        Vector3 btPivotA = new Vector3( 10f+2.1f, -2.0f, 0.0f ); // right next to the door slightly outside
		        Vector3 btAxisA = new Vector3( 0.0f, 1.0f, 0.0f ); // pointing upwards, aka Y-axis

		        spDoorHinge = new HingeConstraint( pDoorBody, ref btPivotA, ref btAxisA,false );

                spDoorHinge.SetLimit(-MathUtil.SIMD_PI * 0.25f, MathUtil.SIMD_PI * 0.25f);
		        m_dynamicsWorld.AddConstraint(spDoorHinge);
		        spDoorHinge.SetDbgDrawSize(5.0f);

	        }
#endif
#if true
            { // create a generic 6DOF constraint
				// SEEMS OK - But debug draw a bit wrong?
		        Matrix tr = Matrix.Identity;
		        tr.Translation = new Vector3(10f, 6f, 0f);
                //tr.getBasis().setEulerZYX(0,0,0);
        //		RigidBody pBodyA = localCreateRigidBody( mass, tr, shape);
		        RigidBody pBodyA = LocalCreateRigidBody( 0.0f, ref tr, shape);
        //		RigidBody pBodyA = localCreateRigidBody( 0.0, tr, 0);
		        pBodyA.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

                tr = Matrix.Identity;
		        tr.Translation = new Vector3(0f, 6f, 0f);		        
                //tr.getBasis().setEulerZYX(0,0,0);
		        RigidBody pBodyB = LocalCreateRigidBody(mass, ref tr, shape);
		        pBodyB.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

		        Matrix frameInA, frameInB;
		        frameInA = Matrix.CreateTranslation(-5,0,0);
		        frameInB = Matrix.CreateTranslation(5,0,0);

		        Generic6DofConstraint pGen6DOF = new Generic6DofConstraint(pBodyA, pBodyB, ref frameInA, ref frameInB, true);
        //		btGeneric6DofConstraint* pGen6DOF = new btGeneric6DofConstraint(*pBodyA, *pBodyB, frameInA, frameInB, false);
		        Vector3 linearLower = new Vector3(-10, -2, -1);
                pGen6DOF.SetLinearLowerLimit(ref linearLower);
                Vector3 linearUpper = new Vector3(10,2,1);
		        pGen6DOF.SetLinearUpperLimit(ref linearUpper);
                // ? why again?
                //linearLower = new Vector3(-10,0,0);
                //pGen6DOF.setLinearLowerLimit(ref linearLower);
        //		pGen6DOF.setLinearUpperLimit(Vector3(10., 0., 0.));
        //		pGen6DOF.setLinearLowerLimit(Vector3(0., 0., 0.));
        //		pGen6DOF.setLinearUpperLimit(Vector3(0., 0., 0.));

        //		pGen6DOF.getTranslationalLimitMotor().m_enableMotor[0] = true;
        //		pGen6DOF.getTranslationalLimitMotor().m_targetVelocity[0] = 5.0f;
        //		pGen6DOF.getTranslationalLimitMotor().m_maxMotorForce[0] = 0.1f;


        //		pGen6DOF.setAngularLowerLimit(Vector3(0., SIMD_HALF_PI*0.9, 0.));
        //		pGen6DOF.setAngularUpperLimit(Vector3(0., -SIMD_HALF_PI*0.9, 0.));
        //		pGen6DOF.setAngularLowerLimit(Vector3(0., 0., -SIMD_HALF_PI));
        //		pGen6DOF.setAngularUpperLimit(Vector3(0., 0., SIMD_HALF_PI));

                Vector3 angularLower = new Vector3(-MathUtil.SIMD_HALF_PI * 0.5f, -0.75f, -MathUtil.SIMD_HALF_PI * 0.8f);
                Vector3 angularUpper = -angularLower;
		        pGen6DOF.SetAngularLowerLimit(ref angularLower);
		        pGen6DOF.SetAngularUpperLimit(ref angularUpper);
        //		pGen6DOF.setAngularLowerLimit(Vector3(0.f, -0.75, SIMD_HALF_PI * 0.8f));
        //		pGen6DOF.setAngularUpperLimit(Vector3(0.f, 0.75, -SIMD_HALF_PI * 0.8f));
        //		pGen6DOF.setAngularLowerLimit(Vector3(0.f, -SIMD_HALF_PI * 0.8f, SIMD_HALF_PI * 1.98f));
        //		pGen6DOF.setAngularUpperLimit(Vector3(0.f, SIMD_HALF_PI * 0.8f,  -SIMD_HALF_PI * 1.98f));

        		
        		
        //		pGen6DOF.setAngularLowerLimit(Vector3(-0.75,-0.5, -0.5));
        //		pGen6DOF.setAngularUpperLimit(Vector3(0.75,0.5, 0.5));
        //		pGen6DOF.setAngularLowerLimit(Vector3(-0.75,0., 0.));
        //		pGen6DOF.setAngularUpperLimit(Vector3(0.75,0., 0.));

		        m_dynamicsWorld.AddConstraint(pGen6DOF, true);
		        pGen6DOF.SetDbgDrawSize(5.0f);
	        }
#endif
#if true
            { // create a ConeTwist constraint

		        Matrix tr = Matrix.CreateTranslation(-10,5,0);

		        RigidBody pBodyA = LocalCreateRigidBody( 1.0f, ref tr, shape);
		        pBodyA.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

		        tr = Matrix.CreateTranslation(-10,-5,0);

		        RigidBody pBodyB = LocalCreateRigidBody(0.0f, ref tr, shape);

		        Matrix frameInA, frameInB;
                frameInA = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
                frameInA.Translation = new Vector3(0, -5, 0);
				frameInB = MathUtil.SetEulerZYX(0, 0, MathUtil.SIMD_HALF_PI);
                frameInB.Translation = new Vector3(0, 5, 0);

		        ConeTwistConstraint pCT = new ConeTwistConstraint(pBodyA, pBodyB, ref frameInA, ref frameInB);
		        pCT.SetLimit(MathUtil.SIMD_QUARTER_PI, MathUtil.SIMD_QUARTER_PI, MathUtil.SIMD_PI * 0.8f, 1.0f,0.3f,1.0f); // soft limit == hard limit
		        m_dynamicsWorld.AddConstraint(pCT, true);
		        pCT.SetDbgDrawSize(5.0f);
	        }
#endif
#if true
            { // Hinge connected to the world, with motor (to hinge motor with new and old constraint solver)
				// WORKS OK
		        Matrix tr = Matrix.Identity;
		        RigidBody pBody = LocalCreateRigidBody( 1.0f, ref tr, shape);
		        pBody.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		        Vector3 btPivotA = new Vector3( 10.0f, 0.0f, 0.0f );
		        Vector3 btAxisA = new Vector3( 0.0f, 0.0f, 1.0f );

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
		Matrix tr = Matrix.CreateTranslation(20,4,0);
		RigidBody pBodyA = LocalCreateRigidBody( 0.0f, ref tr, shape);
		pBodyA.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		// dynamic bodyB (child) below it :
		tr = Matrix.CreateTranslation(20,0,0);
		RigidBody pBodyB = LocalCreateRigidBody(1.0f, ref tr, shape);
		pBodyB.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		// add some (arbitrary) data to build constraint frames
		Vector3 parentAxis = new Vector3(1.0f, 0.0f, 0.0f); 
		Vector3 childAxis = new Vector3(0.0f, 0.0f, 1.0f);
        Vector3 anchor = new Vector3(20.0f, 2.0f, 0.0f);

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

		Matrix tr = Matrix.CreateTranslation(-20f,16f,0f);
        //tr.setIdentity();
        //tr.setOrigin(btVector3(btScalar(-20.), btScalar(16.), btScalar(0.)));
        //tr.getBasis().setEulerZYX(0,0,0);
		RigidBody pBodyA = LocalCreateRigidBody( 0.0f, ref tr, shape);
		pBodyA.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

        //tr.setIdentity();
        //tr.setOrigin(btVector3(btScalar(-10.), btScalar(16.), btScalar(0.)));
        //tr.getBasis().setEulerZYX(0,0,0);
        tr = Matrix.CreateTranslation(-10,16,0);
		RigidBody pBodyB = LocalCreateRigidBody(1.0f, ref tr, shape);
		pBodyB.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

        Matrix frameInA = Matrix.CreateTranslation(10f,0f,0f);
        Matrix frameInB = Matrix.CreateTranslation(0f,0f,0f);

		Generic6DofSpringConstraint pGen6DOFSpring = new Generic6DofSpringConstraint(pBodyA, pBodyB, ref frameInA, ref frameInB, true);
		pGen6DOFSpring.SetLinearUpperLimit(new Vector3(5f, 0f, 0f));
		pGen6DOFSpring.SetLinearLowerLimit(new Vector3(-5f, 0f, 0f));

		pGen6DOFSpring.SetAngularLowerLimit(new Vector3(0f, 0f, -1.5f));
		pGen6DOFSpring.SetAngularUpperLimit(new Vector3(0f, 0f, 1.5f));

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
		Matrix tr = Matrix.CreateTranslation(-20f,4f,0f);
        
        RigidBody pBodyA = LocalCreateRigidBody( 0.0f, ref tr, shape);
		pBodyA.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		// dynamic bodyB (child) below it :
		tr = Matrix.CreateTranslation(-20f,0f,0f);
        RigidBody pBodyB = LocalCreateRigidBody(1.0f, ref tr, shape);
		pBodyB.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		// add some data to build constraint frames
		Vector3 parentAxis = new Vector3(0.0f, 1.0f, 0.0f); 
		Vector3 childAxis = new Vector3(1.0f, 0.0f, 0.0f);
        Vector3 anchor = new Vector3(-20.0f, 0.0f, 0.0f);
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
		Matrix tr = Matrix.CreateTranslation(-20f,-2f,0f);
		RigidBody pBodyA = LocalCreateRigidBody( 1.0f, ref tr, shape);
		pBodyA.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		// dynamic bodyB:
		tr = Matrix.CreateTranslation(-30f,-2f,0f);
		RigidBody pBodyB = LocalCreateRigidBody(10.0f, ref tr, shape);
		pBodyB.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
		// add some data to build constraint frames
		Vector3 axisA = new Vector3(0.0f, 1.0f, 0.0f); 
		Vector3 axisB = new Vector3(0.0f, 1.0f, 0.0f); 
		Vector3 pivotA = new Vector3(-5.0f, 0.0f, 0.0f);
        Vector3 pivotB = new Vector3(5.0f, 0.0f, 0.0f);
		
        spHingeDynAB = new HingeConstraint(pBodyA, pBodyB, ref pivotA, ref pivotB, ref axisA, ref axisB);
        spHingeDynAB.SetLimit(-MathUtil.SIMD_HALF_PI * 0.5f, MathUtil.SIMD_HALF_PI * 0.5f);
		// add constraint to world
		m_dynamicsWorld.AddConstraint(spHingeDynAB, true);
		// draw constraint frames and limits for debugging
		spHingeDynAB.SetDbgDrawSize(5.0f);
	}
#endif
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