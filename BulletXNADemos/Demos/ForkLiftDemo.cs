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
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace BulletXNADemos.Demos
{
    public class ForkLiftDemo : DemoApplication
    {

        //----------------------------------------------------------------------------------------------------------------

        public ForkLiftDemo()
        {
            m_cameraHeight = 4.0f;
            m_minCameraDistance = 3.0f;
            m_maxCameraDistance = 10.0f;
	        m_vehicle = null;
            m_wheelShape = null;
	        m_cameraPosition = new Vector3(30,30,30);
	        m_useDefaultCamera = false;
            string filename = @"C:\users\man\xna-forklift-output.txt";
			//FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
			//BulletGlobals.g_streamWriter = new StreamWriter(filestream);
        }

        //----------------------------------------------------------------------------------------------------------------

        public override void InitializeDemo()
        {
            CollisionShape groundShape = new BoxShape(new Vector3(50, 0.1f, 50));
            //CollisionShape groundShape = new StaticPlaneShape(Vector3.Up, 0f);

            Vector3 wheelDimensions = new Vector3(wheelWidth, wheelRadius, wheelRadius);
            m_wheelShape = new CylinderShapeX(ref wheelDimensions);
            
            m_collisionShapes.Add(groundShape);
	        m_collisionConfiguration = new DefaultCollisionConfiguration();
	        m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);
	        Vector3 worldMin = new Vector3(-1000,-1000,-1000);
	        Vector3 worldMax = new Vector3(1000,1000,1000);
            m_broadphase = new AxisSweep3Internal(ref worldMin, ref worldMax, 0xfffe, 0xffff, 16384, null, false);
            //m_broadphase = new SimpleBroadphase(1000, null);

	        m_constraintSolver = new SequentialImpulseConstraintSolver();
            m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

    	    //m_dynamicsWorld.setGravity(new Vector3(0,0,0));
            Matrix tr = Matrix.CreateTranslation(0,-10,0);

            //either use heightfield or triangle mesh

	        //create ground object
	        LocalCreateRigidBody(0f,ref tr,groundShape);

#if true
            CollisionShape chassisShape = new BoxShape(new Vector3(1.0f, 0.5f, 2.0f));
            //CollisionShape chassisShape = new BoxShape(new Vector3(1.0f, 0.5f, 1.0f));
	        m_collisionShapes.Add(chassisShape);

	        CompoundShape compound = new CompoundShape();
	        m_collisionShapes.Add(compound);
	        //localTrans effectively shifts the center of mass with respect to the chassis
	        Matrix localTrans = Matrix.CreateTranslation(0,1,0);

            compound.AddChildShape(ref localTrans, chassisShape);

            {
                CollisionShape suppShape = new BoxShape(new Vector3(0.5f, 0.1f, 0.5f));
                //localTrans effectively shifts the center of mass with respect to the chassis
                Matrix suppLocalTrans = Matrix.CreateTranslation(0f, 1.0f, 2.5f);
                compound.AddChildShape(ref suppLocalTrans, suppShape);
            }

            tr.Translation = Vector3.Zero;

            m_carChassis = LocalCreateRigidBody(800f, ref tr, compound);//chassisShape);

#endif

            {
#if true
                CollisionShape liftShape = new BoxShape(new Vector3(0.5f, 2.0f, 0.05f));
                m_collisionShapes.Add(liftShape);
                m_liftStartPos = new Vector3(0.0f, 2.5f, 3.05f);
                Matrix liftTrans = Matrix.CreateTranslation(m_liftStartPos);
                m_liftBody = LocalCreateRigidBody(10f, ref liftTrans, liftShape);

                Matrix localA = MathUtil.SetEulerZYX(0f, MathUtil.SIMD_HALF_PI, 0f);
                localA.Translation = new Vector3(0f, 1.0f, 3.05f);

				Matrix localB = MathUtil.SetEulerZYX(0f, MathUtil.SIMD_HALF_PI, 0f);
                localB.Translation = new Vector3(0f, -1.5f, -0.05f);

                m_liftHinge = new HingeConstraint(m_carChassis, m_liftBody, ref localA, ref localB);
				MathUtil.PrintMatrix(BulletGlobals.g_streamWriter,"hinge aFrame",m_liftHinge.GetAFrame());
				MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, "hinge bFrame", m_liftHinge.GetBFrame());

                //		m_liftHinge.setLimit(-LIFT_EPS, LIFT_EPS);
                m_liftHinge.SetLimit(0.0f, 0.0f);
                m_dynamicsWorld.AddConstraint(m_liftHinge, true);

                CollisionShape forkShapeA = new BoxShape(new Vector3(1.0f, 0.1f, 0.1f));
                m_collisionShapes.Add(forkShapeA);
                CompoundShape forkCompound = new CompoundShape();
                m_collisionShapes.Add(forkCompound);
                Matrix forkLocalTrans = Matrix.Identity;
                forkCompound.AddChildShape(ref forkLocalTrans, forkShapeA);

                CollisionShape forkShapeB = new BoxShape(new Vector3(0.1f, 0.02f, 0.6f));
                m_collisionShapes.Add(forkShapeB);
                forkLocalTrans = Matrix.CreateTranslation(-0.9f, -0.08f, 0.7f);
                forkCompound.AddChildShape(ref forkLocalTrans, forkShapeB);

                CollisionShape forkShapeC = new BoxShape(new Vector3(0.1f, 0.02f, 0.6f));
                m_collisionShapes.Add(forkShapeC);
                forkLocalTrans = Matrix.CreateTranslation(0.9f, -0.08f, 0.7f);
                forkCompound.AddChildShape(ref forkLocalTrans, forkShapeC);

                m_forkStartPos = new Vector3(0.0f, 0.6f, 3.2f);
                Matrix forkTrans = Matrix.CreateTranslation(m_forkStartPos);

                m_forkBody = LocalCreateRigidBody(5f, ref forkTrans, forkCompound);

                localA = Matrix.CreateFromYawPitchRoll(0f,0f,MathUtil.SIMD_HALF_PI);
                localA.Translation = new Vector3(0.0f, -1.9f, 0.05f);

                localB = Matrix.CreateFromYawPitchRoll(0f, 0f, MathUtil.SIMD_HALF_PI);
                localB.Translation = new Vector3(0.0f, 0.0f, -0.1f);

                m_forkSlider = new SliderConstraint(m_liftBody, m_forkBody, ref localA, ref localB, true);
                m_forkSlider.SetLowerLinLimit(0.1f);
                m_forkSlider.SetUpperLinLimit(0.1f);
                //		m_forkSlider.setLowerAngLimit(-LIFT_EPS);
                //		m_forkSlider.setUpperAngLimit(LIFT_EPS);
                m_forkSlider.SetLowerAngLimit(0.0f);
                m_forkSlider.SetUpperAngLimit(0.0f);
                m_dynamicsWorld.AddConstraint(m_forkSlider, true);
#endif
#if true

                CompoundShape loadCompound = new CompoundShape(false);
                m_collisionShapes.Add(loadCompound);
                CollisionShape loadShapeA = new BoxShape(new Vector3(2.0f, 0.5f, 0.5f));
                m_collisionShapes.Add(loadShapeA);
                Matrix loadTrans = Matrix.Identity;
                loadCompound.AddChildShape(ref loadTrans, loadShapeA);
                CollisionShape loadShapeB = new BoxShape(new Vector3(0.1f, 1.0f, 1.0f));
                m_collisionShapes.Add(loadShapeB);
                loadTrans = Matrix.CreateTranslation(2.1f, 0.0f, 0.0f);
                loadCompound.AddChildShape(ref loadTrans, loadShapeB);
                CollisionShape loadShapeC = new BoxShape(new Vector3(0.1f, 1.0f, 1.0f));
                m_collisionShapes.Add(loadShapeC);
                loadTrans = Matrix.CreateTranslation(-2.1f, 0.0f, 0.0f);
                loadCompound.AddChildShape(ref loadTrans, loadShapeC);
                m_loadStartPos = new Vector3(0.0f, -3.5f, 7.0f);
                loadTrans = Matrix.CreateTranslation(m_loadStartPos);

                m_loadBody = LocalCreateRigidBody(4f, ref loadTrans, loadCompound);
#endif
            }
            //m_carChassis.setDamping(0.2f, 0.2f);

        	ClientResetScene();

	        /// create vehicle
	        {

                m_vehicleRayCaster = new DefaultVehicleRaycaster(m_dynamicsWorld);
                m_vehicle = new RaycastVehicle(m_tuning, m_carChassis, m_vehicleRayCaster);

                ///never deactivate the vehicle
                m_carChassis.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

                m_dynamicsWorld.AddVehicle(m_vehicle);

                float connectionHeight = 1.2f;

                bool isFrontWheel = true;

                //choose coordinate system
                m_vehicle.SetCoordinateSystem(rightIndex, upIndex, forwardIndex);

                Vector3 connectionPointCS0 = Vector3.Zero;
                //connectionPointCS0 = new Vector3(CUBE_HALF_EXTENTS, connectionHeight, CUBE_HALF_EXTENTS);
                //m_vehicle.addWheel(ref connectionPointCS0, ref wheelDirectionCS0, ref wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

                connectionPointCS0 = new Vector3(CUBE_HALF_EXTENTS - (0.3f * wheelWidth), connectionHeight, 2.0f * CUBE_HALF_EXTENTS - wheelRadius);
                m_vehicle.AddWheel(ref connectionPointCS0, ref wheelDirectionCS0, ref wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

                connectionPointCS0 = new Vector3(-CUBE_HALF_EXTENTS + (0.3f * wheelWidth), connectionHeight, 2.0f * CUBE_HALF_EXTENTS - wheelRadius);
                m_vehicle.AddWheel(ref connectionPointCS0, ref wheelDirectionCS0, ref wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

                //isFrontWheel = false;
                connectionPointCS0 = new Vector3(-CUBE_HALF_EXTENTS + (0.3f * wheelWidth), connectionHeight, -2.0f * CUBE_HALF_EXTENTS + wheelRadius);
                m_vehicle.AddWheel(ref connectionPointCS0, ref wheelDirectionCS0, ref wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

                connectionPointCS0 = new Vector3(CUBE_HALF_EXTENTS - (0.3f * wheelWidth), connectionHeight, -2.0f * CUBE_HALF_EXTENTS + wheelRadius);
                m_vehicle.AddWheel(ref connectionPointCS0, ref wheelDirectionCS0, ref wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

                for (int i = 0; i < m_vehicle.GetNumWheels(); i++)
                {
                    WheelInfo wheel = m_vehicle.GetWheelInfo(i);
                    wheel.m_suspensionStiffness = suspensionStiffness;
                    wheel.m_wheelsDampingRelaxation = suspensionDamping;
                    wheel.m_wheelsDampingCompression = suspensionCompression;
                    wheel.m_frictionSlip = wheelFriction;
                    wheel.m_rollInfluence = rollInfluence;
                }
            }

	        SetCameraDistance(26.0f);
            SetTexturing(true);
            SetShadows(true);

        }

    //----------------------------------------------------------------------------------------------------------------

        public override void ShutdownDemo()
        {
            base.ShutdownDemo();
        }

        //----------------------------------------------------------------------------------------------------------------

	    public void LockLiftHinge()
        {
            if (m_liftHinge != null)
            {
                float hingeAngle = m_liftHinge.GetHingeAngle();
                float lowLim = m_liftHinge.GetLowerLimit();
                float hiLim = m_liftHinge.GetUpperLimit();
                m_liftHinge.EnableAngularMotor(false, 0, 0);
                if (hingeAngle < lowLim)
                {
                    //		m_liftHinge.setLimit(lowLim, lowLim + LIFT_EPS);
                    m_liftHinge.SetLimit(lowLim, lowLim);
                }
                else if (hingeAngle > hiLim)
                {
                    //		m_liftHinge.setLimit(hiLim - LIFT_EPS, hiLim);
                    m_liftHinge.SetLimit(hiLim, hiLim);
                }
                else
                {
                    //		m_liftHinge.setLimit(hingeAngle - LIFT_EPS, hingeAngle + LIFT_EPS);
                    m_liftHinge.SetLimit(hingeAngle, hingeAngle);
                }
            }
            return;
        }

        //----------------------------------------------------------------------------------------------------------------

        public void LockForkSlider()
        {
            if (m_forkSlider != null)
            {
                float linDepth = m_forkSlider.GetLinearPos();
                float lowLim = m_forkSlider.GetLowerLinLimit();
                float hiLim = m_forkSlider.GetUpperLinLimit();
                m_forkSlider.SetPoweredLinMotor(false);
                if (linDepth <= lowLim)
                {
                    m_forkSlider.SetLowerLinLimit(lowLim);
                    m_forkSlider.SetUpperLinLimit(lowLim);
                }
                else if (linDepth > hiLim)
                {
                    m_forkSlider.SetLowerLinLimit(hiLim);
                    m_forkSlider.SetUpperLinLimit(hiLim);
                }
                else
                {
                    m_forkSlider.SetLowerLinLimit(linDepth);
                    m_forkSlider.SetUpperLinLimit(linDepth);
                }
            }
            return;
        }

    //----------------------------------------------------------------------------------------------------------------
        //public override void specialKeyboardUp(Keys key, int x, int y)
        public override void KeyboardCallback(Keys key, int x, int y, GameTime gameTime, bool released, ref KeyboardState newState, ref KeyboardState oldState)
        {
            if (released)
            {
                switch (key)
                {
                    case Keys.Up:
                        {
                            LockForkSlider();
                            gEngineForce = 0f;
                            gBreakingForce = defaultBreakingForce;
                            break;
                        }
                    case Keys.Down:
                        {
                            LockForkSlider();
                            gEngineForce = 0f;
                            gBreakingForce = defaultBreakingForce;
                            break;
                        }
                    case Keys.Left:
                    case Keys.Right:
                        {
                            LockLiftHinge();
                            break;
                        }
                    default:
                        base.KeyboardCallback(key, x, y,gameTime,released,ref newState,ref oldState);
                        break;
                }
            }
            else
            {
                bool shiftHeld = newState.IsKeyDown(Keys.LeftShift) || newState.IsKeyDown(Keys.RightShift);
                if (shiftHeld)
                {
                    switch (key)
                    {
                        case Keys.Left:
                            {

                                m_liftHinge.SetLimit(-MathUtil.SIMD_PI / 16.0f, MathUtil.SIMD_PI / 8.0f);
                                m_liftHinge.EnableAngularMotor(true, -0.1f, 10.0f);
                                break;
                            }
                        case Keys.Right:
                            {

                                m_liftHinge.SetLimit(-MathUtil.SIMD_PI / 16.0f, MathUtil.SIMD_PI / 8.0f);
                                m_liftHinge.EnableAngularMotor(true, 0.1f, 10.0f);
                                break;
                            }
                        case Keys.Up:
                            {
                                m_forkSlider.SetLowerLinLimit(0.1f);
                                m_forkSlider.SetUpperLinLimit(3.9f);
                                m_forkSlider.SetPoweredLinMotor(true);
                                m_forkSlider.SetMaxLinMotorForce(10.0f);
                                m_forkSlider.SetTargetLinMotorVelocity(1.0f);
                                break;
                            }
                        case Keys.Down:
                            {
                                m_forkSlider.SetLowerLinLimit(0.1f);
                                m_forkSlider.SetUpperLinLimit(3.9f);
                                m_forkSlider.SetPoweredLinMotor(true);
                                m_forkSlider.SetMaxLinMotorForce(10.0f);
                                m_forkSlider.SetTargetLinMotorVelocity(-1.0f);
                                break;
                            }

                        default:
                            base.KeyboardCallback(key, x, y, gameTime, released,ref newState,ref oldState);
                            break;
                    }
                }
                else
                {
                    switch (key)
                    {
                        case Keys.Left:
                            {
                                gVehicleSteering += steeringIncrement;
                                if (gVehicleSteering > steeringClamp)
                                    gVehicleSteering = steeringClamp;

                                break;
                            }
                        case Keys.Right:
                            {
                                gVehicleSteering -= steeringIncrement;
                                if (gVehicleSteering < -steeringClamp)
                                    gVehicleSteering = -steeringClamp;

                                break;
                            }
                        case Keys.Up:
                            {
                                gEngineForce = maxEngineForce;
                                gBreakingForce = 0f;
                                break;
                            }
                        case Keys.Down:
                            {
                                gEngineForce = -maxEngineForce;
                                gBreakingForce = 0f;
                                break;
                            }

                        case Keys.F5:
                            m_useDefaultCamera = !m_useDefaultCamera;
                            break;
                        default:
                            base.KeyboardCallback(key, x, y, gameTime, released,ref newState,ref oldState);
                            break;
                    }
                }
            }
        }

        //----------------------------------------------------------------------------------------------------------------

        public override void  ClientMoveAndDisplay(GameTime gameTime)
        {
            {
                int wheelIndex = 2;
                if (m_vehicle != null)
                {
                    m_vehicle.ApplyEngineForce(gEngineForce, wheelIndex);
                    m_vehicle.SetBrake(gBreakingForce, wheelIndex);
                    wheelIndex = 3;
                    m_vehicle.ApplyEngineForce(gEngineForce, wheelIndex);
                    m_vehicle.SetBrake(gBreakingForce, wheelIndex);

                    wheelIndex = 0;
                    m_vehicle.SetSteeringValue(gVehicleSteering, wheelIndex);
                    m_vehicle.ApplyEngineForce(gEngineForce, wheelIndex);
                    m_vehicle.SetBrake(gBreakingForce, wheelIndex);

                    wheelIndex = 1;
                    m_vehicle.SetSteeringValue(gVehicleSteering, wheelIndex);
                    m_vehicle.ApplyEngineForce(gEngineForce, wheelIndex);
                    m_vehicle.SetBrake(gBreakingForce, wheelIndex);

                }
            }

            float dt = GetDeltaTimeMicroseconds() * 0.000001f;
            //float dt = DiscreteDynamicsWorld.s_fixedTimeStep;

            if (m_dynamicsWorld != null)
            {
                //during idle mode, just run 1 simulation step maximum
                int maxSimSubSteps = m_idle ? 1 : 2;
                if (m_idle)
                {
                    dt = 1.0f / 420f;
                }
                int numSimSteps = m_dynamicsWorld.StepSimulation(dt, maxSimSubSteps);

                m_dynamicsWorld.DebugDrawWorld();
            }            
            //simple dynamics world doesn't handle fixed-time-stepping
            //float ms = (float)gameTime.ElapsedGameTime.TotalSeconds;

            /////step the simulation
            //if (m_dynamicsWorld != null)
            //{
            //    m_dynamicsWorld.stepSimulation(ms, 1);
            //    //optional but useful: debug drawing
            //    m_dynamicsWorld.debugDrawWorld();
            //}
        }

        //----------------------------------------------------------------------------------------------------------------

        protected override void RenderSceneAll(GameTime gameTime)
        {
            Vector3 halfExtents = new Vector3(wheelWidth, wheelRadius, wheelRadius);
	        CylinderShapeX wheelShape = new CylinderShapeX(ref halfExtents);
	        Vector3 wheelColor = new Vector3(1,0,0);


            Vector3 worldBoundsMin, worldBoundsMax;
            GetDynamicsWorld().GetBroadphase().GetBroadphaseAabb(out worldBoundsMin, out worldBoundsMax);

            for (int i = 0; i < m_vehicle.GetNumWheels(); i++)
            {
                //synchronize the wheels with the (interpolated) chassis worldtransform
                m_vehicle.UpdateWheelTransform(i, true);
                //draw wheels (cylinders)
                Matrix m = m_vehicle.GetWheelInfo(i).m_worldTransform;
                m_shapeDrawer.DrawXNA(ref m, m_wheelShape, ref wheelColor, GetDebugMode(), ref worldBoundsMin, ref worldBoundsMax, ref m_lookAt, ref m_perspective);
            }
            base.RenderSceneAll(gameTime);
        }

        //----------------------------------------------------------------------------------------------------------------

        public override void UpdateCamera()
        {
        	
	        if(m_useDefaultCamera)
	        {
		        base.UpdateCamera();
		        return;
	        }

	        Matrix chassisWorldTrans;

	        //look at the vehicle
            m_carChassis.GetMotionState().GetWorldTransform(out chassisWorldTrans);
	        m_cameraTargetPosition = chassisWorldTrans.Translation;

	        m_cameraPosition.Y = (15.0f*m_cameraPosition.Y + m_cameraTargetPosition.Y + m_cameraHeight)/16.0f;

	        Vector3 camToObject = m_cameraTargetPosition - m_cameraPosition;

	        //keep distance between min and max distance
	        float cameraDistance = camToObject.Length();
	        float correctionFactor = 0.0f;
	        if (cameraDistance < m_minCameraDistance)
	        {
		        correctionFactor = 0.15f*(m_minCameraDistance-cameraDistance)/cameraDistance;
	        }
	        if (cameraDistance > m_maxCameraDistance)
	        {
		        correctionFactor = 0.15f*(m_maxCameraDistance-cameraDistance)/cameraDistance;
	        }
	        m_cameraPosition -= correctionFactor*camToObject;


            m_lookAt = Matrix.CreateLookAt(m_cameraPosition, m_cameraTargetPosition, m_cameraUp);
 
        }

        //----------------------------------------------------------------------------------------------------------------

        public override void  ClientResetScene()
        {
            base.ClientResetScene();
            gVehicleSteering = 0.0f;
	        gBreakingForce = defaultBreakingForce;
	        gEngineForce = 0.0f;
            Matrix ident = Matrix.Identity;
	        m_carChassis.SetCenterOfMassTransform(ref ident);
            Vector3 zero = Vector3.Zero;
	        m_carChassis.SetLinearVelocity(ref zero);
            m_carChassis.SetAngularVelocity(ref zero);
	        m_dynamicsWorld.GetBroadphase().GetOverlappingPairCache().CleanProxyFromPairs(m_carChassis.GetBroadphaseHandle(),GetDynamicsWorld().GetDispatcher());
	        if (m_vehicle != null)
	        {
		        m_vehicle.ResetSuspension();
		        for (int i=0;i<m_vehicle.GetNumWheels();i++)
		        {
			        //synchronize the wheels with the (interpolated) chassis worldtransform
			        m_vehicle.UpdateWheelTransform(i,true);
		        }
	        }
            if (m_liftBody != null)
            {
                Matrix liftTrans = Matrix.CreateTranslation(m_liftStartPos);
                m_liftBody.Activate();
                m_liftBody.SetCenterOfMassTransform(ref liftTrans);
                m_liftBody.SetLinearVelocity(ref zero);
                m_liftBody.SetAngularVelocity(ref zero);
            }

            if (m_forkBody != null)
            {
                Matrix forkTrans = Matrix.CreateTranslation(m_forkStartPos);
                m_forkBody.Activate();
                m_forkBody.SetCenterOfMassTransform(ref forkTrans);
                m_forkBody.SetLinearVelocity(ref zero);
                m_forkBody.SetAngularVelocity(ref zero);
            }

            if (m_liftHinge != null)
            {
                //	m_liftHinge.setLimit(-LIFT_EPS, LIFT_EPS);
                m_liftHinge.SetLimit(0.0f, 0.0f);
                m_liftHinge.EnableAngularMotor(false, 0, 0);
            }

            if (m_forkSlider != null)
            {
                m_forkSlider.SetLowerLinLimit(0.1f);
                m_forkSlider.SetUpperLinLimit(0.1f);
                m_forkSlider.SetPoweredLinMotor(false);
            }

            if (m_loadBody != null)
            {
                Matrix loadTrans = Matrix.CreateTranslation(m_loadStartPos);
                m_loadBody.Activate();
                m_loadBody.SetCenterOfMassTransform(ref loadTrans);
                m_loadBody.SetLinearVelocity(ref zero);
                m_loadBody.SetAngularVelocity(ref zero);
            }
        }
        static void Main(string[] args)
        {
            using (ForkLiftDemo game = new ForkLiftDemo())
            {
                game.Run();
            }
        }

    //----------------------------------------------------------------------------------------------------------------
        
        private RigidBody m_carChassis;
	
        private RigidBody m_liftBody;
	    private Vector3	m_liftStartPos;
	    private HingeConstraint m_liftHinge;

	    private RigidBody m_forkBody;
	    private Vector3	m_forkStartPos;
	    private SliderConstraint m_forkSlider;

	    private RigidBody m_loadBody;
	    private Vector3	m_loadStartPos;

        private bool m_useDefaultCamera;

        private float m_cameraHeight;

        private float m_minCameraDistance;
        private float m_maxCameraDistance;

        private VehicleTuning m_tuning = new VehicleTuning();
        private IVehicleRaycaster m_vehicleRayCaster;
        private RaycastVehicle m_vehicle;
        private CollisionShape m_wheelShape;

        private float CUBE_HALF_EXTENTS = 1f;
        private float	gEngineForce = 0.0f;

        private float	defaultBreakingForce = 10.0f;
        private float	gBreakingForce = 100.0f;

        private float	maxEngineForce = 1000.0f;//this should be engine/velocity dependent
        private float	maxBreakingForce = 100.0f;

        private float	gVehicleSteering = 0.0f;
        private float	steeringIncrement = 0.04f;
        private float	steeringClamp = 0.3f;
        private float	wheelRadius = 0.5f;
        private float	wheelWidth = 0.4f;
        private float	wheelFriction = 1000;//1e30f;
        private float	suspensionStiffness = 20.0f;
        private float	suspensionDamping = 2.3f;
        private float	suspensionCompression = 4.4f;
        private float	rollInfluence = 0.1f;//1.0f;
        private float suspensionRestLength = 0.6f;



        private int rightIndex = 0;
		private int upIndex = 1;
		private int forwardIndex = 2;
		private Vector3 wheelDirectionCS0 = new Vector3(0,-1,0);
		private Vector3 wheelAxleCS = new Vector3(-1,0,0);
    }
}
