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
using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;
using BulletXNA;
using BulletXNA.BulletCollision.BroadphaseCollision;
using BulletXNA.BulletCollision.CollisionDispatch;
using BulletXNA.BulletCollision.CollisionShapes;
using BulletXNA.BulletDynamics.ConstraintSolver;
using BulletXNA.BulletDynamics.Dynamics;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace BulletXNADemos.Demos
{
    public class DemoApplication : Microsoft.Xna.Framework.Game
    {
        protected IProfileManager m_profileManager;
        protected IProfileIterator m_profileIterator;
        protected IDebugDraw m_debugDraw;
        protected GraphicsDeviceManager m_graphics;
        protected SpriteBatch m_spriteBatch;
        protected IList<CollisionShape> m_collisionShapes = new List<CollisionShape>();
        protected IBroadphaseInterface m_broadphase;
        protected CollisionDispatcher m_dispatcher;
        protected IConstraintSolver m_constraintSolver;
        protected DefaultCollisionConfiguration m_collisionConfiguration;
        protected DynamicsWorld m_dynamicsWorld;
        protected Vector3 m_defaultGravity = new Vector3(0, -10, 0);

        public const float STEPSIZEROTATE = MathUtil.SIMD_PI / 3f; // 60 deg a second
        public const float STEPSIZETRANSLATE = 20f; 

        public const float mousePickClamping = 30f;

        public static int gPickingConstraintId = 0;
        public static Vector3 gOldPickingPos;
        public static float gOldPickingDist = 0f;
        public static RigidBody pickedBody = null;//for deactivation state

        public static int gNumDeepPenetrationChecks;

        public static int gNumSplitImpulseRecoveries;
        public static int gNumGjkChecks;
        public static int gNumAlignedAllocs;
        public static int gNumAlignedFree;
        public static int gTotalBytesAlignedAllocs;

        public static int gNumClampedCcdMotions;

        // public for now to test camera acccess in xna shape draw.
        public Matrix m_lookAt = Matrix.Identity;
        public Matrix m_perspective = Matrix.Identity;

        ///constraint for mouse picking
        protected TypedConstraint m_pickConstraint;

        protected CollisionShape m_shootBoxShape;

        protected float m_cameraDistance;
        protected DebugDrawModes m_debugMode;

        protected float m_pitch;
        protected float m_yaw;

        protected Vector3 m_cameraPosition;
        protected Vector3 m_cameraTargetPosition;//look at

        protected float m_scaleBottom;
        protected float m_scaleFactor;
        protected Vector3 m_cameraUp;
        protected int m_forwardAxis;

        protected int m_glutScreenWidth;
        protected int m_glutScreenHeight;

        protected float m_ShootBoxInitialSpeed;

        protected bool m_stepping;
        protected bool m_singleStep;
        protected bool m_idle;
        protected Keys m_lastKey;
        protected KeyboardState m_lastKeyboardState;
        protected MouseState m_lastMouseState;
        protected GamePadState m_lastGamePadState;

        protected XNA_ShapeDrawer m_shapeDrawer;
        protected bool m_enableshadows;
        protected Vector3 m_lightDirection;
        protected Vector3 m_lightPosition;
        protected Matrix m_lightView;
        protected Matrix m_lightProjection;

        protected float m_lightPower = 0.5f;
        protected Vector4 m_ambientLight = new Vector4(0.1f,0.1f,0.1f,1f);
        protected Vector4 m_diffuseLight = Color.LightGray.ToVector4();

        protected float m_nearClip;
        protected float m_farClip;
        protected float m_aspect;



        //----------------------------------------------------------------------------------------------

        protected void UpdateLights()
        {
            
            //Vector3 target = m_lightPosition + m_lightDirection * 10;
            Vector3 target = Vector3.Zero;
            float aspect = m_glutScreenWidth / m_glutScreenHeight;
            float fov = MathHelper.ToRadians(40.0f);

            m_lightView = Matrix.CreateLookAt(m_lightPosition, target, Vector3.Up);
            m_lightProjection = Matrix.CreatePerspectiveFieldOfView(fov, aspect, 1f, 500f);
        }

        //----------------------------------------------------------------------------------------------

        protected void ShowProfileInfo(ref float xOffset, ref float yStart, float yIncr)
        {
        }

        //----------------------------------------------------------------------------------------------

        protected virtual void RenderSceneAll(GameTime gameTime)
        {
            if (m_enableshadows)
            {
                //renderScenePass(1,gameTime);
            }
            m_dynamicsWorld.DebugDrawWorld();
            RenderScenePass(0, gameTime);
            Vector3 location = new Vector3(10, 10, 0);
            Vector3 colour = new Vector3(1,1,1);
            m_shapeDrawer.DrawText(String.Format("Memory [{0}]", System.GC.GetTotalMemory(false)), location, colour);
            int	xOffset = 10;
            int yStart = 20;
		    int yIncr = 15;

            ShowProfileInfo(xOffset, yStart, yIncr);


            m_shapeDrawer.RenderOthers(gameTime, ref m_lookAt, ref m_perspective);
        }

        protected virtual void RenderScenePass(int pass, GameTime gameTime)
        {
	        Matrix m = Matrix.Identity;
	        Matrix rot = Matrix.Identity;
	        int numObjects = m_dynamicsWorld.GetNumCollisionObjects();
	        Vector3 wireColor = new Vector3(1,0,0);

            for(int i=0;i<numObjects;i++)
	        {
		        CollisionObject colObj=m_dynamicsWorld.GetCollisionObjectArray()[i];
		        RigidBody body = RigidBody.Upcast(colObj);
		        if(body != null && body.GetMotionState() != null)
		        {
			        DefaultMotionState myMotionState = (DefaultMotionState)body.GetMotionState();
                    //myMotionState.m_graphicsWorldTrans.getOpenGLMatrix(m);
                    m = myMotionState.m_graphicsWorldTrans;
			        rot=MathUtil.BasisMatrix(ref myMotionState.m_graphicsWorldTrans);
		        }
		        else
		        {
                    //colObj.getWorldTransform().getOpenGLMatrix(m);
			        rot=MathUtil.BasisMatrix(colObj.GetWorldTransform());
		        }
		        wireColor = new Vector3(1.0f,1.0f,0.5f); //wants deactivation
		        if((i&1) != 0) wireColor= new Vector3(0f,0f,1f);
		        ///color differently for active, sleeping, wantsdeactivation states
		        if (colObj.GetActivationState() == ActivationState.ACTIVE_TAG) //active
		        {
			        if ((i & 1) != 0)
			        {
				        wireColor += new Vector3(1f,0f,0f);
			        }
			        else
			        {			
				        wireColor += new Vector3(.5f,0f,0f);
			        }
		        }
		        if(colObj.GetActivationState()==ActivationState.ISLAND_SLEEPING) //ISLAND_SLEEPING
		        {
                    if ((i & 1) != 0)
                    {
				        wireColor += new Vector3 (0f,1f, 0f);
			        }
			        else
			        {
				        wireColor += new Vector3(0f,05f,0f);
			        }
		        }

                Vector3 aabbMin, aabbMax;
		        m_dynamicsWorld.GetBroadphase().GetBroadphaseAabb(out aabbMin,out aabbMax);

                aabbMin -= MathUtil.MAX_VECTOR;
                aabbMax += MathUtil.MAX_VECTOR;
        //		printf("aabbMin=(%f,%f,%f)\n",aabbMin.getX(),aabbMin.getY(),aabbMin.getZ());
        //		printf("aabbMax=(%f,%f,%f)\n",aabbMax.getX(),aabbMax.getY(),aabbMax.getZ());
        //		m_dynamicsWorld.getDebugDrawer().drawAabb(aabbMin,aabbMax,btVector3(1,1,1));



		        switch(pass)
		        {
                    case 0:
                        {
                            m_shapeDrawer.DrawXNA(ref m, colObj.GetCollisionShape(), ref wireColor, GetDebugMode(), ref aabbMin, ref aabbMax, ref m_lookAt, ref m_perspective);
                            break;
                        }
                    case 1:
                        {
                            Vector3 shadow = Vector3.Transform(m_lightDirection, rot);
                            m_shapeDrawer.DrawShadow(ref m, ref shadow, colObj.GetCollisionShape(), ref aabbMin, ref aabbMax);
                            break;
                        }
		            case	2:
                        {
                            Vector3 adjustedWireColor = wireColor * 0.3f;
                            m_shapeDrawer.DrawXNA(ref m,colObj.GetCollisionShape(),ref adjustedWireColor,0,ref aabbMin,ref aabbMax,ref m_lookAt,ref m_perspective);
                            break;
                        }
		        }
	        }

            switch (pass)
            {
                case 0:
                    {
						//m_shapeDrawer.RenderShadow(gameTime, ref m_lookAt, ref m_perspective);
                        m_shapeDrawer.RenderStandard(gameTime, ref m_lookAt, ref m_perspective);
                        m_shapeDrawer.RenderDebugLines(gameTime, ref m_lookAt, ref m_perspective);
                        break;
                    }
                case 1:
                    {
						//m_shapeDrawer.RenderShadow(gameTime, ref m_lookAt, ref m_perspective);
                        break;
                    }
                case 2:
                    {
                        m_shapeDrawer.RenderStandard(gameTime, ref m_lookAt, ref m_perspective);
                        break;
                    }
            }
        }

        //----------------------------------------------------------------------------------------------

        public DemoApplication()
        {
            m_dynamicsWorld = null;
            m_pickConstraint = null;
            m_shootBoxShape = null;
            m_cameraDistance = 30f;
            m_debugMode = 0;
            m_pitch =(20f/360f)*MathUtil.SIMD_2_PI;
            m_yaw = 0f;
            m_cameraPosition = Vector3.Zero;
            m_cameraTargetPosition = Vector3.Zero;
            m_scaleBottom = 0.5f;
            m_scaleFactor = 2f;
            m_cameraUp = Vector3.Up;
            m_forwardAxis = 2;
            m_glutScreenWidth = 0;
            m_glutScreenHeight = 0;
            m_ShootBoxInitialSpeed = 40f;
            m_stepping = true;
            m_singleStep = false;
            m_idle = false;
            m_enableshadows = true;
            m_lightPosition = new Vector3(5,5,5);
            //m_lightDirection = Vector3.Down;
            m_lightDirection = new Vector3(.5f, -.5f, .5f);
            m_lightDirection.Normalize();

            //#ifndef BT_NO_PROFILE
            //    m_profileIterator = CProfileManager::Get_Iterator();
            //#endif //BT_NO_PROFILE
            
            Content.RootDirectory = "Content";
            m_graphics = new GraphicsDeviceManager(this);
            m_graphics.PreferredBackBufferWidth = 800;
            m_graphics.PreferredBackBufferHeight = 600;


            SetSize(m_graphics.PreferredBackBufferWidth,m_graphics.PreferredBackBufferHeight);
            m_nearClip = 1f;
            m_farClip = 1000f;

            m_aspect = m_glutScreenWidth / m_glutScreenHeight;
            m_perspective = Matrix.CreatePerspectiveFieldOfView(MathHelper.ToRadians(40.0f),m_aspect, m_nearClip, m_farClip);
        }

        //----------------------------------------------------------------------------------------------

        public virtual void Cleanup()
        {
            //#ifndef BT_NO_PROFILE
            //    CProfileManager::Release_Iterator(m_profileIterator);
            //#endif //BT_NO_PROFILE

            m_shootBoxShape = null;
            m_shapeDrawer = null;
        }

        //----------------------------------------------------------------------------------------------

        public Vector3 GetLightDirection()
        {
            return m_lightDirection;
        }

        //----------------------------------------------------------------------------------------------

        public Vector3 GetLightPosition()
        {
            return m_lightPosition;
        }

        //----------------------------------------------------------------------------------------------

        public Matrix GetLightViewMatrix()
        {
            return m_lightView;
        }

        //----------------------------------------------------------------------------------------------

        public Matrix GetLightProjectionMatrix()
        {
            return m_lightProjection;
        }

        //----------------------------------------------------------------------------------------------

        public float GetLightPower()
        {
            return m_lightPower;
        }
        
        //----------------------------------------------------------------------------------------------

        public Vector4 GetAmbientLight()
        {
            return m_ambientLight;
        }
        
        //----------------------------------------------------------------------------------------------

        public Vector4 GetDiffuseLight()
        {
            return m_diffuseLight;
        }

        //----------------------------------------------------------------------------------------------
        public DynamicsWorld GetDynamicsWorld()
        {
            return m_dynamicsWorld;
        }

        //----------------------------------------------------------------------------------------------

        public virtual void SetDrawClusters(bool drawClusters)
        {

        }

        //----------------------------------------------------------------------------------------------

        public void OverrideXNAShapeDrawer(XNA_ShapeDrawer shapeDrawer)
        {
        }

        //----------------------------------------------------------------------------------------------

        public void SetOrthographicProjection()
        {
        }

        //----------------------------------------------------------------------------------------------

        public void ResetPerspectiveProjection()
        {
        }

        //----------------------------------------------------------------------------------------------

        public bool SetTexturing(bool enable)
        {
            m_shapeDrawer.EnableTexture(enable);
            return m_shapeDrawer.HasTextureEnabled();
        }

        //----------------------------------------------------------------------------------------------

        public bool SetShadows(bool enable)
        {
            bool p = m_enableshadows;
            m_enableshadows = enable;
            return (p);
        }

        //----------------------------------------------------------------------------------------------

        public bool GetTexturing()
        {
            return m_shapeDrawer.HasTextureEnabled();
        }

        //----------------------------------------------------------------------------------------------

        public bool GetShadows()
        {
            return m_enableshadows;
        }

        //----------------------------------------------------------------------------------------------

        public DebugDrawModes GetDebugMode()
        {
            return m_debugMode;
        }

        //----------------------------------------------------------------------------------------------

        public void SetDebugMode(DebugDrawModes mode)
        {
            m_debugMode = mode;
        }

        //----------------------------------------------------------------------------------------------

        public void SetAzi(float azi)
        {
            m_yaw = azi;
        }

        //----------------------------------------------------------------------------------------------

        public void SetCameraUp(Vector3 camUp)
        {
            m_cameraUp = camUp;
        }

        //----------------------------------------------------------------------------------------------
        
        public void SetCameraForwardAxis(int axis)
        {
            m_forwardAxis = axis;
        }

        //----------------------------------------------------------------------------------------------

        public virtual void InitializeDemo()
        {
        }

        //----------------------------------------------------------------------------------------------

        public virtual void ShutdownDemo()
        {
            //delete collision shapes
            for (int j = 0; j < m_collisionShapes.Count; j++)
            {
                m_collisionShapes[j].Cleanup();
            }

            //delete dynamics world
            m_dynamicsWorld.Cleanup();

            //delete solver
            m_constraintSolver.Cleanup();

            //delete broadphase
            m_broadphase.Cleanup();

            //delete dispatcher
            m_dispatcher.Cleanup();

            m_collisionConfiguration.Cleanup();

        }

        //----------------------------------------------------------------------------------------------

        public void ToggleIdle()
        {
            m_idle = !m_idle;
        }

        //----------------------------------------------------------------------------------------------

        public virtual void UpdateCamera()
        {
            float rele = m_pitch;
            float razi = m_yaw;

            Quaternion rot = Quaternion.CreateFromAxisAngle(m_cameraUp, razi);
            
            Vector3 eyePos = Vector3.Zero;
            MathUtil.VectorComponent(ref eyePos, m_forwardAxis, -m_cameraDistance);

            Vector3 forward = eyePos;
            if (forward.LengthSquared() < MathUtil.SIMD_EPSILON)
            {
                forward = Vector3.Forward;
            }
            Vector3 right = Vector3.Cross(m_cameraUp, Vector3.Normalize(forward));
            Quaternion roll = Quaternion.CreateFromAxisAngle(right, -rele);
            rot.Normalize();
            roll.Normalize();

            Matrix m1 = Matrix.CreateFromQuaternion(rot);
            Matrix m2 = Matrix.CreateFromQuaternion(roll);
            Matrix m3 = m2 * m1;

            Vector3 eyePos2 = eyePos;

            eyePos = Vector3.Transform(eyePos, (rot * roll));
            eyePos2 = Vector3.Transform(eyePos2, m3);

            //m_cameraTargetPosition = m_cameraPosition + eyePos;
            m_cameraPosition = eyePos;

            m_cameraPosition += m_cameraTargetPosition;

            if (m_glutScreenWidth == 0 && m_glutScreenHeight == 0)
                return;

            m_lookAt = Matrix.CreateLookAt(m_cameraPosition, m_cameraTargetPosition, m_cameraUp);
                        
            int ibreak = 0;
        }

        //----------------------------------------------------------------------------------------------

        public Vector3 GetCameraPosition()
        {
            return m_cameraPosition;
        }
        
        //----------------------------------------------------------------------------------------------

        public Vector3 GetCameraTargetPosition()
        {
            return m_cameraTargetPosition;
        }

        //----------------------------------------------------------------------------------------------

        public float GetDeltaTimeMicroseconds()
        {
            //#ifdef USE_BT_CLOCK
            //        btScalar dt = m_clock.getTimeMicroseconds();
            //        m_clock.reset();
            //        return dt;
            //#else
            //        return btScalar(16666.);
            //#endif
            return 16666f;
        }

        ///glut callbacks
        //----------------------------------------------------------------------------------------------

        public float GetCameraDistance()
        {
            return m_cameraDistance;
        }

        //----------------------------------------------------------------------------------------------

        public void SetCameraDistance(float dist)
        {
            m_cameraDistance = dist;
        }
        //----------------------------------------------------------------------------------------------

        public void MoveAndDisplay(GameTime gameTime)
        {
            if (!m_idle)
            {
                ClientMoveAndDisplay(gameTime);
            }

        }

        //----------------------------------------------------------------------------------------------

        public virtual void ClientMoveAndDisplay(GameTime gameTime)
        {
            //simple dynamics world doesn't handle fixed-time-stepping
            float ms = (float)gameTime.ElapsedGameTime.TotalSeconds;

            ///step the simulation
            if (m_dynamicsWorld != null)
            {
                m_dynamicsWorld.StepSimulation(ms, 1);
            }

            //renderme();
        }

        //----------------------------------------------------------------------------------------------

        public virtual void ClientResetScene()
        {
            if (BulletGlobals.g_streamWriter != null)
            {
                BulletGlobals.g_streamWriter.WriteLine("ClientResetScene");
            }

        //#ifdef SHOW_NUM_DEEP_PENETRATIONS
	        gNumDeepPenetrationChecks = 0;
	        gNumGjkChecks = 0;
        //#endif //SHOW_NUM_DEEP_PENETRATIONS

	        gNumClampedCcdMotions = 0;
	        int numObjects = 0;

	        if (m_dynamicsWorld != null)
	        {
                // Prefer a better place for this...
                m_dynamicsWorld.SetDebugDrawer(m_debugDraw);

		        numObjects = m_dynamicsWorld.GetNumCollisionObjects();
	        }

            IList<CollisionObject> copyArray = m_dynamicsWorld.GetCollisionObjectArray();

	        for (int i=0;i<numObjects;i++)
	        {
		        CollisionObject colObj = copyArray[i];
		        RigidBody body = RigidBody.Upcast(colObj);
		        if (body != null)
		        {
			        if (body.GetMotionState() != null)
			        {
				        DefaultMotionState myMotionState = (DefaultMotionState)body.GetMotionState();
				        myMotionState.m_graphicsWorldTrans = myMotionState.m_startWorldTrans;
				        body.SetCenterOfMassTransform(ref myMotionState.m_graphicsWorldTrans );
				        colObj.SetInterpolationWorldTransform(ref myMotionState.m_startWorldTrans );
				        colObj.ForceActivationState(ActivationState.ACTIVE_TAG);
				        colObj.Activate();
				        colObj.SetDeactivationTime(0);
				        //colObj.setActivationState(WANTS_DEACTIVATION);
			        }
			        //removed cached contact points (this is not necessary if all objects have been removed from the dynamics world)
                    m_dynamicsWorld.GetBroadphase().GetOverlappingPairCache().CleanProxyFromPairs(colObj.GetBroadphaseHandle(),GetDynamicsWorld().GetDispatcher());

			        if (!body.IsStaticObject())
			        {
                        Vector3 zero = Vector3.Zero;
				        body.SetLinearVelocity(ref zero);
				        body.SetAngularVelocity(ref zero);
			        }
		        }
	        }

	        ///reset some internal cached data in the broadphase
	        m_dynamicsWorld.GetBroadphase().ResetPool(GetDynamicsWorld().GetDispatcher());
	        m_dynamicsWorld.GetConstraintSolver().Reset();
        }

        //----------------------------------------------------------------------------------------------

        ///Demo functions
        public virtual void SetShootBoxShape()
        {
            if (m_shootBoxShape == null)
            {
                //#define TEST_UNIFORM_SCALING_SHAPE 1
#if TEST_UNIFORM_SCALING_SHAPE
			    ConvexShape childShape = new BoxShape(new Vector3(1f,1f,1f));
			    m_shootBoxShape = new UniformScalingShape(childShape,0.5f);
#else
                //m_shootBoxShape = new SphereShape(1f);//BoxShape(btVector3(1.f,1.f,1.f));
                m_shootBoxShape = new BoxShape(new Vector3(0.5f, 0.5f, 0.5f));

#endif//
            }
        }

        //----------------------------------------------------------------------------------------------

        public void ShootBox(Vector3 destination)
        {
            if (m_dynamicsWorld != null)
            {
                float mass = 1f;
                Matrix startTransform = Matrix.Identity;
                Vector3 camPos = GetCameraPosition();
                startTransform.Translation = camPos;

                SetShootBoxShape();
                RigidBody body = LocalCreateRigidBody(mass, ref startTransform, m_shootBoxShape);
                body.SetLinearFactor(Vector3.One);
                Vector3 linVel = destination - camPos;
                linVel.Normalize();
                linVel *= m_ShootBoxInitialSpeed;

                Matrix newMatrix = Matrix.CreateFromQuaternion(new Quaternion(0,0,0,1));
                newMatrix.Translation = camPos;
                body.SetWorldTransform(ref newMatrix);
                body.SetLinearVelocity(ref linVel);
                Vector3 temp = Vector3.Zero;
                body.SetAngularVelocity(ref temp);
                body.SetCcdMotionThreshold(1f);
                body.SetCcdSweptSphereRadius(0.2f);
            }
        }

        //----------------------------------------------------------------------------------------------

        public Vector3 GetRayTo(int x, int y)
        {
            float fov = MathHelper.ToRadians(40.0f);

            Vector3 rayFrom = GetCameraPosition();
            Vector3 rayForward = (GetCameraTargetPosition() - GetCameraPosition());
            rayForward.Normalize();
            float farPlane = 10000f;
            rayForward *= farPlane;

            Vector3 rightOffset;
            Vector3 vertical = m_cameraUp;

            Vector3 hor = Vector3.Cross(rayForward, vertical);
            hor.Normalize();
            vertical = Vector3.Cross(hor, rayForward);
            vertical.Normalize();

            float tanfov = (float)Math.Tan(0.5f * fov);

            hor *= 2f * farPlane * tanfov;
            vertical *= 2f * farPlane * tanfov;

            float aspect = 1f;

            if (m_glutScreenWidth > m_glutScreenHeight)
            {
                aspect = m_glutScreenWidth / (float)m_glutScreenHeight;
                hor *= aspect;
            }
            else
            {
                aspect = m_glutScreenHeight / (float)m_glutScreenWidth;
                vertical *= aspect;
            }

            Vector3 rayToCenter = rayFrom + rayForward;
            Vector3 dHor = hor * 1f / (float)m_glutScreenWidth;
            Vector3 dVert = vertical * 1f / (float)m_glutScreenHeight;

            Vector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
            rayTo += x * dHor;
            rayTo -= y * dVert;
            return rayTo;
        }

        //----------------------------------------------------------------------------------------------
        public RigidBody LocalCreateRigidBody(float mass, Matrix startTransform, CollisionShape shape)
        {
            return LocalCreateRigidBody(mass, ref startTransform, shape);
        }

        public RigidBody LocalCreateRigidBody(float mass, Matrix startTransform, CollisionShape shape,bool addToWorld)
        {
            return LocalCreateRigidBody(mass, ref startTransform, shape,addToWorld);
        }


        public RigidBody LocalCreateRigidBody(float mass, ref Matrix startTransform, CollisionShape shape)
        {
            return LocalCreateRigidBody(mass, ref startTransform, shape, true);
        }

        public RigidBody LocalCreateRigidBody(float mass, ref Matrix startTransform, CollisionShape shape,bool addToWorld)
        {
			
            Debug.Assert((shape == null || shape.GetShapeType() != BroadphaseNativeTypes.INVALID_SHAPE_PROXYTYPE));

            //rigidbody is dynamic if and only if mass is non zero, otherwise static
            bool isDynamic = !MathUtil.CompareFloat(mass, 0f);

            Vector3 localInertia = Vector3.Zero;
            if (isDynamic)
            {
                shape.CalculateLocalInertia(mass, out localInertia);
            }
            //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

            //#define USE_MOTIONSTATE 1
            //#ifdef USE_MOTIONSTATE
            DefaultMotionState myMotionState = new DefaultMotionState(startTransform, Matrix.Identity);

            RigidBodyConstructionInfo cInfo = new RigidBodyConstructionInfo(mass, myMotionState, shape, localInertia);

            RigidBody body = new RigidBody(cInfo);

            if (BulletGlobals.g_streamWriter != null)
            {
                BulletGlobals.g_streamWriter.WriteLine("localCreateRigidBody [{0}] startTransform",body.m_debugBodyId);
                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, startTransform);
                BulletGlobals.g_streamWriter.WriteLine("");
            }

            //#else
            //    btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);	
            //    body.setWorldTransform(startTransform);
            //#endif//

            if (addToWorld)
            {
                m_dynamicsWorld.AddRigidBody(body);
            }

            return body;
        }

        //----------------------------------------------------------------------------------------------


        //----------------------------------------------------------------------------------------------

        public virtual void KeyboardCallback(Keys key,int x,int y,GameTime gameTime,bool released,ref KeyboardState newState,ref KeyboardState oldState)
        {
            m_lastKey = 0;
            int keyInt = (int)key;
#if !BT_NO_PROFILE
            if (keyInt >= 0x31 && keyInt <= 0x39)
            {
                int child = keyInt - 0x31;
                if (m_profileIterator != null)
                {
                    m_profileIterator.Enter_Child(child);

                }
            }
            if (keyInt == 0x30)
            {
                if (m_profileIterator != null)
                {
                    m_profileIterator.Enter_Parent();
                }
            }
#endif //BT_NO_PROFILE

            switch (key)
            {
                case Keys.Q:
                    Cleanup();
                    Exit();
                    break;

                case Keys.L: StepLeft((STEPSIZEROTATE * (float)gameTime.ElapsedGameTime.TotalSeconds)); break;
                case Keys.R: StepRight((STEPSIZEROTATE * (float)gameTime.ElapsedGameTime.TotalSeconds)); break;
                case Keys.F: StepFront((STEPSIZEROTATE * (float)gameTime.ElapsedGameTime.TotalSeconds)); break;
                case Keys.B: StepBack((STEPSIZEROTATE * (float)gameTime.ElapsedGameTime.TotalSeconds)); break;
                case Keys.Z: ZoomIn(0.4f); break;
                case Keys.X: ZoomOut(0.4f); break;
                case Keys.I: ToggleIdle(); break;
                case Keys.G: m_enableshadows = !m_enableshadows; break;
                case Keys.U: m_shapeDrawer.EnableTexture(!m_shapeDrawer.EnableTexture(false)); break;
                case Keys.H:
                    if ((m_debugMode & DebugDrawModes.DBG_NoHelpText) != 0)
                        m_debugMode = m_debugMode & (~DebugDrawModes.DBG_NoHelpText);
                    else
                        m_debugMode |= DebugDrawModes.DBG_NoHelpText;
                    break;

                case Keys.W:
                    if ((m_debugMode & DebugDrawModes.DBG_DrawWireframe) != 0)
                        m_debugMode = m_debugMode & (~DebugDrawModes.DBG_DrawWireframe);
                    else
                        m_debugMode |= DebugDrawModes.DBG_DrawWireframe;
                    break;

                case Keys.P:
                    if ((m_debugMode & DebugDrawModes.DBG_ProfileTimings) != 0)
                        m_debugMode = m_debugMode & (~DebugDrawModes.DBG_ProfileTimings);
                    else
                        m_debugMode |= DebugDrawModes.DBG_ProfileTimings;
                    break;

                case Keys.M:
                    if ((m_debugMode & DebugDrawModes.DBG_EnableSatComparison) != 0)
                        m_debugMode = m_debugMode & (~DebugDrawModes.DBG_EnableSatComparison);
                    else
                        m_debugMode |= DebugDrawModes.DBG_EnableSatComparison;
                    break;

                case Keys.N:
                    if ((m_debugMode & DebugDrawModes.DBG_DisableBulletLCP) != 0)
                        m_debugMode = m_debugMode & (~DebugDrawModes.DBG_DisableBulletLCP);
                    else
                        m_debugMode |= DebugDrawModes.DBG_DisableBulletLCP;
                    break;

                case Keys.T:
                    if ((m_debugMode & DebugDrawModes.DBG_DrawText) != 0)
                        m_debugMode = m_debugMode & (~DebugDrawModes.DBG_DrawText);
                    else
                        m_debugMode |= DebugDrawModes.DBG_DrawText;
                    break;
                case Keys.Y:
                    if ((m_debugMode & DebugDrawModes.DBG_DrawFeaturesText) != 0)
                        m_debugMode = m_debugMode & (~DebugDrawModes.DBG_DrawFeaturesText);
                    else
                        m_debugMode |= DebugDrawModes.DBG_DrawFeaturesText;
                    break;
                case Keys.A:
                    if ((m_debugMode & DebugDrawModes.DBG_DrawAabb) != 0)
                        m_debugMode = m_debugMode & (~DebugDrawModes.DBG_DrawAabb);
                    else
                        m_debugMode |= DebugDrawModes.DBG_DrawAabb;
                    break;
                case Keys.C:
                    if ((m_debugMode & DebugDrawModes.DBG_DrawContactPoints) != 0)
                        m_debugMode = m_debugMode & (~DebugDrawModes.DBG_DrawContactPoints);
                    else
                        m_debugMode |= DebugDrawModes.DBG_DrawContactPoints;
                    break;
                //case 'C' : 
                //    if (m_debugMode & DebugDrawModes.DBG_DrawConstraints)
                //        m_debugMode = m_debugMode & (~DebugDrawModes.DBG_DrawConstraints);
                //    else
                //        m_debugMode |= DebugDrawModes.DBG_DrawConstraints;
                //    break;
                //case 'L' : 
                //    if (m_debugMode & DebugDrawModes.DBG_DrawConstraintLimits)
                //        m_debugMode = m_debugMode & (~DebugDrawModes.DBG_DrawConstraintLimits);
                //    else
                //        m_debugMode |= DebugDrawModes.DBG_DrawConstraintLimits;
                //    break;

                case Keys.D:
                    if ((m_debugMode & DebugDrawModes.DBG_NoDeactivation) != 0)
                        m_debugMode = m_debugMode & (~DebugDrawModes.DBG_NoDeactivation);
                    else
                        m_debugMode |= DebugDrawModes.DBG_NoDeactivation;
                    if ((m_debugMode & DebugDrawModes.DBG_NoDeactivation) != 0)
                    {
                        BulletGlobals.gDisableDeactivation = true;
                    }
                    else
                    {
                        BulletGlobals.gDisableDeactivation = false;
                    }
                    break;
                case Keys.O:
                    {
                        m_stepping = !m_stepping;
                        break;
                    }
                // MAN - reintroduce singlesteps here.
                //case Keys.S: clientMoveAndDisplay(); break;
                //    case ' ' : newRandom(); break;
                case Keys.Space:
                    ClientResetScene();
                    break;
                case Keys.D1:
                    {
                        if ((m_debugMode & DebugDrawModes.DBG_EnableCCD) != 0)
                            m_debugMode = m_debugMode & (~DebugDrawModes.DBG_EnableCCD);
                        else
                            m_debugMode |= DebugDrawModes.DBG_EnableCCD;
                        break;
                    }

                case Keys.OemPeriod:
                    {
                        ShootBox(GetRayTo(x, y));//getCameraTargetPosition());
                        break;
                    }

                case Keys.OemPlus:
                    {
                        m_ShootBoxInitialSpeed += 10f;
                        break;
                    }
                case Keys.OemMinus:
                    {
                        m_ShootBoxInitialSpeed -= 10f;
                        break;
                    }
	            case Keys.F1:
		            {

			            break;
		            }

	            case Keys.F2:
		            {

			            break;
		            }
	            case Keys.End:
		            {
                        //int numObj = getDynamicsWorld().getNumCollisionObjects();
                        //if (numObj != 0)
                        //{
                        //    CollisionObject obj = getDynamicsWorld().getCollisionObjectArray()[numObj-1];

                        //    getDynamicsWorld().removeCollisionObject(obj);
                        //    RigidBody body = RigidBody.upcast(obj);
                        //    if (body != null && body.getMotionState() != null)
                        //    {
                        //        body.getMotionState().c;					
                        //    }
                        //    delete obj;


                        //}
                        break;
		            }
	            case Keys.Left : StepLeft((STEPSIZEROTATE * (float)gameTime.ElapsedGameTime.TotalSeconds)); break;
                case Keys.Right: StepRight((STEPSIZEROTATE * (float)gameTime.ElapsedGameTime.TotalSeconds)); break;
                case Keys.Up: StepFront((STEPSIZEROTATE * (float)gameTime.ElapsedGameTime.TotalSeconds)); break;
                case Keys.Down: StepBack((STEPSIZEROTATE * (float)gameTime.ElapsedGameTime.TotalSeconds)); break;
                case Keys.PageUp: ZoomIn(0.4f); break;
                case Keys.PageDown: ZoomOut(0.4f); break;
	            case Keys.Home : ToggleIdle(); break;

                default:
                    //        std::cout << "unused key : " << key << std::endl;
                    break;
            }

            if (GetDynamicsWorld() != null && GetDynamicsWorld().GetDebugDrawer() != null)
            {
                GetDynamicsWorld().GetDebugDrawer().SetDebugMode(m_debugMode);
            }
        }

        //----------------------------------------------------------------------------------------------

        public virtual void SpecialKeyboard(int key, int x, int y)
        {
        }

        //----------------------------------------------------------------------------------------------

        public virtual void SpecialKeyboardUp(int key, int x, int y)
        {
        }

        //----------------------------------------------------------------------------------------------

        public virtual void Reshape(int w, int h)
        {
        }

        //----------------------------------------------------------------------------------------------

        public virtual void MouseFunc(ref MouseState oldMouseState, ref MouseState newMouseState)
        {
            Vector3 rayTo = GetRayTo(newMouseState.X, newMouseState.Y);

            if (WasReleased(ref oldMouseState,ref newMouseState,2))
            {
                ShootBox(rayTo);
            }
            else if (WasReleased(ref oldMouseState,ref newMouseState,1))
            {
                //apply an impulse
                if (m_dynamicsWorld != null)
                {
                    ClosestRayResultCallback rayCallback = new ClosestRayResultCallback(m_cameraPosition, rayTo);
                    m_dynamicsWorld.RayTest(ref m_cameraPosition, ref rayTo, rayCallback);
                    if (rayCallback.HasHit())
                    {
                        RigidBody body = RigidBody.Upcast(rayCallback.m_collisionObject);
                        if (body != null)
                        {
                            body.SetActivationState(ActivationState.ACTIVE_TAG);
                            Vector3 impulse = rayTo;
                            impulse.Normalize();
                            float impulseStrength = 10f;
                            impulse *= impulseStrength;
                            Vector3 relPos = rayCallback.m_hitPointWorld - body.GetCenterOfMassPosition();
                            body.ApplyImpulse(ref impulse, ref relPos);
                        }
                    }
                }
            }
            else if (newMouseState.LeftButton == ButtonState.Pressed)
            {
                //add a point to point constraint for picking
                if (m_dynamicsWorld != null)
                {
                    ClosestRayResultCallback rayCallback = new ClosestRayResultCallback(m_cameraPosition, rayTo);
                    m_dynamicsWorld.RayTest(ref m_cameraPosition, ref rayTo, rayCallback);
                    if (rayCallback.HasHit())
                    {
                        RigidBody body = RigidBody.Upcast(rayCallback.m_collisionObject);
                        if (body != null)
                        {
                            //other exclusions?
                            if (!(body.IsStaticObject() || body.IsKinematicObject()))
                            {
                                pickedBody = body;
                                pickedBody.SetActivationState(ActivationState.DISABLE_DEACTIVATION);


                                Vector3 pickPos = rayCallback.m_hitPointWorld;

                                Vector3 localPivot = Vector3.Transform(pickPos, Matrix.Invert(body.GetCenterOfMassTransform()));

                                Point2PointConstraint p2p = new Point2PointConstraint(body, ref localPivot);
                                p2p.m_setting.m_impulseClamp = mousePickClamping;

                                m_dynamicsWorld.AddConstraint(p2p,true);
                                m_pickConstraint = p2p;

                                //save mouse position for dragging
                                gOldPickingPos = rayTo;

                                Vector3 eyePos = m_cameraPosition;

                                gOldPickingDist = (pickPos - eyePos).Length();

                                //very weak constraint for picking
                                p2p.m_setting.m_tau = 0.1f;
                            }
                        }
                    }
                }

            }
            else if (WasReleased(ref oldMouseState,ref newMouseState,0))
            {
                if (m_pickConstraint != null && m_dynamicsWorld != null)
                {
                    m_dynamicsWorld.RemoveConstraint(m_pickConstraint);
                    m_pickConstraint = null;
                    //printf("removed constraint %i",gPickingConstraintId);
                    m_pickConstraint = null;
                    pickedBody.ForceActivationState(ActivationState.ACTIVE_TAG);
                    pickedBody.SetDeactivationTime(0f);
                    pickedBody = null;
                }
            }
        }

        //----------------------------------------------------------------------------------------------

        public virtual void MouseMotionFunc(ref MouseState mouseState)
        {
            if (m_pickConstraint != null)
            {
                //move the constraint pivot
                Point2PointConstraint p2p = (Point2PointConstraint)(m_pickConstraint);
                if (p2p != null)
                {
                    //keep it at the same picking distance

                    Vector3 newRayTo = GetRayTo(mouseState.X, mouseState.Y);
                    Vector3 eyePos = m_cameraPosition;
                    Vector3 dir = newRayTo - eyePos;
                    dir.Normalize();
                    dir *= gOldPickingDist;

                    Vector3 newPos = eyePos + dir;
                    p2p.SetPivotB(ref newPos);
                }
            }
        }

        //----------------------------------------------------------------------------------------------

        public virtual void DisplayCallback()
        {
        }

        //----------------------------------------------------------------------------------------------

        //public virtual void renderme()
        //{
        //    m_shapeDrawer.startDraw(GraphicsDevice, ref m_lookAt, ref m_perspective);
        //}

        //----------------------------------------------------------------------------------------------

        public void StepLeft(float delta)
        {
            m_yaw -= delta;
        }

        //----------------------------------------------------------------------------------------------

        public void StepRight(float delta)
        {
            m_yaw += delta;
            if (m_yaw >= MathUtil.SIMD_2_PI)
            {
                m_yaw -= MathUtil.SIMD_2_PI;
            } 
        }
        
        //----------------------------------------------------------------------------------------------

        public void StepFront(float delta)
        {
            m_pitch += delta;
            if (m_pitch >= MathUtil.SIMD_2_PI)
            {
                m_pitch -= MathUtil.SIMD_2_PI;
            }
        }

        //----------------------------------------------------------------------------------------------

        public void StepBack(float delta)
        {
            m_pitch -= delta;
            if (m_pitch < 0)
            {
                m_pitch += MathUtil.SIMD_2_PI;
            }
        }

        //----------------------------------------------------------------------------------------------

        public void ZoomIn(float delta)
        {
            m_cameraDistance -= delta; 
            if (m_cameraDistance < 0.1f)
            {
                m_cameraDistance = 0.1f;
            }
        }

        //----------------------------------------------------------------------------------------------

        public void ZoomOut(float delta)
        {
            m_cameraDistance += delta; 
        }

        //----------------------------------------------------------------------------------------------

        public bool IsIdle()
        {
            return m_idle;
        }

        //----------------------------------------------------------------------------------------------

        public void SetIdle(bool idle)
        {
            m_idle = idle;
        }

        //----------------------------------------------------------------------------------------------

        protected override void Initialize()
        {
            base.Initialize();
            InitializeDemo();
        }

        //----------------------------------------------------------------------------------------------

        protected override void Dispose(bool disposing)
        {
            ShutdownDemo();
            base.Dispose(disposing);
        }

        //----------------------------------------------------------------------------------------------

        protected override void Update(GameTime gameTime)
        {
            UpdateCamera();
            UpdateLights();

            KeyboardState currentKeyboardState = Keyboard.GetState();
            GenerateKeyEvents(ref m_lastKeyboardState, ref currentKeyboardState,gameTime);
            m_lastKeyboardState = currentKeyboardState;

            MouseState mouseState = Mouse.GetState();
            GenerateMouseEvents(ref m_lastMouseState, ref mouseState);
            m_lastMouseState = mouseState;


            GamePadState gamePadState = GamePad.GetState(PlayerIndex.One);
            if (gamePadState.IsConnected)
            {
                GenerateGamePadEvents(ref m_lastGamePadState, ref gamePadState,gameTime);
            }

            MoveAndDisplay(gameTime);

        }

        //----------------------------------------------------------------------------------------------
        
        private void GenerateGamePadEvents(ref GamePadState old, ref GamePadState current, GameTime gameTime)
        {
            if (current.ThumbSticks.Right.LengthSquared() > 0)
            {
                Vector2 right = current.ThumbSticks.Right * STEPSIZEROTATE * (float)gameTime.ElapsedGameTime.TotalSeconds;
                StepLeft(right.X);
                StepFront(right.Y);
            }
            if (current.ThumbSticks.Left.LengthSquared() > 0)
            {
                Vector2 left = current.ThumbSticks.Left * STEPSIZETRANSLATE * (float)gameTime.ElapsedGameTime.TotalSeconds;

                Vector3 forward = m_cameraTargetPosition - m_cameraPosition;
                forward.Normalize();

                float rele = m_pitch;
                float razi = m_yaw;

                Vector3 right = Vector3.Cross(forward,m_cameraUp);

                m_cameraPosition += forward * left.Y;
                m_cameraPosition += right * left.X;
            }

            if (current.Triggers.Left > 0f)
            {
                m_cameraPosition.Y -= STEPSIZETRANSLATE * current.Triggers.Left * (float)gameTime.ElapsedGameTime.TotalSeconds;
            }
            if (current.Triggers.Right> 0f)
            {
                m_cameraPosition.Y += STEPSIZETRANSLATE * current.Triggers.Right * (float)gameTime.ElapsedGameTime.TotalSeconds;
            }

            if (current.Buttons.Back == ButtonState.Pressed)
            {
                Cleanup();
                Exit();
            }

			if (current.Buttons.A == ButtonState.Pressed)
			{
				ClientResetScene();
			}


        }


        //----------------------------------------------------------------------------------------------

        static Enum[] keysEnumValues = GetEnumValues(typeof(Keys));
        private void GenerateKeyEvents(ref KeyboardState old, ref KeyboardState current,GameTime gameTime)
        {
            foreach (Keys key in keysEnumValues)
            {
                bool released = WasReleased(ref old,ref current, key);
                if (released || IsHeldKey(ref current,key))
                {
                    KeyboardCallback(key,0,0,gameTime,released,ref current,ref old);
                }
            }
        }
 
        //----------------------------------------------------------------------------------------------
        // workaround from justastro at : http://forums.create.msdn.com/forums/p/1610/157478.aspx
        
        public static Enum[] GetEnumValues(Type enumType)
        {

          if (enumType.BaseType == typeof(Enum))
          {
            FieldInfo[] info = enumType.GetFields(BindingFlags.Static | BindingFlags.Public);
            Enum[] values = new Enum[info.Length];
            for (int i=0; i<values.Length; ++i)
            {
              values[i] = (Enum)info[i].GetValue(null);
            }
            return values;
          }
          else
          {
             throw new Exception("Given type is not an Enum type");
          }
        }

        //----------------------------------------------------------------------------------------------
        // This is a way of generating 'pressed' events for keys that we want to hold down
        private bool IsHeldKey(ref KeyboardState current,Keys key)
        {
            return (current.IsKeyDown(key) && ((key == Keys.Left || key == Keys.Right || key == Keys.Up || 
                key == Keys.Down || key == Keys.PageUp || key == Keys.PageDown)));
        }
        //----------------------------------------------------------------------------------------------

        private bool WasReleased(ref KeyboardState old, ref KeyboardState current, Keys key)
        {
            // figure out if the key was released between states.
            return old.IsKeyDown(key) && !current.IsKeyDown(key);
        }

        //----------------------------------------------------------------------------------------------

        private bool WasReleased(ref MouseState old, ref MouseState current, int buttonIndex)
        {
            if (buttonIndex == 0)
            {
                return old.LeftButton == ButtonState.Pressed && current.LeftButton == ButtonState.Released;
            }
            if (buttonIndex == 1)
            {
                return old.MiddleButton == ButtonState.Pressed && current.MiddleButton == ButtonState.Released;
            }
            if (buttonIndex == 2)
            {
                return old.RightButton == ButtonState.Pressed && current.RightButton == ButtonState.Released;
            }
            return false;
        }

       //----------------------------------------------------------------------------------------------

        public void GenerateMouseEvents(ref MouseState oldState, ref MouseState newState)
        {
            MouseFunc(ref oldState, ref newState);
        }
        
        
        
        //----------------------------------------------------------------------------------------------

        protected override void Draw(GameTime gameTime)
        {
            RenderSceneAll(gameTime);
            base.Draw(gameTime);
        }
        //----------------------------------------------------------------------------------------------
        public void ShowProfileInfo(int xOffset, int yStart, int yIncr)
        {
            if (m_profileManager != null)
            {
                double time_since_reset = 0f;
                if (!m_idle)
                {
                    time_since_reset = m_profileManager.Get_Time_Since_Reset();
                }


                {
                    //recompute profiling data, and store profile strings
                    String blockTime;
                    double totalTime = 0;

                    int frames_since_reset = m_profileManager.Get_Frame_Count_Since_Reset();

                    m_profileIterator.First();

                    double parent_time = m_profileIterator.Is_Root() ? time_since_reset : m_profileIterator.Get_Current_Parent_Total_Time();

                    {
                        blockTime = String.Format("--- Profiling: {0} (total running time: {1:0.000} ms) ---", m_profileIterator.Get_Current_Parent_Name(), parent_time);
                        DisplayProfileString(xOffset, yStart, blockTime);
                        yStart += yIncr;

                        blockTime = "press number (1,2...) to display child timings, or 0 to go up to parent";
                        DisplayProfileString(xOffset, yStart, blockTime);
                        yStart += yIncr;

                    }


                    double accumulated_time = 0f;

                    for (int i = 0; !m_profileIterator.Is_Done(); m_profileIterator.Next())
                    {
                        double current_total_time = m_profileIterator.Get_Current_Total_Time();
                        accumulated_time += current_total_time;
                        double fraction = parent_time > MathUtil.SIMD_EPSILON ? (current_total_time / parent_time) * 100 : 0f;

                        blockTime = String.Format("{0} -- {1} ({2:0.00} %%) :: {3:0.000} ms / frame ({4} calls)",
                            ++i, m_profileIterator.Get_Current_Name(), fraction,
                            (current_total_time / (double)frames_since_reset), m_profileIterator.Get_Current_Total_Calls());
                        DisplayProfileString(xOffset, yStart, blockTime);
                        yStart += yIncr;
                        totalTime += current_total_time;
                    }

                    blockTime = String.Format("{0} ({1:0.000}%) :: {2:0.000} ms", "Unaccounted",
                        // (min(0, time_since_reset - totalTime) / time_since_reset) * 100);
                        parent_time > MathUtil.SIMD_EPSILON ? ((parent_time - accumulated_time) / parent_time) * 100 : 0f, parent_time - accumulated_time);

                    DisplayProfileString(xOffset, yStart, blockTime);
                    yStart += yIncr;



                    blockTime = "-------------------------------------------------";
                    DisplayProfileString(xOffset, yStart, blockTime);
                    yStart += yIncr;
                }
            }
        }

        
        //----------------------------------------------------------------------------------------------

        private void DisplayProfileString(int xOffset, int yStart, String message)
        {
            m_shapeDrawer.DrawText(message,new Vector3(xOffset,yStart,0),new Vector3(1,1,1));
        }

        //----------------------------------------------------------------------------------------------

        protected override void LoadContent()
        {
            //GraphicsDevice.Reset();
            base.LoadContent();
            // This needs to be here so that the GraphicsDevice has been created first.

            //VertexDeclaration vertexDeclaration = new VertexDeclaration(GraphicsDevice, VertexPositionColor.VertexElements);
            //BasicEffect basicEffect = new BasicEffect(GraphicsDevice, null);
            //m_debugDraw = new DefaultDebugDraw(vertexDeclaration,basicEffect);

            m_debugMode = DebugDrawModes.DBG_DrawWireframe | DebugDrawModes.DBG_DrawConstraints | DebugDrawModes.DBG_DrawConstraintLimits;
            m_shapeDrawer = new XNA_ShapeDrawer(this);
            m_debugDraw = m_shapeDrawer;
            BulletGlobals.gDebugDraw = m_debugDraw;
            m_shapeDrawer.LoadContent();
            m_shapeDrawer.EnableTexture(true);
            m_enableshadows = true;

        }
        //----------------------------------------------------------------------------------------------

        public void SetSize(int x, int y)
        {
            m_glutScreenWidth = x;
            m_glutScreenHeight = y;
        }

        
    }
}
