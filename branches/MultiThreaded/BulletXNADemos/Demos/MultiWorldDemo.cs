using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BulletXNA.BulletDynamics;
using BulletXNA.LinearMath;
using BulletXNA.BulletCollision;
using Microsoft.Xna.Framework;
using BulletXNA;
using System.Diagnostics;
using System.Threading.Tasks;

namespace BulletXNADemos.Demos
{
    public class MultiWorldDemo : DemoApplication
    {


        public override void InitializeDemo()
        {
            SetupWorlds();
            SetupWorldObjects();
        }

        //----------------------------------------------------------------------------------------------
        
        public void SetupWorlds()
        {
            DiscreteDynamicsWorld world1 = null;
            DiscreteDynamicsWorld world2 = null;

            InitialiseWorld(ref world1);
            InitialiseWorld(ref world2);

            m_worlds.Add(world1);
            m_worlds.Add(world2);

        }

        //----------------------------------------------------------------------------------------------

        public void SetupWorldObjects()
        {
            IndexedVector3 halfExtents = new IndexedVector3(10, 5, 10);
            CollisionShape groundShape = new BoxShape(ref halfExtents);

            IndexedVector3 world1Center = new IndexedVector3(-20, -5, 0);
            IndexedVector3 world2Center = new IndexedVector3(20, -5, 0);

            IndexedMatrix groundTransform1 = IndexedMatrix.CreateTranslation(world1Center);
            IndexedMatrix groundTransform2 = IndexedMatrix.CreateTranslation(world2Center);

            IndexedVector3 halfExtents2 = new IndexedVector3(0.5f);
            CollisionShape smallBoxShape = new BoxShape(ref halfExtents2);

            //IndexedMatrix groundTransform = IndexedMatrix.CreateTranslation(new IndexedVector3(0,-10,0));
            float mass = 0f;

            LocalCreateRigidBodyMultiWorld(mass, ref groundTransform1, groundShape ,m_worlds[0]);
            LocalCreateRigidBodyMultiWorld(mass, ref groundTransform2, groundShape, m_worlds[1]);

            mass = 1f;
            for (int i = 0; i < 5; ++i)
            {
                IndexedVector3 offset = new IndexedVector3(0, halfExtents.Y + halfExtents2.Y + (2 * i), 0);

                IndexedMatrix boxTransform1 = IndexedMatrix.CreateTranslation(world1Center+offset);
                IndexedMatrix boxTransform2 = IndexedMatrix.CreateTranslation(world2Center + offset);

                LocalCreateRigidBodyMultiWorld(mass, ref boxTransform1, smallBoxShape, m_worlds[0]);
                LocalCreateRigidBodyMultiWorld(mass, ref boxTransform2, smallBoxShape, m_worlds[1]);
            }

        }

        //----------------------------------------------------------------------------------------------

        public override void ClientMoveAndDisplay(GameTime gameTime)
        {
            //simple dynamics world doesn't handle fixed-time-stepping
            //float ms = (float)gameTime.ElapsedGameTime.TotalSeconds;
            //float ms = 16666f / 1000000.0f;
            float ms = 1f / 60f;
            //ms *= 0.1f;
            ///step the simulation
            ///
            List<Task> taskList = new List<Task>();
            foreach (DiscreteDynamicsWorld world in m_worlds)
            {
                var worldCopy = world;
                Task t1 = Task.Factory.StartNew(() => worldCopy.StepSimulation(ms,1));
            }
            Task.WaitAll(taskList.ToArray());

        }

        //----------------------------------------------------------------------------------------------


        public void InitialiseWorld(ref DiscreteDynamicsWorld world)
        {

            //m_broadphase = new DbvtBroadphase();
            IOverlappingPairCache pairCache = null;
            //pairCache = new SortedOverlappingPairCache();

            //m_broadphase = new SimpleBroadphase(1000, pairCache);

            ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
            SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
            DefaultCollisionConfiguration collisionConfig = new DefaultCollisionConfiguration();

            ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
            CollisionDispatcher dispatcher = new CollisionDispatcher(collisionConfig);

            IndexedVector3 worldMin = new IndexedVector3(-1000, -1000, -1000);
            IndexedVector3 worldMax = -worldMin;
            IBroadphaseInterface bp = new AxisSweep3Internal(ref worldMin, ref worldMax, 0xfffe, 0xffff, 16384, null, false, dispatcher);


            world = new DiscreteDynamicsWorld(dispatcher, bp, sol, collisionConfig);

            IndexedVector3 gravity = new IndexedVector3(0, -10, 0);
            world.SetGravity(ref gravity);
            world.SetDebugDrawer(BulletGlobals.gDebugDraw);

        }

        //----------------------------------------------------------------------------------------------

        public override void ClientResetScene()
        {
            //#ifdef SHOW_NUM_DEEP_PENETRATIONS
            gNumDeepPenetrationChecks = 0;
            gNumGjkChecks = 0;
            //#endif //SHOW_NUM_DEEP_PENETRATIONS

            gNumClampedCcdMotions = 0;
            int numObjects = 0;

            foreach (DiscreteDynamicsWorld world in m_worlds)
            {
                // Prefer a better place for this...
                world.SetDebugDrawer(m_debugDraw);
                numObjects = world.GetNumCollisionObjects();

                IList<CollisionObject> copyArray = world.GetCollisionObjectArray();

                for (int i = 0; i < numObjects; i++)
                {
                    CollisionObject colObj = copyArray[i];
                    RigidBody body = RigidBody.Upcast(colObj);
                    if (body != null)
                    {
                        if (body.GetMotionState() != null)
                        {
                            DefaultMotionState myMotionState = (DefaultMotionState)body.GetMotionState();
                            myMotionState.m_graphicsWorldTrans = myMotionState.m_startWorldTrans;
                            body.SetCenterOfMassTransform(ref myMotionState.m_graphicsWorldTrans);
                            colObj.SetInterpolationWorldTransform(ref myMotionState.m_startWorldTrans);
                            if (colObj.GetActivationState() != ActivationState.DISABLE_DEACTIVATION)
                            {
                                colObj.ForceActivationState(ActivationState.ACTIVE_TAG);
                                colObj.Activate();
                                colObj.SetDeactivationTime(0);
                            }
                            //colObj.setActivationState(WANTS_DEACTIVATION);
                        }
                        //removed cached contact points (this is not necessary if all objects have been removed from the dynamics world)
                        world.GetBroadphase().GetOverlappingPairCache().CleanProxyFromPairs(colObj.GetBroadphaseHandle(), world.GetDispatcher());

                        if (!body.IsStaticObject())
                        {
                            IndexedVector3 zero = IndexedVector3.Zero;
                            body.SetLinearVelocity(ref zero);
                            body.SetAngularVelocity(ref zero);
                        }
                    }
                }
                ///reset some internal cached data in the broadphase
                world.GetBroadphase().ResetPool(world.GetDispatcher());
                world.GetConstraintSolver().Reset();
            }
    
        }
        //----------------------------------------------------------------------------------------------

        protected override void RenderSceneAll(GameTime gameTime)
        {

            foreach (DiscreteDynamicsWorld world in m_worlds)
            {
                world.DebugDrawWorld();
                RenderScenePassMultiWorld(0, gameTime,world);
                IndexedVector3 location = new IndexedVector3(10, 10, 0);
                IndexedVector3 colour = new IndexedVector3(1, 1, 1);
                //m_shapeDrawer.DrawText(String.Format("Memory [{0}]", System.GC.GetTotalMemory(false)), ref location, ref colour);
                int xOffset = 10;
                int yStart = 20;
                int yIncr = 15;

                //ShowProfileInfo(xOffset, yStart, yIncr);

            }
            m_shapeDrawer.RenderOthers(gameTime, m_lookAt, m_perspective);
            m_shapeDrawer.RenderStandard(gameTime, ref m_lookAt, ref m_perspective);
            m_shapeDrawer.RenderDebugLines(gameTime, ref m_lookAt, ref m_perspective);
        }

        //----------------------------------------------------------------------------------------------

        protected virtual void RenderScenePassMultiWorld(int pass, GameTime gameTime,DiscreteDynamicsWorld world)
        {
            IndexedMatrix m = IndexedMatrix.Identity;
            IndexedBasisMatrix rot = IndexedBasisMatrix.Identity;
            int numObjects = world.GetNumCollisionObjects();
            IndexedVector3 wireColor = new IndexedVector3(1, 0, 0);

            for (int i = 0; i < numObjects; i++)
            {
                CollisionObject colObj = world.GetCollisionObjectArray()[i];
                RigidBody body = RigidBody.Upcast(colObj);
                if (body != null && body.GetMotionState() != null)
                {
                    DefaultMotionState myMotionState = (DefaultMotionState)body.GetMotionState();
                    //myMotionState.m_graphicsWorldTrans.getOpenGLMatrix(m);
                    m = myMotionState.m_graphicsWorldTrans;
                    rot = myMotionState.m_graphicsWorldTrans._basis;
                }
                else
                {
                    //colObj.getWorldTransform().getOpenGLMatrix(m);
                    m = colObj.GetWorldTransform();
                    rot = colObj.GetWorldTransform()._basis;
                }
                wireColor = new IndexedVector3(1.0f, 1.0f, 0.5f); //wants deactivation
                if ((i & 1) != 0) wireColor = new IndexedVector3(0f, 0f, 1f);
                ///color differently for active, sleeping, wantsdeactivation states
                if (colObj.GetActivationState() == ActivationState.ACTIVE_TAG) //active
                {
                    if ((i & 1) != 0)
                    {
                        wireColor += new IndexedVector3(1f, 0f, 0f);
                    }
                    else
                    {
                        wireColor += new IndexedVector3(.5f, 0f, 0f);
                    }
                }
                if (colObj.GetActivationState() == ActivationState.ISLAND_SLEEPING) //ISLAND_SLEEPING
                {
                    if ((i & 1) != 0)
                    {
                        wireColor += new IndexedVector3(0f, 1f, 0f);
                    }
                    else
                    {
                        wireColor += new IndexedVector3(0f, 05f, 0f);
                    }
                }

                IndexedVector3 min, max;
                world.GetBroadphase().GetBroadphaseAabb(out min, out max);

                min -= MathUtil.MAX_VECTOR;
                max += MathUtil.MAX_VECTOR;
                //		printf("aabbMin=(%f,%f,%f)\n",aabbMin.getX(),aabbMin.getY(),aabbMin.getZ());
                //		printf("aabbMax=(%f,%f,%f)\n",aabbMax.getX(),aabbMax.getY(),aabbMax.getZ());
                //		m_dynamicsWorld.getDebugDrawer().drawAabb(aabbMin,aabbMax,btVector3(1,1,1));

                switch (pass)
                {
                    case 0:
                        {
                            m_shapeDrawer.DrawXNA(ref m, colObj.GetCollisionShape(), ref wireColor, m_debugDraw.GetDebugMode(), ref min, ref max, ref m_lookAt, ref m_perspective);
                            break;
                        }
                    case 1:
                        {
                            IndexedVector3 shadow = rot * m_lightDirection;
                            m_shapeDrawer.DrawShadow(ref m, ref shadow, colObj.GetCollisionShape(), ref min, ref max);
                            break;
                        }
                    case 2:
                        {
                            IndexedVector3 adjustedWireColor = wireColor * 0.3f;
                            m_shapeDrawer.DrawXNA(ref m, colObj.GetCollisionShape(), ref adjustedWireColor, 0, ref min, ref max, ref m_lookAt, ref m_perspective);
                            break;
                        }
                }
            }
        }

        //----------------------------------------------------------------------------------------------

        public RigidBody LocalCreateRigidBodyMultiWorld(float mass, ref IndexedMatrix startTransform, CollisionShape shape, DiscreteDynamicsWorld world)
        {

            Debug.Assert((shape == null || shape.GetShapeType() != BroadphaseNativeTypes.INVALID_SHAPE_PROXYTYPE));

            //rigidbody is dynamic if and only if mass is non zero, otherwise static
            bool isDynamic = !MathUtil.CompareFloat(mass, 0f);

            IndexedVector3 localInertia = IndexedVector3.Zero;
            if (isDynamic)
            {
                shape.CalculateLocalInertia(mass, out localInertia);
            }
            //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

            //#define USE_MOTIONSTATE 1
            //#ifdef USE_MOTIONSTATE
            DefaultMotionState myMotionState = new DefaultMotionState(startTransform, IndexedMatrix.Identity);

            RigidBodyConstructionInfo cInfo = new RigidBodyConstructionInfo(mass, myMotionState, shape, localInertia);

            RigidBody body = new RigidBody(cInfo);

            if (BulletGlobals.g_streamWriter != null && true)
            {
                BulletGlobals.g_streamWriter.WriteLine("localCreateRigidBody [{0}] startTransform", body.m_debugBodyId);
                MathUtil.PrintMatrix(BulletGlobals.g_streamWriter, startTransform);
                BulletGlobals.g_streamWriter.WriteLine("");
            }

            world.AddRigidBody(body);

            return body;
        }

        //----------------------------------------------------------------------------------------------

        public override void ShutdownDemo()
        {
            foreach (DiscreteDynamicsWorld world in m_worlds)
            {
                //delete dynamics world
                world.Cleanup();
            }

        }
        //----------------------------------------------------------------------------------------------

        public List<DiscreteDynamicsWorld> m_worlds = new List<DiscreteDynamicsWorld>();
        //Future<bool> parallelDraw = ParallelTasks.Parallel.Start<bool>(DoThreadedDraw, new WorkOptions() { DetachFromParent = true, MaximumThreads = 1 });
        //parallelDraw.GetResult();



        static void Main(string[] args)
        {
            using (MultiWorldDemo game = new MultiWorldDemo())
            {
                game.Run();
            }
        }

    }
}
