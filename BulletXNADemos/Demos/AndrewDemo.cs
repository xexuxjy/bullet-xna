using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using BulletXNA.BulletCollision;
using System.IO;
using BulletXNA.LinearMath;
using BulletXNA;
using BulletXNA.BulletDynamics;
using System.Diagnostics;
using Microsoft.Xna.Framework.Input;

namespace BulletXNADemos.Demos
{

    public enum CollisionGroup
    {
        GROUP_NON_COLLIDABLE,
        GROUP_COLLIDABLE_NON_PUSHABLE,
        GROUP_COLLIDABLE_PUSHABLE,
    }

    public class AndrewDemo : DemoApplication
    {
        bool worldLoaded = false;

        private const float scaleFactor = 1;
        private const float heightFieldOffset = 2000 / scaleFactor;
        private Dictionary<Vector3, int> heightFields = new Dictionary<Vector3, int>(); // Keeps track of the different height maps and their current sample rate
        private KinematicCharacterController playerController;
        private PairCachingGhostObject ghostObject;
        private KinematicCharacterController mobController;
        private List<BulletObjectData> ObjectsToLoad = new List<BulletObjectData>();

        float frozenTime = 0f;
        float requiredFrozenTime = 0f;

        Dictionary<SceneNode, BulletMobState> mobControllers;

        static void Main(string[] args)
        {
            using (AndrewDemo game = new AndrewDemo())
            {
                game.Run();
            }
        }

        public override void InitializeDemo()
        {
            base.InitializeDemo();

            SetCameraDistance(50);
            heightFields.Clear();
            //collisionShapes = new ObjectArray<CollisionShape>();
            m_collisionConfiguration = new DefaultCollisionConfiguration();

            ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
            m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);
            m_broadphase = new DbvtBroadphase(null,m_dispatcher);

            m_broadphase.GetOverlappingPairCache().SetInternalGhostPairCallback(new GhostPairCallback());	// Needed once to enable ghost objects inside Bullet

            //broadphase = new AxisSweep3(new Vector3(-1000, -1000, -1000), new Vector3(1000, 1000, 1000));  //new DbvtBroadphase();
            SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
            m_constraintSolver = sol;

            m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);
            m_dynamicsWorld.GetDispatchInfo().SetAllowedCcdPenetration(0.0001f);
            //world.DispatchInfo.AllowedCcdPenetration = 0.0001f;
            //world.Gravity = new Vector3(0, gravity, 0);
            mobControllers = new Dictionary<SceneNode, BulletMobState>();

            LoadPlayerController(new Entity(), new SceneNode(), null, Vector3.Zero);

            //IndexedVector3 walkDir = new IndexedVector3(0, 0, -1);
            //playerController.SetWalkDirection(ref walkDir);

            //float[,] heights = new float[128, 128];
            //LoadHeightField(heights, 10, 129, 129, Vector3.Zero, 1);

            float[,] heights = new float[32, 32];
            LoadHeightField(heights, 10, 33, 33, Vector3.Zero, 1);


            ClientResetScene();
        }



        public virtual void LoadHeightField(float[,] heights, float heightRange, uint vertsX, uint vertsZ, Vector3 loc, int metersPerSample)
        {
            //if (worldLoaded == false || metersPerSample > 2)
            //    return;
            loc = loc / scaleFactor;
            //heightRange = heightRange * 1000;
            //loc.X += heightFieldOffset;
            //loc.z += heightFieldOffset; // these axes are out by about 1m?
            if (heightFields.ContainsKey(loc))
            {
                int oldMetersPerSample = heightFields[loc];
                if (oldMetersPerSample == metersPerSample)
                {
                    return; // no need to update
                }
                else
                {
                    // we need to delete the old one to rebuild this one
                    foreach (CollisionObject a in m_dynamicsWorld.GetCollisionObjectArray())
                    {
                        if (a.GetCollisionShape() != null && a.GetCollisionShape().GetShapeType() == BroadphaseNativeTypes.TERRAIN_SHAPE_PROXYTYPE)
                        {
                            string terrainName = (string)a.GetUserPointer();
                            if (terrainName == "TestHeightField_" + loc)
                            {
                                a.Cleanup();
                                heightFields.Remove(loc);
                                System.Console.WriteLine("Removed heightmap at position '{0}' with metersPerSample: '{1}' and old: '{2}'",
                                    loc, metersPerSample, oldMetersPerSample);
                                break;
                            }
                        }
                    }
                }
            }
            byte[] terr = new byte[vertsX * vertsZ * 4];
            MemoryStream file = new MemoryStream(terr);
            BinaryWriter writer = new BinaryWriter(file);
            for (int i = 0; i < vertsX; i++)
            {
                for (int j = 0; j < vertsZ; j++)
                {
                    writer.Write((float)((heightRange / 2) + 4 * Math.Sin(j * 0.5f) * Math.Cos(i)));
                    //writer.Write(0f);
                }
            }
            writer.Flush();
            file.Position = 0;
            float heightScale = heightRange / 32767f / scaleFactor;
            int upAxis = 1;
            CollisionShape terrainShape = new HeightfieldTerrainShape((int)vertsX, (int)vertsZ, terr, heightScale, 0, heightRange, upAxis, PHY_ScalarType.PHY_FLOAT, true);
            IndexedMatrix worldTransform = IndexedMatrix.CreateTranslation(loc);
            DefaultMotionState objectMotionState = new DefaultMotionState(worldTransform, IndexedMatrix.Identity);

            //terrainShape = new StaticPlaneShape(Vector3.Up, 0f);
            //IndexedVector3 halfExtents = new IndexedVector3(50, 50, 50);

            //terrainShape = new BoxShape(ref halfExtents);

            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(0, objectMotionState, terrainShape);
            RigidBody terrain = new RigidBody(rbInfo);


            //IndexedMatrix groundTransform = IndexedMatrix.CreateTranslation(new IndexedVector3(0, -50, 0));
            IndexedMatrix groundTransform = IndexedMatrix.CreateTranslation(new IndexedVector3(0, -5, 0));
            LocalCreateRigidBody(0f, groundTransform, terrainShape);

            terrain.SetUserPointer("TestHeightField_" + loc.ToString());
            //terrain.SetCollisionFlags(CollisionFlags.CF_KINEMATIC_OBJECT);
            ////m_dynamicsWorld.AddCollisionObject(terrain, CollisionFilterGroups.DefaultFilter, CollisionFilterGroups.AllFilter);
            //m_dynamicsWorld.AddCollisionObject(terrain);


            System.Console.WriteLine("Added heightmap at position: '{0}' with metersPerSample: '{1}'", loc, metersPerSample);
            heightFields.Add(loc, metersPerSample);
        }

        /// <summary>
        /// Unloads the heightmap physics that is linked to the location passed in.
        /// </summary>
        /// <param name="loc"></param>
        public virtual void UnloadHeightField(Vector3 loc)
        {
            loc = loc / scaleFactor;
            loc.X += heightFieldOffset;
            //loc.z -= heightFieldOffset; // these axes are out by about 1m?
            if (heightFields.ContainsKey(loc))
            {
                // we need to delete the old one to rebuild this one
                foreach (CollisionObject a in m_dynamicsWorld.GetCollisionObjectArray())
                {
                    if (a.GetCollisionShape().GetName() == "TestHeightField_" + loc.ToString())
                    {
                        a.Cleanup();
                        heightFields.Remove(loc);
                        System.Console.WriteLine("Removed heightmap at position '{0}'",
                                loc);
                        break;
                    }
                }
            }
        }


        public virtual void LoadPlayerController(Entity playerEntity, SceneNode characterNode, object userData, Vector3 mobNodePositionUpdate)
        {
            //if (!initialized)
            //{
            //    characterToLoad = characterNode;
            //    characterEntityToLoad = playerEntity;
            //    JumpHandlerToLoad = jumpHandler;
            //    MobNodePositionUpdateToLoad = mobNodePositionUpdate;
            //    userDataToLoad = userData;
            //    return;
            //}
            if (playerController != null)
                return;
            float modelHeight = 2f;// (playerEntity.BoundingBox.Max.Y) / scaleFactor; // AJ: used to subtract minimum from maximum- playerEntity.BoundingBox.Minimum.y
            System.Console.WriteLine("Player capsule info: modelheight '{0}', boundingbox max '{1}', bounding box min '{2}' and playerPosition '{3}'",
                modelHeight, playerEntity.BoundingBox.Max.Y, playerEntity.BoundingBox.Min.Y, characterNode.Position);
            float radius = 1.75f;
            float height = 1.75f;



            ConvexShape capsule = new CapsuleShape(radius, height);
            //ConvexShape capsule = new SphereShape(radius);
            ghostObject = new PairCachingGhostObject();
            Vector3 position = new Vector3(0, 0, 0);//new Vector3(characterNode.Position.X / scaleFactor, (characterNode.Position.Y + 1500) / scaleFactor, characterNode.Position.Z / scaleFactor);
            //IndexedMatrix worldTransform = IndexedMatrix.CreateTranslation(characterNode.Position.X / scaleFactor,
            //    (characterNode.Position.Y + 1500) / scaleFactor, characterNode.Position.Z / scaleFactor);

            IndexedMatrix worldTransform = IndexedMatrix.CreateTranslation(position);

            ghostObject.SetWorldTransform(worldTransform);
            //broadphase.OverlappingPairCache.SetInternalGhostPairCallback(new GhostPairCallback());


            ghostObject.SetCollisionShape(capsule);
            ghostObject.SetCollisionFlags(CollisionFlags.CF_CHARACTER_OBJECT);
            float stepHeight = 0.35f;
            playerController = new KinematicCharacterController(ghostObject, capsule, stepHeight, 1);
            //characterToLoad = null;
            BulletMobState mobMovementState = new BulletMobState(playerController, mobNodePositionUpdate);
            //mobMovementState.JumpEvent += jumpHandler;
            mobControllers.Add(characterNode, mobMovementState);
            //m_dynamicsWorld.AddCollisionObject(ghostObject, CollisionFilterGroups.CharacterFilter, CollisionFilterGroups.StaticFilter | CollisionFilterGroups.DefaultFilter);
            m_dynamicsWorld.AddCollisionObject(ghostObject, CollisionFilterGroups.CharacterFilter, CollisionFilterGroups.StaticFilter | CollisionFilterGroups.DefaultFilter);
            //m_dynamicsWorld.AddCollisionObject(ghostObject, CollisionFilterGroups.DefaultFilter, CollisionFilterGroups.AllFilter);
            m_dynamicsWorld.AddAction(playerController);
            //collisionShapes.Add(capsule);
            //frozenTime = 0;
        }

        public virtual void PlayerMoveRequest(SceneNode sceneNode, Vector3 desiredDisplacement, ref Vector3 pos, bool followTerrain, float timeSinceLastFrame, ref bool dirty)
        {
            if (playerController == null)
                return;


            desiredDisplacement = desiredDisplacement / scaleFactor;
            IndexedVector3 indexedDisplacement = new IndexedVector3(desiredDisplacement);
            playerController.SetWalkDirection(ref indexedDisplacement);

            m_dynamicsWorld.StepSimulation(timeSinceLastFrame / 1000, 3);
            //log.Info("BULLET: completed step simulation");
            UpdateMobPositions();
            TryAddObjects();
            // Get the players current position
            IndexedMatrix worldTransform = playerController.GetGhostObject().GetWorldTransform();
            pos = new Vector3(worldTransform.ToMatrix().M41, worldTransform.ToMatrix().M42, worldTransform.ToMatrix().M43);
            //pos.y -= playerController.Height / 2;
            //pos.y -= playerController.Radius;
            if (pos.Y < 0)
            {
                pos.Y = 0;
                IndexedVector3 indexedPosition = new IndexedVector3(pos);
                playerController.Warp(ref indexedPosition);
            }
            pos = pos * scaleFactor;
        }

        protected virtual void UpdateMobPositions()
        {
            foreach (SceneNode node in mobControllers.Keys)
            {
                IndexedMatrix worldTransform = mobControllers[node].MobController.GetGhostObject().GetWorldTransform();
                Vector3 pos = new Vector3(worldTransform.ToMatrix().M41, worldTransform.ToMatrix().M42, worldTransform.ToMatrix().M43);
                //pos.y -= mobControllers[node].MobController.Height / 2;
                //pos.y -= mobControllers[node].MobController.Radius;
                mobControllers[node].UpdateMobNodePosition(pos * scaleFactor);
                //log.Info("new mob position is: " + pos);
                //node.Position = pos * scaleFactor;
            }
        }

        protected virtual void TryAddObjects()
        {
            foreach (BulletObjectData objectData in ObjectsToLoad)
            {
                if (true)//(checkPlayerPositions(objectData.position))
                {
                    m_dynamicsWorld.AddCollisionObject(objectData.collisionObject, CollisionFilterGroups.StaticFilter, CollisionFilterGroups.StaticFilter);
                    ObjectsToLoad.Remove(objectData);
                    return;
                }
            }
        }


        public override void ClientMoveAndDisplay(GameTime gameTime)
        {
            
            IndexedVector3 walkDirection = IndexedVector3.Zero;
            float walkVelocity = 1.1f * 4.0f; // 4 km/h -> 1.1 m/s
            float walkSpeed = walkVelocity * 0.001f;

            IndexedMatrix xform = ghostObject.GetWorldTransform();


            IndexedVector3 forwardDir = xform._basis[2];
            IndexedVector3 upDir = xform._basis[1];
            IndexedVector3 strafeDir = xform._basis[0];

            forwardDir.Normalize();
            upDir.Normalize();
            strafeDir.Normalize();

            KeyboardState keyboardState = Keyboard.GetState();


            if (keyboardState.IsKeyDown(Keys.I))
            {
                walkDirection += forwardDir;

            }
            if (keyboardState.IsKeyDown(Keys.K))
            {
                walkDirection -= forwardDir;

            }
            if (keyboardState.IsKeyDown(Keys.J))
            {
                IndexedMatrix orn = ghostObject.GetWorldTransform();
                orn._basis *= IndexedBasisMatrix.CreateFromAxisAngle(new IndexedVector3(0, 1, 0), 0.01f);
                ghostObject.SetWorldTransform(orn);

            }
            if (keyboardState.IsKeyDown(Keys.L))
            {
                IndexedMatrix orn = ghostObject.GetWorldTransform();
                orn._basis *= IndexedBasisMatrix.CreateFromAxisAngle(new IndexedVector3(0, 1, 0), -0.01f);
                ghostObject.SetWorldTransform(orn);

            }
            //if (key == Keys.O)
            //{
            //    playerController.Jump();
            //}

            IndexedVector3 result = walkDirection * walkSpeed;
            playerController.SetWalkDirection(ref result);

            base.ClientMoveAndDisplay(gameTime);

        }
    }

    public class BulletMobState
    {
        private bool falling = false;
        private float fallTime = 0.0f;
        private static Vector3 forwardMomentum = Vector3.One;
        //private Jump jump = new Jump();
        private KinematicCharacterController mobController;
        //private MobNodePositionUpdate mobNodePositionUpdate;
        private Vector3 mobNodePositionUpdate;
        //public event JumpEventHandler JumpEvent;

        public BulletMobState(KinematicCharacterController mobController, Vector3 mobNodePositionUpdate)
        {
            this.mobController = mobController;
            this.mobNodePositionUpdate = mobNodePositionUpdate;
        }

        public void UpdateMobNodePosition(Vector3 position)
        {
            //mobNodePositionUpdate(position);
        }

        public void StopJump()
        {
            //if (JumpEvent != null)
            //    JumpEvent(this, false);
        }

        public KinematicCharacterController MobController
        {
            get
            {
                return mobController;
            }
        }
    }

    public class Entity
    {
        public BoundingBox BoundingBox;
    }

    public class SceneNode
    {
        public Vector3 Position;
    }


    struct BulletObjectData
    {
        public string name;
        public CollisionObject collisionObject;
        public Vector3 position;
        public Quaternion orientation;
        public Vector3 scale;
        public object userData;
    }

    ///The btGhostPairCallback interfaces and forwards adding and removal of overlapping pairs from the btBroadphaseInterface to btGhostObject.



}
