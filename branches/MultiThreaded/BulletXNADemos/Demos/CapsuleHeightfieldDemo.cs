//using BulletXNA.BulletDynamics.ConstraintSolver;
//using Microsoft.Xna.Framework;
//using BulletXNA.BulletCollision.CollisionShapes;
//using BulletXNA;
//using BulletXNA.BulletDynamics.Dynamics;
//using BulletXNA.BulletCollision.CollisionDispatch;
//using BulletXNA.BulletDynamics.Dynamics;
//using BulletXNA.BulletCollision.BroadphaseCollision;
//using BulletXNA.BulletCollision.CollisionDispatch;
//using System;
//using BulletXNA.BulletCollision.CollisionShapes;
//using System.IO;

//namespace BulletXNADemos.Demos
//{
//    public class CapsuleHeightfieldDemo : DemoApplication
//    {
//        public override void InitializeDemo()
//        {
//            //SetCameraDistance(1f * 50f);

//            string filename = @"C:\users\man\bullett\xna-heightfield-output.txt";
//            FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
//            BulletGlobals.g_streamWriter = new StreamWriter(filestream);

//            ///collision configuration contains default setup for memory, collision setup
//            m_collisionConfiguration = new DefaultCollisionConfiguration();

//            ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
//            m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);

//            //m_broadphase = new DbvtBroadphase();
//            Vector3 worldMin = new Vector3(-1000, -1000, -1000);
//            Vector3 worldMax = new Vector3(1000, 1000, 1000);

//            //m_broadphase = new AxisSweep3Internal(ref worldMin, ref worldMax, 0xfffe, 0xffff, 16384, null, false);
//            m_broadphase = new SimpleBroadphase(100, null);
//            IOverlappingPairCache pairCache = null;
//            //pairCache = new SortedOverlappingPairCache();

//            //m_broadphase = new SimpleBroadphase(1000, pairCache);

//            ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
//            SequentialImpulseConstraintSolver sol = new SequentialImpulseConstraintSolver();
//            m_constraintSolver = sol;

//            m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

//            Vector3 gravity = new Vector3(0, -10, 0);
//            m_dynamicsWorld.SetGravity(ref gravity);
//            float[] heightMap = new float[RegionSize * RegionSize];
//            int counter = 0;
//            for (int i = 0; i < RegionSize; ++i)
//            {
//                for (int j = 0; j < RegionSize; ++j)
//                {
//                    heightMap[counter++] = 1f;
//                }
//            }
//            SetTerrain(heightMap);
//            AvatarGeomAndBodyCreation(0f,0f,0f);
//            //StaticPlaneShape planeShape = new StaticPlaneShape(new Vector3(0, 1, 0), 0f);
//            //LocalCreateRigidBody(0f, Matrix.Identity, planeShape);
//        }

//        public void DeleteTerrain()
//        {

//        }

//        public  void SetTerrain(float[] heightMap)
//        {
//            //uint Constants.RegionSize = 256;
//            if (m_terrainShape != null)
//            {
//                DeleteTerrain();
//            }

//            float hfmax;
//            float hfmin;


//            _origheightmap = heightMap;

//            hfmin = 0;
//            //hfmax = 256;
//            hfmax = 8;
//            byte[] bmap = new byte[heightMap.Length * 4];
//            int bmappos = 0;
//            for (int i = 0; i < heightMap.Length; i++)
//            {
//                byte[] val = BitConverter.GetBytes(heightMap[i]);
//                for (int j = 0; j < val.Length; j++)
//                {
//                    bmap[bmappos++] = val[j];
//                }
//            }
//            int upAxis = 1; //2
//            m_terrainShape = new HeightfieldTerrainShape(RegionSize, RegionSize, bmap,
//                                                                 1.0f, hfmin, hfmax, upAxis,
//                                                                 PHY_ScalarType.PHY_FLOAT, false);
//            float AabbCenterX = RegionSize / 2f;
//            //float AabbCenterY = RegionSize / 2f;
//            float AabbCenterZ = RegionSize / 2f;

//            float AabbCenterY = 0;
//            float temphfmin, temphfmax;

//            temphfmin = hfmin;
//            temphfmax = hfmax;

//            if (temphfmin < 0)
//            {
//                temphfmax = 0 - temphfmin;
//                temphfmin = 0 - temphfmin;
//            }
//            else if (temphfmin > 0)
//            {
//                temphfmax = temphfmax + (0 - temphfmin);

//            }
//            //AabbCenterZ = temphfmax / 2f;
//            AabbCenterY = temphfmax / 2f;

//            m_terrainPosition = new Vector3(AabbCenterX, AabbCenterY, AabbCenterZ);


//            m_terrainTransform = Matrix.CreateTranslation(m_terrainPosition);


//            m_terrainMotionState = new DefaultMotionState(m_terrainTransform, Matrix.Identity);
//            //m_terrainBody= new RigidBody(0, m_terrainMotionState, m_terrainShape, Vector3.Zero);
//            m_terrainBody = new RigidBody(0, null, m_terrainShape, Vector3.Zero);

//            m_terrainBody.SetUserPointer((IntPtr)0);
//            m_terrainBody.SetCollisionFlags(m_terrainBody.GetCollisionFlags() | CollisionFlags.CF_KINEMATIC_OBJECT);//STATIC_OBJECT);

//            m_dynamicsWorld.AddRigidBody(m_terrainBody);
//        }

//        private void AvatarGeomAndBodyCreation(float npositionX, float npositionY, float npositionZ)
//        {

//            if (CAPSULE_LENGTH <= 0)
//            {
//                //m_log.Warn("[PHYSICS]: The capsule size you specified in opensim.ini is invalid!  Setting it to the smallest possible size!");
//                CAPSULE_LENGTH = 0.01f;

//            }

//            if (CAPSULE_RADIUS <= 0)
//            {
//                //m_log.Warn("[PHYSICS]: The capsule size you specified in opensim.ini is invalid!  Setting it to the smallest possible size!");
//                CAPSULE_RADIUS = 0.01f;

//            }

//            //m_capsuleShape = new CapsuleShape(CAPSULE_RADIUS, CAPSULE_LENGTH);
//            m_capsuleShape = new CapsuleShapeZ(CAPSULE_RADIUS, CAPSULE_LENGTH);
//            //m_capsuleShape = new SphereShape(CAPSULE_RADIUS);
//            //m_capsuleShape = new BoxShape(new Vector3(CAPSULE_RADIUS,CAPSULE_RADIUS,CAPSULE_RADIUS));

//            // Copy OpenMetaverse.Vector3 to Microsoft.Xna.Framework.Vector3
//            m_bodyPosition.X = npositionX;
//            m_bodyPosition.Y = npositionY;
//            m_bodyPosition.Z = npositionZ;



//            m_capsuleOrientationAxis = new Vector3(1, 0, 1);
//            m_bodyOrientation = new Quaternion(m_capsuleOrientationAxis, MathUtil.SIMD_HALF_PI);
//            m_bodyOrientation.Normalize();

//            // This used to be a btTransform
//            //m_bodyTransform = new btTransform(m_bodyOrientation, m_bodyPosition);)

//            m_bodyTransform = Matrix.CreateFromQuaternion(m_bodyOrientation);
//            //m_bodyTransform = Matrix.Identity;
//            m_bodyTransform._origin = m_bodyPosition;



//            if (m_bodyMotionState == null)
//            {
//                m_bodyMotionState = new DefaultMotionState(m_bodyTransform, Matrix.Identity);
//            }
//            else
//            {
//                m_bodyMotionState.SetWorldTransform(m_bodyTransform);
//            }

//            //m_mass = Mass;

//            m_capsuleBody = new RigidBody(m_mass, m_bodyMotionState, m_capsuleShape, Vector3.Zero);

//            // this is used for self identification.
//            m_capsuleBody.SetUserPointer(this);  // "this" is of type BulletXnaPlugin.BulletXnaCharacter

//            // Used to detect whether or not the proxy is standing on something
//            //ClosestCastResult = new ClosestNotMeRaycastResultCallback(Body);
//            //ClosestNotMeConvexResultCallback callback = new ClosestNotMeConvexResultCallback(m_capsuleBody);

//            m_dynamicsWorld.AddRigidBody(m_capsuleBody);

//            // Since this is a users character proxy, we are disabling the automatic deactivation.
//            m_capsuleBody.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

//            if (m_aMotor != null)
//            {

//                m_dynamicsWorld.RemoveConstraint(m_aMotor);
//                m_aMotor = null;
//            }

//            Matrix frameA = Matrix.Identity;
//            Matrix frameB = Matrix.Identity;

//            //m_aMotor = new Generic6DofConstraint(m_capsuleBody, m_terrainBody, ref frameA, ref frameB, false);
//            //m_aMotor.SetAngularLowerLimit(Vector3.Zero);
//            //m_aMotor.SetAngularUpperLimit(Vector3.Zero);

//        }

//        static void Main(string[] args)
//        {
//            using (CapsuleHeightfieldDemo game = new CapsuleHeightfieldDemo())
//            {
//                game.Run();
//            }

//        }

//        //static void Main(string[] args)
//        //{
//        //    using (ConstraintDemo game = new ConstraintDemo())
//        //    {
//        //        game.Run();
//        //    }
//        //}

//        float m_mass = 1f;
//        IMotionState m_terrainMotionState;
//        RigidBody m_terrainBody;
//        Generic6DofConstraint m_aMotor;
//        float[] _origheightmap;
//        HeightfieldTerrainShape m_terrainShape;
//        Vector3 m_terrainPosition;
//        Matrix m_terrainTransform;

//        IMotionState m_bodyMotionState;
//        Vector3 m_bodyPosition;
//        Quaternion m_bodyOrientation;
//        Matrix m_bodyTransform;
//        CollisionShape m_capsuleShape;
//        Vector3 m_capsuleOrientationAxis;
//        RigidBody m_capsuleBody;

//        float CAPSULE_LENGTH = 1f;
//        float CAPSULE_RADIUS = 0.5f;
//        const int RegionSize = 16;
//    }

//}
