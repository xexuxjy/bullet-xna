using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;
using BulletXNA.BulletCollision;
using Microsoft.Xna.Framework.Graphics;
using BulletXNA.BulletDynamics;

namespace BulletXNADemos.Demos
{
    public class StaticLevelDemo : DemoApplication
    {

        public override void InitializeDemo()
        {
            m_maxIterations = 10;
            SetCameraDistance(50f);

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

            m_dynamicsWorld.SetForceUpdateAllAabbs(false);

            BuildLevelMap();
        }

        public void BuildLevelMap()
        {
            Texture2D map = Content.Load<Texture2D>("kcmap");

            Color[] plats = new Color[map.Width * map.Height];
            map.GetData<Color>(plats);

            Vector3 bottomLeft = new Vector3(30, 30, 0);

            BuildCollisionData(plats, map.Width, map.Height, bottomLeft);
        }

        public void BuildCollisionData(Color[] map, int width, int height, Vector3 bottomLeft)
        {
            Vector2 dim = Vector2.One;
            Vector3 scale = new Vector3(dim, 1);

            BoxShape collisionBoxShape = new BoxShape(new IndexedVector3(0.5f));

            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    int currentIndex = x + y * width;
                    if (map[currentIndex] == Color.White)
                    {
                        float yOffset = -dim.Y;//0f;// -(dim.Y / 2f);
                        Vector3 position = new Vector3(x - bottomLeft.X, (bottomLeft.Y - y) + yOffset, bottomLeft.Z);
                        m_waterLocations.Add(position);
                    }

                    if (map[currentIndex] == Color.White)
                    {
                        // check the 4 ordinals , if we're surrounded by other solid blocks
                        // then we don't need a block here.
                        bool upSet = false;
                        bool downSet = false;
                        bool leftSet = false;
                        bool rightSet = false;

                        if (x >= 1 && x < width - 1)
                        {
                            if (map[currentIndex - 1] == Color.White)
                            {
                                leftSet = true;
                            }
                            if (map[currentIndex + 1] == Color.White)
                            {
                                rightSet = true;

                            }
                        }

                        if (y >= 1 && y < height - 1)
                        {
                            if (map[currentIndex - height] == Color.White)
                            {
                                upSet = true;
                            }
                            if (map[currentIndex + height] == Color.White)
                            {
                                downSet = true;
                            }

                        }

                        // if we're not surrounded by blocks then add in.
                        if (!(upSet && downSet && leftSet && rightSet))
                        {
                            Object rigifdBody;
                            float yOffset = -dim.Y;//0f;// -(dim.Y / 2f);
                            Vector3 position = new Vector3(x - bottomLeft.X, (bottomLeft.Y - y) + yOffset, bottomLeft.Z);

                            RigidBodyConstructionInfo constructionInfo = new BulletXNA.BulletDynamics.RigidBodyConstructionInfo(0f, null, (BulletXNA.BulletCollision.CollisionShape)collisionBoxShape);
                            RigidBody rigidBody = new BulletXNA.BulletDynamics.RigidBody(constructionInfo);
                            Matrix bsm = Matrix.CreateTranslation(position);
                            rigidBody.SetWorldTransform(bsm);
                            // FIXME MAN - setup some collision flags on these bodies...
                            BulletXNA.BulletCollision.CollisionFilterGroups flags = (BulletXNA.BulletCollision.CollisionFilterGroups)(1 << 8);
                            BulletXNA.BulletCollision.CollisionFilterGroups mask = (BulletXNA.BulletCollision.CollisionFilterGroups)(1 << 9);
                            //rigidBody.CollisionFlags |= (BulletSharp.CollisionFlags)CollisionObjectType.Ground;
                            m_dynamicsWorld.AddRigidBody(rigidBody, flags, mask);

                        }
                    }

                    // Build water ghost objects.
                    foreach (Vector3 pos in m_waterLocations)
                    {
                        GhostObject ghostObject = new GhostObject();
                        
                        ghostObject.SetCollisionShape((BulletXNA.BulletCollision.CollisionShape)collisionBoxShape);

                        CollisionFilterGroups flags = (CollisionFilterGroups)(1 << 10);
                        CollisionFilterGroups mask = (CollisionFilterGroups)(1<<9);

                        ghostObject.SetCollisionFlags(CollisionFlags.CF_NO_CONTACT_RESPONSE | CollisionFlags.CF_STATIC_OBJECT);		// We can choose to make it "solid" if we want...
                        ghostObject.SetWorldTransform(BulletXNA.LinearMath.IndexedMatrix.CreateTranslation(pos));
                        m_dynamicsWorld.AddCollisionObject(ghostObject, flags, mask);
                        break;
                    }


                }
            }



        }


        static void Main(string[] args)
        {
            using (StaticLevelDemo game = new StaticLevelDemo())
            {
                game.Run();
            }
        }

        public ObjectArray<Vector3> m_waterLocations = new ObjectArray<Vector3>();
    }
}
