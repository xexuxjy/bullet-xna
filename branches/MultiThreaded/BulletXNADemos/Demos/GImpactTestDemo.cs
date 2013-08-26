#define TEST_GIMPACT_TORUS

using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using BulletXNA.LinearMath;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace BulletXNADemos.Demos
{
    public class GImpactTestDemo : DemoApplication
    {

        TriangleIndexVertexArray m_indexVertexArrays;
        TriangleIndexVertexArray m_indexVertexArrays2;

        CollisionShape m_trimeshShape;
        CollisionShape m_trimeshShape2;

        IndexedVector3 m_kinTorusTran;
        Quaternion m_kinTorusRot;
        RigidBody m_kinematicTorus;


        public override void InitializeDemo()
        {
            m_cameraDistance = 10.0f;
            //string filename = @"e:\users\man\bullet\gimpact-demo-xna.txt";
            //FileStream filestream = File.Open(filename, FileMode.Create, FileAccess.Write, FileShare.Read);
            //BulletGlobals.g_streamWriter = new StreamWriter(filestream);

            /// Init Bullet
            m_collisionConfiguration = new DefaultCollisionConfiguration();

            m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);
            //btOverlappingPairCache* broadphase = new btSimpleBroadphase();
            //m_broadphase = new btSimpleBroadphase();

            int maxProxies = 1024;
            IndexedVector3 worldAabbMin = new IndexedVector3(-10000, -10000, -10000);
            IndexedVector3 worldAabbMax = new IndexedVector3(10000, 10000, 10000);
            m_broadphase = new AxisSweep3Internal(ref worldAabbMin, ref worldAabbMax, 0xfffe, 0xffff, 16384, null, false);  
            //m_broadphase = new SimpleBroadphase(16384,null);
            m_constraintSolver = new SequentialImpulseConstraintSolver();

            m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

            //create trimesh model and shape
            InitGImpactCollision();



            /// Create Scene
            float mass = 0.0f;
            IndexedMatrix startTransform = IndexedMatrix.Identity;


            CollisionShape staticboxShape1 = new BoxShape(new IndexedVector3(200, 1, 200));//floor
            staticboxShape1.SetUserPointer("Floor");
            CollisionShape staticboxShape2 = new BoxShape(new IndexedVector3(1, 50, 200));//left wall
            staticboxShape1.SetUserPointer("LeftWall");
            CollisionShape staticboxShape3 = new BoxShape(new IndexedVector3(1, 50, 200));//right wall
            staticboxShape1.SetUserPointer("RightWall");
            CollisionShape staticboxShape4 = new BoxShape(new IndexedVector3(200, 50, 1));//front wall
            staticboxShape1.SetUserPointer("FrontWall");
            CollisionShape staticboxShape5 = new BoxShape(new IndexedVector3(200, 50, 1));//back wall
            staticboxShape1.SetUserPointer("BackWall");

            CompoundShape staticScenario = new CompoundShape();//static scenario

            startTransform._origin = new IndexedVector3(0, 0, 0);
            staticScenario.AddChildShape(ref startTransform, staticboxShape1);
            startTransform._origin = new IndexedVector3(-200, 25, 0);
            staticScenario.AddChildShape(ref startTransform, staticboxShape2);
            startTransform._origin = new IndexedVector3(200, 25, 0);
            staticScenario.AddChildShape(ref startTransform, staticboxShape3);
            startTransform._origin = new IndexedVector3(0, 25, 200);
            staticScenario.AddChildShape(ref startTransform, staticboxShape4);
            startTransform._origin = new IndexedVector3(0, 25, -200);
            staticScenario.AddChildShape(ref startTransform, staticboxShape5);

            startTransform._origin = new IndexedVector3(0, 0, 0);

            //RigidBody staticBody = LocalCreateRigidBody(mass, startTransform, staticScenario);
            RigidBody staticBody = LocalCreateRigidBody(mass, startTransform, staticboxShape1);


	        staticBody.SetCollisionFlags(staticBody.GetCollisionFlags()|CollisionFlags.CF_STATIC_OBJECT);

	        //enable custom material callback
	        staticBody.SetCollisionFlags(staticBody.GetCollisionFlags()|CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK);

	        //static plane
	        IndexedVector3 normal = new IndexedVector3(0.4f,1.5f,-0.4f);
	        normal.Normalize();
	        CollisionShape staticplaneShape6 = new StaticPlaneShape(ref normal,0.0f);// A plane

	        startTransform._origin = IndexedVector3.Zero;

            RigidBody staticBody2 = LocalCreateRigidBody(mass, startTransform, staticplaneShape6);

	        staticBody2.SetCollisionFlags(staticBody2.GetCollisionFlags()|CollisionFlags.CF_STATIC_OBJECT);

            startTransform = IndexedMatrix.Identity;

            /// Create Dynamic Boxes
            {
                int numBoxes = 1;
                for (int i = 0; i < numBoxes; i++)
                {
                    CollisionShape boxShape = new BoxShape(new IndexedVector3(1, 1, 1));
                    //CollisionShape mesh = new BvhTriangleMeshShape(m_indexVertexArrays2,true,true);
                    startTransform._origin = new IndexedVector3(2 * i - (numBoxes-1), 2, -3);
                    //startTransform._origin = new IndexedVector3(2 * i - 5, 10, -3);
                    //LocalCreateRigidBody(1, startTransform, m_trimeshShape2);
                    LocalCreateRigidBody(1, startTransform, boxShape);
                }
            }
        }


        public void InitGImpactCollision()
        {

            /// Create Torus Shape
            {
                m_indexVertexArrays = new TriangleIndexVertexArray(DemoMeshes.TORUS_NUM_TRIANGLES, DemoMeshes.gTorusIndices, 3, DemoMeshes.TORUS_NUM_VERTICES, DemoMeshes.gTorusVertices, 3);

#if BULLET_GIMPACT_CONVEX_DECOMPOSITION
			btGImpactConvexDecompositionShape * trimesh  = new
			btGImpactConvexDecompositionShape(
			m_indexVertexArrays, IndexedVector3(1.f,1.f,1.f),btScalar(0.01));
			trimesh->setMargin(0.07);
			trimesh->updateBound();


#else
                //GImpactMeshShape trimesh = new GImpactMeshShape(m_indexVertexArrays);
                //IndexedVector3 scaling = IndexedVector3.One;
                //trimesh.SetLocalScaling(ref scaling);
                //trimesh.SetMargin(0.07f); ///?????
                //trimesh.UpdateBound();
#endif

                //m_trimeshShape = trimesh;

            }

            /// Create Bunny Shape
            {
                m_indexVertexArrays2 = new TriangleIndexVertexArray(DemoMeshes.BUNNY_NUM_TRIANGLES, DemoMeshes.gBunnyIndices, 3, DemoMeshes.BUNNY_NUM_VERTICES, DemoMeshes.gBunnyVertices, 3);
#if BULLET_GIMPACT_CONVEX_DECOMPOSITION
			btGImpactConvexDecompositionShape * trimesh2  = new
			btGImpactConvexDecompositionShape(
			m_indexVertexArrays2, IndexedVector3(4.f,4.f,4.f),btScalar(0.01));
			trimesh2->setMargin(0.07);
			trimesh2->updateBound();
#else
                GImpactMeshShape trimesh2 = new GImpactMeshShape(m_indexVertexArrays2);
                IndexedVector3 scaling = new IndexedVector3(4.0f, 4.0f, 4.0f);
                trimesh2.SetLocalScaling(ref scaling);
                //trimesh2.SetMargin(0.07f); ///?????
                trimesh2.UpdateBound();
#endif
                m_trimeshShape2 = trimesh2;

            }


            ///register GIMPACT algorithm
            CollisionDispatcher dispatcher = m_dynamicsWorld.GetDispatcher() as CollisionDispatcher;

            GImpactCollisionAlgorithm.RegisterAlgorithm(dispatcher);
        }


        public void ShootTrimesh(IndexedVector3 startPosition, IndexedVector3 destination)
    {

	    if (m_dynamicsWorld != null)
	    {
		    float mass = 4.0f;
            IndexedMatrix startTransform = IndexedMatrix.Identity;
            startTransform._origin = new IndexedVector3(startPosition);

		    RigidBody body = LocalCreateRigidBody(mass, startTransform,m_trimeshShape2);

		    IndexedVector3 linVel = new IndexedVector3(destination - startPosition);
		    linVel.Normalize();
		    linVel*=m_ShootBoxInitialSpeed*0.25f;

		    body.SetLinearVelocity(ref linVel);
		    body.SetAngularVelocity(IndexedVector3.Zero);
	    }
    }

    public override void KeyboardCallback(Keys key, int x, int y, GameTime gameTime, bool released, ref Microsoft.Xna.Framework.Input.KeyboardState newState, ref Microsoft.Xna.Framework.Input.KeyboardState oldState)
    {
        if (key == Keys.OemPeriod)
        {
            ShootTrimesh(GetCameraPosition(), GetCameraTargetPosition());
        }
        else
        {
            base.KeyboardCallback(key, x, y, gameTime, released, ref newState, ref oldState);
        }
    }


        static void Main(string[] args)
        {
            using (GImpactTestDemo game = new GImpactTestDemo())
            {
                game.Run();
            }
        }

    }
}
