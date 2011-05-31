#define TEST_GIMPACT_TORUS

using BulletXNA.BulletCollision.CollisionShapes;
using Microsoft.Xna.Framework;
using BulletXNA.BulletCollision.CollisionDispatch;
using BulletXNA.BulletCollision.GImpact;
using BulletXNA.BulletDynamics.Dynamics;
using BulletXNA.BulletDynamics.ConstraintSolver;
using BulletXNA.BulletCollision.BroadphaseCollision;

namespace BulletXNADemos.Demos
{
    public class GImpactTestDemo : DemoApplication
    {

        TriangleIndexVertexArray m_indexVertexArrays;
        TriangleIndexVertexArray m_indexVertexArrays2;

        CollisionShape m_trimeshShape;
        CollisionShape m_trimeshShape2;

        Vector3 m_kinTorusTran;
        Quaternion m_kinTorusRot;
        RigidBody m_kinematicTorus;


        public override void InitializeDemo()
        {
            /// Init Bullet
            m_collisionConfiguration = new DefaultCollisionConfiguration();

            m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);
            //btOverlappingPairCache* broadphase = new btSimpleBroadphase();
            //m_broadphase = new btSimpleBroadphase();

            int maxProxies = 1024;
            Vector3 worldAabbMin = new Vector3(-10000, -10000, -10000);
            Vector3 worldAabbMax = new Vector3(10000, 10000, 10000);
            m_broadphase = new AxisSweep3Internal(ref worldAabbMin, ref worldAabbMax, 0xfffe, 0xffff, 16384, null, false);
            m_constraintSolver = new SequentialImpulseConstraintSolver();

            m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);

            //create trimesh model and shape
            InitGImpactCollision();



            /// Create Scene
            float mass = 0.0f;
            Matrix startTransform = Matrix.Identity;

            CollisionShape staticboxShape1 = new BoxShape(new Vector3(200, 1, 200));//floor
            m_collisionShapes.Add(staticboxShape1);
            startTransform.Translation = new Vector3(0, -10, 0);
            LocalCreateRigidBody(mass, startTransform, staticboxShape1);

            CollisionShape staticboxShape2 = new BoxShape(new Vector3(1, 50, 200));//left wall
            m_collisionShapes.Add(staticboxShape2);
            startTransform.Translation = new Vector3(-200, 15, 0);
            LocalCreateRigidBody(mass, startTransform, staticboxShape2);

            CollisionShape staticboxShape3 = new BoxShape(new Vector3(1, 50, 200));//right wall
            m_collisionShapes.Add(staticboxShape3);
            startTransform.Translation = new Vector3(200, 15, 0);
            LocalCreateRigidBody(mass, startTransform, staticboxShape3);

            CollisionShape staticboxShape4 = new BoxShape(new Vector3(200, 50, 1));//front wall
            m_collisionShapes.Add(staticboxShape4);
            startTransform.Translation = new Vector3(0, 15, 200);
            LocalCreateRigidBody(mass, startTransform, staticboxShape4);

            CollisionShape staticboxShape5 = new BoxShape(new Vector3(200, 50, 1));//back wall
            m_collisionShapes.Add(staticboxShape5);
            startTransform.Translation = new Vector3(0, 15, -200);
            LocalCreateRigidBody(mass, startTransform, staticboxShape5);


            //static plane

            Vector3 normal = new Vector3(-0.5f, 0.5f, 0.0f);
            normal.Normalize();
            CollisionShape staticplaneShape6 = new StaticPlaneShape(normal, 0.0f);// A plane
            m_collisionShapes.Add(staticplaneShape6);
            startTransform.Translation = new Vector3(0, -9, 0);

            RigidBody staticBody2 = LocalCreateRigidBody(mass, startTransform, staticplaneShape6);

            //another static plane

            normal = new Vector3(0.5f, 0.7f, 0.0f);
            //normal.normalize();
            CollisionShape staticplaneShape7 = new StaticPlaneShape(normal, 0.0f);// A plane
            m_collisionShapes.Add(staticplaneShape7);
            startTransform.Translation = new Vector3(0, -10, 0);

            staticBody2 = LocalCreateRigidBody(mass, startTransform, staticplaneShape7);

            /// Create Static Torus
            float height = 28;
            float step = 2.5f;
            float massT = 1.0f;

            startTransform = Matrix.CreateFromQuaternion(Quaternion.CreateFromYawPitchRoll(3.14159265f * 0.5f, 0f, 3.14159265f * 0.5f));
            startTransform.Translation = new Vector3(0, height, -5);

            m_kinematicTorus = LocalCreateRigidBody(0.0f, startTransform, m_trimeshShape);

            m_kinematicTorus.SetCollisionFlags(m_kinematicTorus.GetCollisionFlags() | CollisionFlags.CF_KINEMATIC_OBJECT);
            m_kinematicTorus.SetActivationState(ActivationState.DISABLE_DEACTIVATION);

            /// Kinematic
            m_kinTorusTran = new Vector3(-0.1f, 0, 0);
            m_kinTorusRot = Quaternion.CreateFromYawPitchRoll(0f, 3.14159265f * 0.01f, 0f);

#if TEST_GIMPACT_TORUS

            /// Create dynamic Torus
            int numToroids = 1;
            for (int i = 0; i < numToroids; i++)
            {
                height -= step;
                startTransform = Matrix.CreateFromQuaternion(Quaternion.CreateFromYawPitchRoll(0f, 0f, 3.14159265f * 0.5f));
                startTransform.Translation = new Vector3(0, height, -5);

                RigidBody bodyA = LocalCreateRigidBody(massT, startTransform, m_trimeshShape);

                height -= step;
                startTransform = Matrix.CreateFromQuaternion(Quaternion.CreateFromYawPitchRoll(3.14159265f * 0.5f, 0f, 3.14159265f * 0.5f));
                startTransform.Translation = new Vector3(0, height, -5);
                RigidBody bodyB = LocalCreateRigidBody(massT, startTransform, m_trimeshShape);

            }
#endif

            startTransform = Matrix.Identity;

            /// Create Dynamic Boxes
            {
                int numBoxes = 6;
                for (int i = 0; i < numBoxes; i++)
                {
                    CollisionShape boxShape = new BoxShape(new Vector3(1, 1, 1));
                    m_collisionShapes.Add(boxShape);

                    startTransform.Translation = new Vector3(2 * i - 5, 2, -3);
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
			m_indexVertexArrays, Vector3(1.f,1.f,1.f),btScalar(0.01));
			trimesh->setMargin(0.07);
			trimesh->updateBound();


#else
                GImpactMeshShape trimesh = new GImpactMeshShape(m_indexVertexArrays);
                Vector3 scaling = Vector3.One;
                trimesh.SetLocalScaling(ref scaling);
                trimesh.SetMargin(0.07f); ///?????
                trimesh.UpdateBound();
#endif

                m_trimeshShape = trimesh;

            }

            /// Create Bunny Shape
            {
                m_indexVertexArrays2 = new TriangleIndexVertexArray(DemoMeshes.BUNNY_NUM_TRIANGLES, DemoMeshes.gBunnyIndices, 3, DemoMeshes.BUNNY_NUM_VERTICES, DemoMeshes.gBunnyVertices, 3);
#if BULLET_GIMPACT_CONVEX_DECOMPOSITION
			btGImpactConvexDecompositionShape * trimesh2  = new
			btGImpactConvexDecompositionShape(
			m_indexVertexArrays2, Vector3(4.f,4.f,4.f),btScalar(0.01));
			trimesh2->setMargin(0.07);
			trimesh2->updateBound();
#else
                GImpactMeshShape trimesh2 = new GImpactMeshShape(m_indexVertexArrays2);
                Vector3 scaling = new Vector3(4.0f, 4.0f, 4.0f);
                trimesh2.SetLocalScaling(ref scaling);
                trimesh2.SetMargin(0.07f); ///?????
                trimesh2.UpdateBound();
#endif
                m_trimeshShape2 = trimesh2;

            }


            ///register GIMPACT algorithm
            CollisionDispatcher dispatcher = m_dynamicsWorld.GetDispatcher() as CollisionDispatcher;

            GImpactCollisionAlgorithm.RegisterAlgorithm(dispatcher);




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
