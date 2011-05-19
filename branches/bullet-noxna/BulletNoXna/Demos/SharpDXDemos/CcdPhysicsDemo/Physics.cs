using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using DemoFramework;
using Matrix = BulletXNA.LinearMath.Matrix;
using Vector3 = BulletXNA.LinearMath.Vector3;

namespace CcdPhysicsDemo
{
    class Physics : PhysicsContext
    {
        bool ccdMode = true;

        public bool CcdMode
        {
            get { return ccdMode; }
        }

        float CubeHalfExtents = 0.5f;
        float ExtraHeight = 1.0f;

        public void ToggleCcdMode()
        {
            ccdMode = !ccdMode;

            ClientResetScene();
        }

        void CreateStack(CollisionShape boxShape, int size, float zPos)
        {
            Matrix trans;
            float mass = 1.0f;

            for (int i = 0; i < size; i++)
            {
                // This constructs a row, from left to right
                int rowSize = size - i;
                for (int j = 0; j < rowSize; j++)
                {
                    trans = Matrix.CreateTranslation(
                        -rowSize * CubeHalfExtents + CubeHalfExtents + j * 2.0f * CubeHalfExtents,
                        CubeHalfExtents + i * CubeHalfExtents * 2.0f,
                        zPos);

                    RigidBody body = LocalCreateRigidBody(mass, trans, boxShape);
                    body.ActivationState = ActivationState.IslandSleeping;
                }
            }
        }

        public override void InitPhysics()
        {
            int i;

            shootBoxInitialSpeed = 4000;

            // collision configuration contains default setup for memory, collision setup
            CollisionConf = new DefaultCollisionConfiguration();

            Dispatcher = new CollisionDispatcher(CollisionConf);
            Dispatcher.RegisterCollisionCreateFunc((int)BroadphaseNativeType.BoxShape, (int)BroadphaseNativeType.BoxShape,
                CollisionConf.GetCollisionAlgorithmCreateFunc(BroadphaseNativeType.ConvexShape, BroadphaseNativeType.ConvexShape));

            Broadphase = new DbvtBroadphase();


            // the default constraint solver.
            Solver = new SequentialImpulseConstraintSolver();

            World = new DiscreteDynamicsWorld(Dispatcher, Broadphase, Solver, CollisionConf);
            World.GetSolverInfo().m_splitImpulse = true;
            World.GetSolverInfo().m_numIterations = 20;

            //World.GetDispatchInfo().UseContinuous = ccdMode;

            Vector3 g = new Vector3(0, -10, 0);
            World.SetGravity(ref g);

            BoxShape ground = new BoxShape(new Vector3(200, 1, 200));
            //ground.InitializePolyhedralFeatures();
            CollisionShapes.Add(ground);
            RigidBody body = LocalCreateRigidBody(0, Matrix.Identity, ground);
            body.UserObject = "Ground";

            CollisionShape shape = new CylinderShape(new Vector3(CubeHalfExtents));
            CollisionShapes.Add(shape);

            int numObjects = 120;
            for (i = 0; i < numObjects; i++)
            {
                //stack them
                int colsize = 10;
                int row = (int)((i * CubeHalfExtents * 2) / (colsize * 2 * CubeHalfExtents));
                int row2 = row;
                int col = (i) % (colsize) - colsize / 2;

                if (col > 3)
                {
                    col = 11;
                    row2 |= 1;
                }

                Matrix trans = Matrix.CreateTranslation(col * 2 * CubeHalfExtents + (row2 % 2) * CubeHalfExtents,
                    row * 2 * CubeHalfExtents + CubeHalfExtents + ExtraHeight, 0);

                body = LocalCreateRigidBody(1, trans, shape);

                if (ccdMode)
                {
                    body.CcdMotionThreshold = 1e-7f;
                    body.CcdSweptSphereRadius = 0.9f * CubeHalfExtents;
                }
            }
        }

        public override void ShootBox(Vector3 camPos, Vector3 destination)
        {
            if (World != null)
            {
                float mass = 1.0f;

                if (shootBoxShape == null)
                {
                    shootBoxShape = new BoxShape(new Vector3(1.0f));
                    //shootBoxShape.InitializePolyhedralFeatures();
                }

                RigidBody body = LocalCreateRigidBody(mass, Matrix.CreateTranslation(camPos), shootBoxShape);
                body.SetLinearFactor(new Vector3(1, 1, 1));
                //body->setRestitution(1);

                Vector3 linVel = destination - camPos;
                linVel.Normalize();
                body.LinearVelocity = linVel * shootBoxInitialSpeed;
                body.SetAngularVelocity(Vector3.Zero);
                body.SetContactProcessingThreshold(1e30f);

                ///when using m_ccdMode, disable regular CCD
                if (ccdMode)
                {
                    body.CcdMotionThreshold = 0.0001f;
                    body.CcdSweptSphereRadius = 0.4f;
                }
            }
        }
    }
}
