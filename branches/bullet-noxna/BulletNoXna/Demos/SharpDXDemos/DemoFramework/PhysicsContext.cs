using System.Collections.Generic;
using BulletXNA;
using BulletXNA.BulletCollision;
using BulletXNA.BulletDynamics;
using BulletXNA.LinearMath;

namespace DemoFramework
{
    public class PhysicsContext : System.IDisposable
    {
        public DynamicsWorld World { get; protected set; }

        protected ICollisionConfiguration CollisionConf;
        protected CollisionDispatcher Dispatcher;
        protected IBroadphaseInterface Broadphase;
        protected IConstraintSolver Solver;
        public List<CollisionShape> CollisionShapes { get; private set; }

        protected BoxShape shootBoxShape;
        protected float shootBoxInitialSpeed = 40;

        public PhysicsContext()
        {
            CollisionShapes = new List<CollisionShape>();

            InitPhysics();
        }

        public virtual void InitPhysics()
        {
        }

        public void ExitPhysics()
        {
            //removed/dispose constraints
            int i;
            for (i = World.NumConstraints - 1; i >= 0; i--)
            {
                TypedConstraint constraint = World.GetConstraint(i);
                World.RemoveConstraint(constraint);
                constraint.Cleanup(); ;
            }

            //remove the rigidbodies from the dynamics world and delete them
            for (i = World.NumCollisionObjects - 1; i >= 0; i--)
            {
                CollisionObject obj = World.GetCollisionObjectArray()[i];
                RigidBody body = obj as RigidBody;
                if (body != null && body.MotionState != null)
                {
                    //body.MotionState.Cleanup();
                }
                World.RemoveCollisionObject(obj);
                obj.Cleanup();
            }

            //delete collision shapes
            foreach (CollisionShape shape in CollisionShapes)
                shape.Cleanup();
            CollisionShapes.Clear();

            World.Cleanup();
            Broadphase.Cleanup();
            Dispatcher.Cleanup();
        }

        public void ClientResetScene()
        {
            ExitPhysics();
            InitPhysics();
        }

        public virtual void Dispose()
        {
            ExitPhysics();
        }

        public virtual int Update(float elapsedTime)
        {
            return World.StepSimulation(elapsedTime, 1);
        }

        public virtual RigidBody LocalCreateRigidBody(float mass, Matrix startTransform, CollisionShape shape)
        {
            //rigidbody is dynamic if and only if mass is non zero, otherwise static
            bool isDynamic = (mass != 0.0f);

            Vector3 localInertia = Vector3.Zero;
            if (isDynamic)
                shape.CalculateLocalInertia(mass, out localInertia);

            //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
            DefaultMotionState myMotionState = new DefaultMotionState(startTransform, Matrix.Identity);

            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, shape, localInertia);
            RigidBody body = new RigidBody(rbInfo);

            World.AddRigidBody(body);

            return body;
        }

        public virtual void ShootBox(Vector3 camPos, Vector3 destination)
        {
            if (World == null)
                return;

            float mass = 1.0f;

            if (shootBoxShape == null)
            {
                shootBoxShape = new BoxShape(new Vector3(1.0f, 1.0f, 1.0f));
                //shootBoxShape.InitializePolyhedralFeatures();
            }

            RigidBody body = LocalCreateRigidBody(mass, Matrix.CreateTranslation(camPos), shootBoxShape);
            body.SetLinearFactor(new Vector3(1, 1, 1));
            //body.Restitution = 1;

            Vector3 linVel = destination - camPos;
            linVel.Normalize();

            body.LinearVelocity = linVel * shootBoxInitialSpeed;
            body.CcdMotionThreshold = 0.5f;
            body.CcdSweptSphereRadius = 0.9f;
        }
    };
};
