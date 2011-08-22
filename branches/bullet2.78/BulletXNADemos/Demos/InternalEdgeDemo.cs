#define SWAP_WINDING
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BulletXNA.BulletCollision;
using BulletXNA.LinearMath;
using BulletXNA.BulletDynamics;
using BulletXNA.BulletCollision.CollisionDispatch;
using BulletXNA;

namespace BulletXNADemos.Demos
{
    public class InternalEdgeDemo : DemoApplication
    {
        public InternalEdgeDemo()
        {
            m_animatedMesh = true;
        }


        public override void InitializeDemo()
        {
            int totalTriangles = 2 * (NUM_VERTS_X - 1) * (NUM_VERTS_Y - 1);

            int vertStride = 1;
            int indexStride = 3;

            BulletGlobals.gContactAddedCallback = new CustomMaterialCombinerCallback();


            gVertices = new ObjectArray<IndexedVector3>(totalVerts);
            gIndices = new ObjectArray<int>(totalTriangles * 3);

            SetVertexPositions(waveheight, 0.0f);
            
            gVertices.GetRawArray()[1].Y = 0.1f;


	int index=0;
    int i, j;
	for (i=0;i<NUM_VERTS_X-1;i++)
	{
		for (j=0;j<NUM_VERTS_Y-1;j++)
		{

#if SWAP_WINDING
#if SHIFT_INDICES
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			
#else
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			gIndices[index++] = j*NUM_VERTS_X+i;

			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = j*NUM_VERTS_X+i;
#endif //SHIFT_INDICES
#else //SWAP_WINDING

#if SHIFT_INDICES
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i+1;

#if TEST_INCONSISTENT_WINDING
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

#else //TEST_INCONSISTENT_WINDING
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
#endif //TEST_INCONSISTENT_WINDING
			
			
			
#else //SHIFT_INDICES
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
#endif //SHIFT_INDICES

#endif //SWAP_WINDING
        }
    }

            m_indexVertexArrays = new TriangleIndexVertexArray(totalTriangles,
                gIndices,
                indexStride,
                totalVerts, gVertices, vertStride);

            bool useQuantizedAabbCompression = true;

             IndexedVector3  aabbMin = new IndexedVector3 (-1000,-1000,-1000);
             IndexedVector3 aabbMax = new IndexedVector3(1000, 1000, 1000);
	
            trimeshShape  = new BvhTriangleMeshShape(m_indexVertexArrays,useQuantizedAabbCompression,ref aabbMin,ref aabbMax,true);

            CollisionShape groundShape = trimeshShape;

            TriangleInfoMap triangleInfoMap = new TriangleInfoMap();

            InternalEdgeUtility.GenerateInternalEdgeInfo(trimeshShape, triangleInfoMap);


            m_collisionConfiguration = new DefaultCollisionConfiguration();


            m_dispatcher = new CollisionDispatcher(m_collisionConfiguration);



            m_broadphase = new DbvtBroadphase();
            m_constraintSolver = new SequentialImpulseConstraintSolver();
            m_dynamicsWorld = new DiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_constraintSolver, m_collisionConfiguration);
            m_dynamicsWorld.SetDebugDrawer(m_debugDraw);

            IndexedVector3 gravity = new IndexedVector3(0,-10,0);
	        m_dynamicsWorld.SetGravity(ref gravity);

	
	        float mass = 0.0f;
	        IndexedMatrix startTransform = IndexedMatrix.CreateTranslation(new IndexedVector3(0,-2,0));

            ConvexHullShape colShape = new ConvexHullShape(new List<IndexedVector3>(), 0);
	        for (int k=0;k<DemoMeshes.TaruVtxCount;k++)
	        {
                IndexedVector3 vtx = DemoMeshes.TaruVtx[k];
		        colShape.AddPoint(ref vtx);
	        }
	        //this will enable polyhedral contact clipping, better quality, slightly slower
            //colShape.InitializePolyhedralFeatures();

	        //the polyhedral contact clipping can use either GJK or SAT test to find the separating axis
	        m_dynamicsWorld.GetDispatchInfo().m_enableSatConvex=false;


            {
                for (int i2 = 0; i2 < 1; i2++)
                {
                    startTransform._origin = new IndexedVector3(-10.0f + i2 * 3.0f, 2.2f + i2 * 0.1f, -1.3f);
                    RigidBody body = LocalCreateRigidBody(10, startTransform, colShape);
                    body.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
                    body.SetLinearVelocity(new IndexedVector3(0, 0, -1));
                    //body->setContactProcessingThreshold(0.f);
                }
            }
            {
                BoxShape colShape2 = new BoxShape(new IndexedVector3(1, 1, 1));
                //colShape.InitializePolyhedralFeatures();
                m_collisionShapes.Add(colShape2);
                startTransform._origin = new IndexedVector3(-16.0f + i * 3.0f, 1.0f + i * 0.1f, -1.3f);
                RigidBody body = LocalCreateRigidBody(10, startTransform, colShape2);
                body.SetActivationState(ActivationState.DISABLE_DEACTIVATION);
                body.SetLinearVelocity(new IndexedVector3(0, 0, -1));
            }

            startTransform = IndexedMatrix.Identity;

            staticBody = LocalCreateRigidBody(mass, startTransform,groundShape);
	        //staticBody->setContactProcessingThreshold(-0.031f);
	        staticBody.SetCollisionFlags(staticBody.GetCollisionFlags() | CollisionFlags.CF_KINEMATIC_OBJECT);//STATIC_OBJECT);

	        //enable custom material callback
	        staticBody.SetCollisionFlags(staticBody.GetCollisionFlags()  | CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK);
	        SetDebugMode(DebugDrawModes.DBG_DrawText|DebugDrawModes.DBG_NoHelpText|DebugDrawModes.DBG_DrawWireframe|DebugDrawModes.DBG_DrawContactPoints);



            //base.InitializeDemo();
            //ClientResetScene();

        }


        public void SetVertexPositions(float waveheight, float offset)
        {

            for (int i = 0; i < NUM_VERTS_X; i++)
            {
                for (int j = 0; j < NUM_VERTS_Y; j++)
                {
                    gVertices[i + j * NUM_VERTS_X] = new IndexedVector3(
                        (i - NUM_VERTS_X * 0.5f) * TRIANGLE_SIZE,
                        //0.f,
                        waveheight * (float)Math.Sin((float)i + offset) * (float)Math.Cos((float)j + offset),
                        (j - NUM_VERTS_Y * 0.5f) * TRIANGLE_SIZE);
                }
            }

        }

        protected override void Update(Microsoft.Xna.Framework.GameTime gameTime)
        {
            if (m_animatedMesh)
            {
                offset += 0.01f;

                IndexedVector3 aabbMin, aabbMax;
                trimeshShape.GetMeshInterface().CalculateAabbBruteForce(out aabbMin, out aabbMax);
                trimeshShape.RefitTree(ref aabbMin, ref aabbMax);


            }
            base.Update(gameTime);
        }



        private bool m_animatedMesh;
        private TriangleIndexVertexArray m_indexVertexArrays;
        private const int NUM_VERTS_X = 2;
        private const int NUM_VERTS_Y = 2;
        private const int totalVerts = NUM_VERTS_X * NUM_VERTS_Y;
        private ObjectArray<IndexedVector3> gVertices = null;
        private ObjectArray<int> gIndices = null;
        static BvhTriangleMeshShape trimeshShape = null;
        static RigidBody staticBody = null;
        static float waveheight = 0.0f;
        static float offset = 0.0f;

        const float TRIANGLE_SIZE = 20.0f;


        static void Main(string[] args)
        {
            using (InternalEdgeDemo game = new InternalEdgeDemo())
            {
                game.Run();
            }
        }

    }


    public class CustomMaterialCombinerCallback : IContactAddedCallback
    {

        public bool Callback(ref ManifoldPoint cp, CollisionObject colObj0, int partId0, int index0, CollisionObject colObj1, int partId1, int index1)
        {
            bool enable = true;
           	if (enable)
	        {
		        InternalEdgeUtility.AdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1,BulletXNA.BulletCollision.CollisionDispatch.InternalEdgeUtility.InternalEdgeAdjustFlags.BT_TRIANGLE_CONVEX_BACKFACE_NONE);
		        //btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_BACKFACE_MODE);
		        //btAdjustInternalEdgeContacts(cp,colObj1,colObj0, partId1,index1, BT_TRIANGLE_CONVEX_DOUBLE_SIDED+BT_TRIANGLE_CONCAVE_DOUBLE_SIDED);
	        }

	        float friction0 = colObj0.GetFriction();
	        float friction1 = colObj1.GetFriction();
	        float restitution0 = colObj0.GetRestitution();
	        float restitution1 = colObj1.GetRestitution();

	        if ((colObj0.GetCollisionFlags() & CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK) != 0)
	        {
		        friction0 = 1.0f;//partId0,index0
		        restitution0 = 0.0f;
	        }
	        if ((colObj1.GetCollisionFlags() & CollisionFlags.CF_CUSTOM_MATERIAL_CALLBACK) != 0)
	        {
		        if ((index1&1) != 0)
		        {
			        friction1 = 1.0f;//partId1,index1
		        } 
                else
		        {
			        friction1 = 0.0f;
		        }
		        restitution1 = 0.0f;
	        }

	        cp.m_combinedFriction = CalculateCombinedFriction(friction0,friction1);
	        cp.m_combinedRestitution = CalculateCombinedRestitution(restitution0,restitution1);

	        //this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
	        return true;
 
        }

        ///User can override this material combiner by implementing gContactAddedCallback and setting body0->m_collisionFlags |= btCollisionObject::customMaterialCallback;
        public float CalculateCombinedFriction(float friction0, float friction1)
        {
            return 0.0f;
            float friction = friction0 * friction1;

            const float MAX_FRICTION = 10.0f;
            if (friction < -MAX_FRICTION)
                friction = -MAX_FRICTION;
            if (friction > MAX_FRICTION)
                friction = MAX_FRICTION;
            return friction;

        }

        public float CalculateCombinedRestitution(float restitution0, float restitution1)
        {
            return restitution0 * restitution1;
        }



    }



}
