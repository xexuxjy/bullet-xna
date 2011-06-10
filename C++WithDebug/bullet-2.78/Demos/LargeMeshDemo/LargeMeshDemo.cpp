/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//#define TEST_SERIALIZATION 1

///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 2
#define ARRAY_SIZE_Y 4
#define ARRAY_SIZE_Z 2

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "LargeMeshDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#ifdef TEST_SERIALIZATION
#include "LinearMath/btSerializer.h"
#endif //TEST_SERIALIZATION

#include <stdio.h> //printf debugging


void LargeMeshDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
		
	renderme(); 

	glFlush();

	swapBuffers();

}

//static int numTriangles = 14;
//static btVector3 vertices[] = {
//	btVector3(1.78321f ,1.78321f ,-1.783479f),
//	btVector3(1.78321f ,-1.78321f ,-1.783479f),
//	btVector3(-1.78321f ,-1.78321f ,-1.783479f),
//	btVector3(-1.78321f ,1.78321f ,-1.783479f),
//	btVector3(0.0f ,0.0f ,1.783479f),
//	btVector3(0.0f ,0.0f ,1.783479f),
//	btVector3(0.0f ,0.0f ,1.783479f),
//	btVector3(0.0f ,0.0f ,1.783479f),
//	btVector3(-1.78321f ,-1.78321f ,-1.783479f),
//	btVector3(0.0f ,0.0f ,1.783479f)
//};
//
//static int indices[] = {0,1,2,3,0,2,4,5,6,4,6,7,4,2,1,4,1,5,5,1,0,5,0,6,6,0,3,6,3,7,7,3,8,7,8,9,9,8,2,9,2,4};
//

static int numTriangles = 2;

static btVector3 vertices[] = {
	btVector3(-5,0 ,-5),
	btVector3(5 ,-0,-5),
	btVector3(5,0,5),
	btVector3(-5,-0,5)
};

//static Vector3[] vertices = {
//    new Vector3(1.78321f ,1.78321f ,-1.783479f),
//    new Vector3(1.78321f ,-1.78321f ,-1.783479f),
//    new Vector3(-1.78321f ,-1.78321f ,-1.783479f),
//    new Vector3(-1.78321f ,1.78321f ,-1.783479f)
//                            };


static int indices[] = { 0, 1,2 , 0, 2,3 };

btCollisionShape* LargeMeshDemo::BuildLargeMesh()
{
	int vertStride = sizeof(btVector3);
	int indexStride = 3*sizeof(int);

	//btTriangleIndexVertexArray* vertexArray = new btTriangleIndexVertexArray(numTriangles,&indices[0],indexStride,10,((btScalar*)&vertices[0]),vertStride);

	btTriangleIndexVertexArray* vertexArray = new btTriangleIndexVertexArray(numTriangles,&indices[0],indexStride,4,((btScalar*)&vertices[0]),vertStride);
	btTriangleMeshShape* triangleMesh = new btTriangleMeshShape(vertexArray);
	return triangleMesh;

}



void LargeMeshDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}





void	LargeMeshDemo::initPhysics()
{
	g_file = fopen("e:/users/man/bullet/cpp-largemesh-output.txt","wb");
	setTexturing(true);
	setShadows(true);

	//g_file = fopen("C:/users/man/cpp-basic-output.txt","w");

	setCameraDistance(btScalar(SCALING*20.));

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
	m_broadphase = new btSimpleBroadphase(1000,NULL);

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btCollisionShape* groundShape = BuildLargeMesh();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	//groundTransform.getBasis().setEulerYPR(0,0,SIMD_PI/2.0f);
	//Matrix rotateMatrix = Matrix.CreateFromYawPitchRoll(0, 0, MathUtil.SIMD_PI*0.8f);
	groundTransform.getBasis().setEulerYPR(SIMD_PI*0.8f,0,0);
	groundTransform.setOrigin(btVector3(0,0,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);
		localCreateRigidBody(mass,groundTransform,groundShape);
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		//bool isDynamic = (mass != 0.f);

		//btVector3 localInertia(0,0,0);
		//if (isDynamic)
		//	groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		//btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		//btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		//btRigidBody* body = new btRigidBody(rbInfo);

		////add the body to the dynamics world
		//m_dynamicsWorld->addRigidBody(body);
	}

	btCollisionShape* boxShape = new btBoxShape(btVector3(0.2f,0.2f,0.2f));

	btTransform boxTransform;
	boxTransform.setIdentity();
	boxTransform.setOrigin(btVector3(0,3,0));


	localCreateRigidBody(1.25f,boxTransform,boxShape);

	clientResetScene();


#ifdef TEST_SERIALIZATION
	//test serializing this 

	int maxSerializeBufferSize = 1024*1024*5;

	btDefaultSerializer*	serializer = new btDefaultSerializer(maxSerializeBufferSize);
	m_dynamicsWorld->serialize(serializer);
	
	FILE* f2 = fopen("testFile.bullet","wb");
	fwrite(serializer->m_buffer,serializer->m_currentSize,1,f2);
	fclose(f2);
#endif

#if 0
	bParse::btBulletFile* bulletFile2 = new bParse::btBulletFile("testFile.bullet");
	bool ok = (bulletFile2->getFlags()& bParse::FD_OK)!=0;
	bool verboseDumpAllTypes = true;
	if (ok)
		bulletFile2->parse(verboseDumpAllTypes);
	
	if (verboseDumpAllTypes)
	{
		bulletFile2->dumpChunks(bulletFile2->getFileDNA());
	}
#endif //TEST_SERIALIZATION

}
	

void	LargeMeshDemo::exitPhysics()
{
	if(g_file)
	{
		fclose(g_file);
	}
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}




