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


///create 125 (5x5x5) dynamic object

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.

#include "BvhBugDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"

#include <stdio.h> //printf debugging


void BvhBugDemo::clientMoveAndDisplay()
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



void BvhBugDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}





void	BvhBugDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(SCALING*50.));

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}

	btCollisionShape* shape = SetupShape();
	btTransform objTransform;
	objTransform.setIdentity();
	objTransform.setOrigin(btVector3(0,2,0));

	localCreateRigidBody(mass, objTransform, shape);


}
void	BvhBugDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}


void	BvhBugDemo::exitPhysics()
{

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
	m_collisionShapes.clear();

	delete m_dynamicsWorld;

	delete m_solver;

	delete m_broadphase;

	delete m_dispatcher;

	delete m_collisionConfiguration;


}




btCollisionShape* CreateMeshShape2(int indicesCount, int* indices, int verticesCount, float* vertices )
{
    //We must copy the indices and vertices since the passed memory is released when this call returns.
    btIndexedMesh indexedMesh;
    int* copiedIndices = new int[indicesCount];
    bsMemcpy(copiedIndices, indices, indicesCount * sizeof(int));
    int numVertices = verticesCount * 3;
    float* copiedVertices = new float[numVertices];
    bsMemcpy(copiedVertices, vertices, numVertices * sizeof(float));
    indexedMesh.m_indexType = PHY_INTEGER;
    indexedMesh.m_triangleIndexBase = (const unsigned char*)copiedIndices;
    indexedMesh.m_triangleIndexStride = sizeof(int) * 3;
    indexedMesh.m_numTriangles = indicesCount / 3;
    indexedMesh.m_vertexType = PHY_FLOAT;
    indexedMesh.m_numVertices = verticesCount;
    indexedMesh.m_vertexBase = (const unsigned char*)copiedVertices;
    indexedMesh.m_vertexStride = sizeof(float) * 3;
    btTriangleIndexVertexArray* vertexArray = new btTriangleIndexVertexArray();
    vertexArray->addIndexedMesh(indexedMesh, PHY_INTEGER);
    btBvhTriangleMeshShape* meshShape = new btBvhTriangleMeshShape(vertexArray, true, true);
    meshShape->setMargin(m_worldData.params->collisionMargin);
    return meshShape;
}





public CollisionShape SetupShape()
{
	//Indices
	delete[] indicies;
	indices = new int[744];
	indices[0] = 0;
	indices[1] = 0;
	indices[2] = 1;
	indices[3] = 0;
	indices[4] = 1;
	indices[5] = 2;
	indices[6] = 0;
	indices[7] = 2;
	indices[8] = 3;
	indices[9] = 0;
	indices[10] = 3;
	indices[11] = 4;
	indices[12] = 5;
	indices[13] = 0;
	indices[14] = 6;
	indices[15] = 6;
	indices[16] = 0;
	indices[17] = 1;
	indices[18] = 6;
	indices[19] = 1;
	indices[20] = 7;
	indices[21] = 7;
	indices[22] = 1;
	indices[23] = 2;
	indices[24] = 7;
	indices[25] = 2;
	indices[26] = 8;
	indices[27] = 8;
	indices[28] = 2;
	indices[29] = 3;
	indices[30] = 8;
	indices[31] = 3;
	indices[32] = 9;
	indices[33] = 9;
	indices[34] = 3;
	indices[35] = 4;
	indices[36] = 9;
	indices[37] = 4;
	indices[38] = 5;
	indices[39] = 5;
	indices[40] = 4;
	indices[41] = 0;
	indices[42] = 10;
	indices[43] = 5;
	indices[44] = 11;
	indices[45] = 11;
	indices[46] = 5;
	indices[47] = 6;
	indices[48] = 11;
	indices[49] = 6;
	indices[50] = 12;
	indices[51] = 12;
	indices[52] = 6;
	indices[53] = 7;
	indices[54] = 12;
	indices[55] = 7;
	indices[56] = 13;
	indices[57] = 13;
	indices[58] = 7;
	indices[59] = 8;
	indices[60] = 13;
	indices[61] = 8;
	indices[62] = 14;
	indices[63] = 14;
	indices[64] = 8;
	indices[65] = 9;
	indices[66] = 14;
	indices[67] = 9;
	indices[68] = 10;
	indices[69] = 10;
	indices[70] = 9;
	indices[71] = 5;
	indices[72] = 15;
	indices[73] = 10;
	indices[74] = 16;
	indices[75] = 16;
	indices[76] = 10;
	indices[77] = 11;
	indices[78] = 16;
	indices[79] = 11;
	indices[80] = 17;
	indices[81] = 17;
	indices[82] = 11;
	indices[83] = 12;
	indices[84] = 17;
	indices[85] = 12;
	indices[86] = 18;
	indices[87] = 18;
	indices[88] = 12;
	indices[89] = 13;
	indices[90] = 18;
	indices[91] = 13;
	indices[92] = 19;
	indices[93] = 19;
	indices[94] = 13;
	indices[95] = 14;
	indices[96] = 19;
	indices[97] = 14;
	indices[98] = 15;
	indices[99] = 15;
	indices[100] = 14;
	indices[101] = 10;
	indices[102] = 20;
	indices[103] = 15;
	indices[104] = 21;
	indices[105] = 21;
	indices[106] = 15;
	indices[107] = 16;
	indices[108] = 21;
	indices[109] = 16;
	indices[110] = 22;
	indices[111] = 22;
	indices[112] = 16;
	indices[113] = 17;
	indices[114] = 22;
	indices[115] = 17;
	indices[116] = 23;
	indices[117] = 23;
	indices[118] = 17;
	indices[119] = 18;
	indices[120] = 23;
	indices[121] = 18;
	indices[122] = 24;
	indices[123] = 24;
	indices[124] = 18;
	indices[125] = 19;
	indices[126] = 24;
	indices[127] = 19;
	indices[128] = 20;
	indices[129] = 20;
	indices[130] = 19;
	indices[131] = 15;
	indices[132] = 25;
	indices[133] = 20;
	indices[134] = 26;
	indices[135] = 26;
	indices[136] = 20;
	indices[137] = 21;
	indices[138] = 26;
	indices[139] = 21;
	indices[140] = 27;
	indices[141] = 27;
	indices[142] = 21;
	indices[143] = 22;
	indices[144] = 27;
	indices[145] = 22;
	indices[146] = 28;
	indices[147] = 28;
	indices[148] = 22;
	indices[149] = 23;
	indices[150] = 28;
	indices[151] = 23;
	indices[152] = 29;
	indices[153] = 29;
	indices[154] = 23;
	indices[155] = 24;
	indices[156] = 29;
	indices[157] = 24;
	indices[158] = 25;
	indices[159] = 25;
	indices[160] = 24;
	indices[161] = 20;
	indices[162] = 30;
	indices[163] = 25;
	indices[164] = 31;
	indices[165] = 31;
	indices[166] = 25;
	indices[167] = 26;
	indices[168] = 31;
	indices[169] = 26;
	indices[170] = 32;
	indices[171] = 32;
	indices[172] = 26;
	indices[173] = 27;
	indices[174] = 32;
	indices[175] = 27;
	indices[176] = 33;
	indices[177] = 33;
	indices[178] = 27;
	indices[179] = 28;
	indices[180] = 33;
	indices[181] = 28;
	indices[182] = 34;
	indices[183] = 34;
	indices[184] = 28;
	indices[185] = 29;
	indices[186] = 34;
	indices[187] = 29;
	indices[188] = 30;
	indices[189] = 30;
	indices[190] = 29;
	indices[191] = 25;
	indices[192] = 35;
	indices[193] = 30;
	indices[194] = 36;
	indices[195] = 36;
	indices[196] = 30;
	indices[197] = 31;
	indices[198] = 36;
	indices[199] = 31;
	indices[200] = 37;
	indices[201] = 37;
	indices[202] = 31;
	indices[203] = 32;
	indices[204] = 37;
	indices[205] = 32;
	indices[206] = 38;
	indices[207] = 38;
	indices[208] = 32;
	indices[209] = 33;
	indices[210] = 38;
	indices[211] = 33;
	indices[212] = 39;
	indices[213] = 39;
	indices[214] = 33;
	indices[215] = 34;
	indices[216] = 39;
	indices[217] = 34;
	indices[218] = 35;
	indices[219] = 35;
	indices[220] = 34;
	indices[221] = 30;
	indices[222] = 40;
	indices[223] = 35;
	indices[224] = 41;
	indices[225] = 41;
	indices[226] = 35;
	indices[227] = 36;
	indices[228] = 41;
	indices[229] = 36;
	indices[230] = 42;
	indices[231] = 42;
	indices[232] = 36;
	indices[233] = 37;
	indices[234] = 42;
	indices[235] = 37;
	indices[236] = 43;
	indices[237] = 43;
	indices[238] = 37;
	indices[239] = 38;
	indices[240] = 43;
	indices[241] = 38;
	indices[242] = 44;
	indices[243] = 44;
	indices[244] = 38;
	indices[245] = 39;
	indices[246] = 44;
	indices[247] = 39;
	indices[248] = 40;
	indices[249] = 40;
	indices[250] = 39;
	indices[251] = 35;
	indices[252] = 45;
	indices[253] = 40;
	indices[254] = 46;
	indices[255] = 46;
	indices[256] = 40;
	indices[257] = 41;
	indices[258] = 46;
	indices[259] = 41;
	indices[260] = 47;
	indices[261] = 47;
	indices[262] = 41;
	indices[263] = 42;
	indices[264] = 47;
	indices[265] = 42;
	indices[266] = 48;
	indices[267] = 48;
	indices[268] = 42;
	indices[269] = 43;
	indices[270] = 48;
	indices[271] = 43;
	indices[272] = 49;
	indices[273] = 49;
	indices[274] = 43;
	indices[275] = 44;
	indices[276] = 49;
	indices[277] = 44;
	indices[278] = 45;
	indices[279] = 45;
	indices[280] = 44;
	indices[281] = 40;
	indices[282] = 50;
	indices[283] = 45;
	indices[284] = 51;
	indices[285] = 51;
	indices[286] = 45;
	indices[287] = 46;
	indices[288] = 51;
	indices[289] = 46;
	indices[290] = 52;
	indices[291] = 52;
	indices[292] = 46;
	indices[293] = 47;
	indices[294] = 52;
	indices[295] = 47;
	indices[296] = 53;
	indices[297] = 53;
	indices[298] = 47;
	indices[299] = 48;
	indices[300] = 53;
	indices[301] = 48;
	indices[302] = 54;
	indices[303] = 54;
	indices[304] = 48;
	indices[305] = 49;
	indices[306] = 54;
	indices[307] = 49;
	indices[308] = 50;
	indices[309] = 50;
	indices[310] = 49;
	indices[311] = 45;
	indices[312] = 55;
	indices[313] = 50;
	indices[314] = 56;
	indices[315] = 56;
	indices[316] = 50;
	indices[317] = 51;
	indices[318] = 56;
	indices[319] = 51;
	indices[320] = 57;
	indices[321] = 57;
	indices[322] = 51;
	indices[323] = 52;
	indices[324] = 57;
	indices[325] = 52;
	indices[326] = 58;
	indices[327] = 58;
	indices[328] = 52;
	indices[329] = 53;
	indices[330] = 58;
	indices[331] = 53;
	indices[332] = 59;
	indices[333] = 59;
	indices[334] = 53;
	indices[335] = 54;
	indices[336] = 59;
	indices[337] = 54;
	indices[338] = 55;
	indices[339] = 55;
	indices[340] = 54;
	indices[341] = 50;
	indices[342] = 60;
	indices[343] = 55;
	indices[344] = 61;
	indices[345] = 61;
	indices[346] = 55;
	indices[347] = 56;
	indices[348] = 61;
	indices[349] = 56;
	indices[350] = 62;
	indices[351] = 62;
	indices[352] = 56;
	indices[353] = 57;
	indices[354] = 62;
	indices[355] = 57;
	indices[356] = 63;
	indices[357] = 63;
	indices[358] = 57;
	indices[359] = 58;
	indices[360] = 63;
	indices[361] = 58;
	indices[362] = 64;
	indices[363] = 64;
	indices[364] = 58;
	indices[365] = 59;
	indices[366] = 64;
	indices[367] = 59;
	indices[368] = 60;
	indices[369] = 60;
	indices[370] = 59;
	indices[371] = 55;
	indices[372] = 65;
	indices[373] = 60;
	indices[374] = 66;
	indices[375] = 66;
	indices[376] = 60;
	indices[377] = 61;
	indices[378] = 66;
	indices[379] = 61;
	indices[380] = 67;
	indices[381] = 67;
	indices[382] = 61;
	indices[383] = 62;
	indices[384] = 67;
	indices[385] = 62;
	indices[386] = 68;
	indices[387] = 68;
	indices[388] = 62;
	indices[389] = 63;
	indices[390] = 68;
	indices[391] = 63;
	indices[392] = 69;
	indices[393] = 69;
	indices[394] = 63;
	indices[395] = 64;
	indices[396] = 69;
	indices[397] = 64;
	indices[398] = 65;
	indices[399] = 65;
	indices[400] = 64;
	indices[401] = 60;
	indices[402] = 70;
	indices[403] = 65;
	indices[404] = 71;
	indices[405] = 71;
	indices[406] = 65;
	indices[407] = 66;
	indices[408] = 71;
	indices[409] = 66;
	indices[410] = 72;
	indices[411] = 72;
	indices[412] = 66;
	indices[413] = 67;
	indices[414] = 72;
	indices[415] = 67;
	indices[416] = 73;
	indices[417] = 73;
	indices[418] = 67;
	indices[419] = 68;
	indices[420] = 73;
	indices[421] = 68;
	indices[422] = 74;
	indices[423] = 74;
	indices[424] = 68;
	indices[425] = 69;
	indices[426] = 74;
	indices[427] = 69;
	indices[428] = 70;
	indices[429] = 70;
	indices[430] = 69;
	indices[431] = 65;
	indices[432] = 75;
	indices[433] = 70;
	indices[434] = 76;
	indices[435] = 76;
	indices[436] = 70;
	indices[437] = 71;
	indices[438] = 76;
	indices[439] = 71;
	indices[440] = 77;
	indices[441] = 77;
	indices[442] = 71;
	indices[443] = 72;
	indices[444] = 77;
	indices[445] = 72;
	indices[446] = 78;
	indices[447] = 78;
	indices[448] = 72;
	indices[449] = 73;
	indices[450] = 78;
	indices[451] = 73;
	indices[452] = 79;
	indices[453] = 79;
	indices[454] = 73;
	indices[455] = 74;
	indices[456] = 79;
	indices[457] = 74;
	indices[458] = 75;
	indices[459] = 75;
	indices[460] = 74;
	indices[461] = 70;
	indices[462] = 80;
	indices[463] = 75;
	indices[464] = 81;
	indices[465] = 81;
	indices[466] = 75;
	indices[467] = 76;
	indices[468] = 81;
	indices[469] = 76;
	indices[470] = 82;
	indices[471] = 82;
	indices[472] = 76;
	indices[473] = 77;
	indices[474] = 82;
	indices[475] = 77;
	indices[476] = 83;
	indices[477] = 83;
	indices[478] = 77;
	indices[479] = 78;
	indices[480] = 83;
	indices[481] = 78;
	indices[482] = 84;
	indices[483] = 84;
	indices[484] = 78;
	indices[485] = 79;
	indices[486] = 84;
	indices[487] = 79;
	indices[488] = 80;
	indices[489] = 80;
	indices[490] = 79;
	indices[491] = 75;
	indices[492] = 85;
	indices[493] = 80;
	indices[494] = 86;
	indices[495] = 86;
	indices[496] = 80;
	indices[497] = 81;
	indices[498] = 86;
	indices[499] = 81;
	indices[500] = 87;
	indices[501] = 87;
	indices[502] = 81;
	indices[503] = 82;
	indices[504] = 87;
	indices[505] = 82;
	indices[506] = 88;
	indices[507] = 88;
	indices[508] = 82;
	indices[509] = 83;
	indices[510] = 88;
	indices[511] = 83;
	indices[512] = 89;
	indices[513] = 89;
	indices[514] = 83;
	indices[515] = 84;
	indices[516] = 89;
	indices[517] = 84;
	indices[518] = 85;
	indices[519] = 85;
	indices[520] = 84;
	indices[521] = 80;
	indices[522] = 90;
	indices[523] = 85;
	indices[524] = 91;
	indices[525] = 91;
	indices[526] = 85;
	indices[527] = 86;
	indices[528] = 91;
	indices[529] = 86;
	indices[530] = 92;
	indices[531] = 92;
	indices[532] = 86;
	indices[533] = 87;
	indices[534] = 92;
	indices[535] = 87;
	indices[536] = 93;
	indices[537] = 93;
	indices[538] = 87;
	indices[539] = 88;
	indices[540] = 93;
	indices[541] = 88;
	indices[542] = 94;
	indices[543] = 94;
	indices[544] = 88;
	indices[545] = 89;
	indices[546] = 94;
	indices[547] = 89;
	indices[548] = 90;
	indices[549] = 90;
	indices[550] = 89;
	indices[551] = 85;
	indices[552] = 95;
	indices[553] = 90;
	indices[554] = 96;
	indices[555] = 96;
	indices[556] = 90;
	indices[557] = 91;
	indices[558] = 96;
	indices[559] = 91;
	indices[560] = 97;
	indices[561] = 97;
	indices[562] = 91;
	indices[563] = 92;
	indices[564] = 97;
	indices[565] = 92;
	indices[566] = 98;
	indices[567] = 98;
	indices[568] = 92;
	indices[569] = 93;
	indices[570] = 98;
	indices[571] = 93;
	indices[572] = 99;
	indices[573] = 99;
	indices[574] = 93;
	indices[575] = 94;
	indices[576] = 99;
	indices[577] = 94;
	indices[578] = 95;
	indices[579] = 95;
	indices[580] = 94;
	indices[581] = 90;
	indices[582] = 100;
	indices[583] = 95;
	indices[584] = 101;
	indices[585] = 101;
	indices[586] = 95;
	indices[587] = 96;
	indices[588] = 101;
	indices[589] = 96;
	indices[590] = 102;
	indices[591] = 102;
	indices[592] = 96;
	indices[593] = 97;
	indices[594] = 102;
	indices[595] = 97;
	indices[596] = 103;
	indices[597] = 103;
	indices[598] = 97;
	indices[599] = 98;
	indices[600] = 103;
	indices[601] = 98;
	indices[602] = 104;
	indices[603] = 104;
	indices[604] = 98;
	indices[605] = 99;
	indices[606] = 104;
	indices[607] = 99;
	indices[608] = 100;
	indices[609] = 100;
	indices[610] = 99;
	indices[611] = 95;
	indices[612] = 105;
	indices[613] = 100;
	indices[614] = 106;
	indices[615] = 106;
	indices[616] = 100;
	indices[617] = 101;
	indices[618] = 106;
	indices[619] = 101;
	indices[620] = 107;
	indices[621] = 107;
	indices[622] = 101;
	indices[623] = 102;
	indices[624] = 107;
	indices[625] = 102;
	indices[626] = 108;
	indices[627] = 108;
	indices[628] = 102;
	indices[629] = 103;
	indices[630] = 108;
	indices[631] = 103;
	indices[632] = 109;
	indices[633] = 109;
	indices[634] = 103;
	indices[635] = 104;
	indices[636] = 109;
	indices[637] = 104;
	indices[638] = 105;
	indices[639] = 105;
	indices[640] = 104;
	indices[641] = 100;
	indices[642] = 110;
	indices[643] = 105;
	indices[644] = 111;
	indices[645] = 111;
	indices[646] = 105;
	indices[647] = 106;
	indices[648] = 111;
	indices[649] = 106;
	indices[650] = 112;
	indices[651] = 112;
	indices[652] = 106;
	indices[653] = 107;
	indices[654] = 112;
	indices[655] = 107;
	indices[656] = 113;
	indices[657] = 113;
	indices[658] = 107;
	indices[659] = 108;
	indices[660] = 113;
	indices[661] = 108;
	indices[662] = 114;
	indices[663] = 114;
	indices[664] = 108;
	indices[665] = 109;
	indices[666] = 114;
	indices[667] = 109;
	indices[668] = 110;
	indices[669] = 110;
	indices[670] = 109;
	indices[671] = 105;
	indices[672] = 115;
	indices[673] = 110;
	indices[674] = 116;
	indices[675] = 116;
	indices[676] = 110;
	indices[677] = 111;
	indices[678] = 116;
	indices[679] = 111;
	indices[680] = 117;
	indices[681] = 117;
	indices[682] = 111;
	indices[683] = 112;
	indices[684] = 117;
	indices[685] = 112;
	indices[686] = 118;
	indices[687] = 118;
	indices[688] = 112;
	indices[689] = 113;
	indices[690] = 118;
	indices[691] = 113;
	indices[692] = 119;
	indices[693] = 119;
	indices[694] = 113;
	indices[695] = 114;
	indices[696] = 119;
	indices[697] = 114;
	indices[698] = 115;
	indices[699] = 115;
	indices[700] = 114;
	indices[701] = 110;
	indices[702] = 120;
	indices[703] = 120;
	indices[704] = 121;
	indices[705] = 120;
	indices[706] = 121;
	indices[707] = 122;
	indices[708] = 120;
	indices[709] = 122;
	indices[710] = 123;
	indices[711] = 120;
	indices[712] = 123;
	indices[713] = 124;
	indices[714] = 120;
	indices[715] = 115;
	indices[716] = 121;
	indices[717] = 121;
	indices[718] = 115;
	indices[719] = 116;
	indices[720] = 121;
	indices[721] = 116;
	indices[722] = 122;
	indices[723] = 122;
	indices[724] = 116;
	indices[725] = 117;
	indices[726] = 122;
	indices[727] = 117;
	indices[728] = 123;
	indices[729] = 123;
	indices[730] = 117;
	indices[731] = 118;
	indices[732] = 123;
	indices[733] = 118;
	indices[734] = 124;
	indices[735] = 124;
	indices[736] = 118;
	indices[737] = 119;
	indices[738] = 124;
	indices[739] = 119;
	indices[740] = 120;
	indices[741] = 120;
	indices[742] = 119;
	indices[743] = 115;
	//VerticesFloats
	int numVectors = 125;
	delete[] Vertices;
	Vertices = new float[numVectors*3];
	Vertices[0] = 0.0000;
	Vertices[1] = 0.0000;
	Vertices[2] = 0.0000;
	Vertices[3] = 1.1411;
	Vertices[4] = 0.0000;
	Vertices[5] = 0.0000;
	Vertices[6] = 0.5706;
	Vertices[7] = 1.1446;
	Vertices[8] = 0.0000;
	Vertices[9] = -0.5706;
	Vertices[10] = 1.1446;
	Vertices[11] = 0.0000;
	Vertices[12] = -1.1411;
	Vertices[13] = 0.0000;
	Vertices[14] = 0.0000;
	Vertices[15] = 0.0000;
	Vertices[16] = 0.0000;
	Vertices[17] = 0.0000;
	Vertices[18] = 1.1411;
	Vertices[19] = 0.0000;
	Vertices[20] = 0.0000;
	Vertices[21] = 0.5706;
	Vertices[22] = 1.1056;
	Vertices[23] = 0.4973;
	Vertices[24] = -0.5706;
	Vertices[25] = 1.1056;
	Vertices[26] = 0.4973;
	Vertices[27] = -1.1411;
	Vertices[28] = 0.0000;
	Vertices[29] = 0.0000;
	Vertices[30] = 0.0000;
	Vertices[31] = 0.0000;
	Vertices[32] = 0.0000;
	Vertices[33] = 1.1411;
	Vertices[34] = 0.0000;
	Vertices[35] = 0.0000;
	Vertices[36] = 0.5706;
	Vertices[37] = 0.9913;
	Vertices[38] = 0.9607;
	Vertices[39] = -0.5706;
	Vertices[40] = 0.9913;
	Vertices[41] = 0.9607;
	Vertices[42] = -1.1411;
	Vertices[43] = 0.0000;
	Vertices[44] = 0.0000;
	Vertices[45] = 0.0000;
	Vertices[46] = 0.0000;
	Vertices[47] = 0.0000;
	Vertices[48] = 1.1411;
	Vertices[49] = 0.0000;
	Vertices[50] = 0.0000;
	Vertices[51] = 0.5706;
	Vertices[52] = 0.8094;
	Vertices[53] = 1.3586;
	Vertices[54] = -0.5706;
	Vertices[55] = 0.8094;
	Vertices[56] = 1.3586;
	Vertices[57] = -1.1411;
	Vertices[58] = 0.0000;
	Vertices[59] = 0.0000;
	Vertices[60] = 0.0000;
	Vertices[61] = 0.0000;
	Vertices[62] = 0.0000;
	Vertices[63] = 1.1411;
	Vertices[64] = 0.0000;
	Vertices[65] = 0.0000;
	Vertices[66] = 0.5706;
	Vertices[67] = 0.5723;
	Vertices[68] = 1.6640;
	Vertices[69] = -0.5706;
	Vertices[70] = 0.5723;
	Vertices[71] = 1.6640;
	Vertices[72] = -1.1411;
	Vertices[73] = 0.0000;
	Vertices[74] = 0.0000;
	Vertices[75] = 0.0000;
	Vertices[76] = 0.0000;
	Vertices[77] = 0.0000;
	Vertices[78] = 1.1411;
	Vertices[79] = 0.0000;
	Vertices[80] = 0.0000;
	Vertices[81] = 0.5706;
	Vertices[82] = 0.2963;
	Vertices[83] = 1.8559;
	Vertices[84] = -0.5706;
	Vertices[85] = 0.2963;
	Vertices[86] = 1.8559;
	Vertices[87] = -1.1411;
	Vertices[88] = 0.0000;
	Vertices[89] = 0.0000;
	Vertices[90] = 0.0000;
	Vertices[91] = 0.0000;
	Vertices[92] = 0.0000;
	Vertices[93] = 1.1411;
	Vertices[94] = 0.0000;
	Vertices[95] = 0.0000;
	Vertices[96] = 0.5706;
	Vertices[97] = 0.0000;
	Vertices[98] = 1.9214;
	Vertices[99] = -0.5706;
	Vertices[100] = 0.0000;
	Vertices[101] = 1.9214;
	Vertices[102] = -1.1411;
	Vertices[103] = 0.0000;
	Vertices[104] = 0.0000;
	Vertices[105] = 0.0000;
	Vertices[106] = 0.0000;
	Vertices[107] = 0.0000;
	Vertices[108] = 1.1411;
	Vertices[109] = 0.0000;
	Vertices[110] = 0.0000;
	Vertices[111] = 0.5706;
	Vertices[112] = -0.2963;
	Vertices[113] = 1.8559;
	Vertices[114] = -0.5706;
	Vertices[115] = -0.2963;
	Vertices[116] = 1.8559;
	Vertices[117] = -1.1411;
	Vertices[118] = 0.0000;
	Vertices[119] = 0.0000;
	Vertices[120] = 0.0000;
	Vertices[121] = 0.0000;
	Vertices[122] = 0.0000;
	Vertices[123] = 1.1411;
	Vertices[124] = 0.0000;
	Vertices[125] = 0.0000;
	Vertices[126] = 0.5706;
	Vertices[127] = -0.5723;
	Vertices[128] = 1.6640;
	Vertices[129] = -0.5706;
	Vertices[130] = -0.5723;
	Vertices[131] = 1.6640;
	Vertices[132] = -1.1411;
	Vertices[133] = 0.0000;
	Vertices[134] = 0.0000;
	Vertices[135] = 0.0000;
	Vertices[136] = 0.0000;
	Vertices[137] = 0.0000;
	Vertices[138] = 1.1411;
	Vertices[139] = 0.0000;
	Vertices[140] = 0.0000;
	Vertices[141] = 0.5706;
	Vertices[142] = -0.8094;
	Vertices[143] = 1.3586;
	Vertices[144] = -0.5706;
	Vertices[145] = -0.8094;
	Vertices[146] = 1.3586;
	Vertices[147] = -1.1411;
	Vertices[148] = 0.0000;
	Vertices[149] = 0.0000;
	Vertices[150] = 0.0000;
	Vertices[151] = 0.0000;
	Vertices[152] = 0.0000;
	Vertices[153] = 1.1411;
	Vertices[154] = 0.0000;
	Vertices[155] = 0.0000;
	Vertices[156] = 0.5706;
	Vertices[157] = -0.9913;
	Vertices[158] = 0.9607;
	Vertices[159] = -0.5706;
	Vertices[160] = -0.9913;
	Vertices[161] = 0.9607;
	Vertices[162] = -1.1411;
	Vertices[163] = 0.0000;
	Vertices[164] = 0.0000;
	Vertices[165] = 0.0000;
	Vertices[166] = 0.0000;
	Vertices[167] = 0.0000;
	Vertices[168] = 1.1411;
	Vertices[169] = 0.0000;
	Vertices[170] = 0.0000;
	Vertices[171] = 0.5706;
	Vertices[172] = -1.1056;
	Vertices[173] = 0.4973;
	Vertices[174] = -0.5706;
	Vertices[175] = -1.1056;
	Vertices[176] = 0.4973;
	Vertices[177] = -1.1411;
	Vertices[178] = 0.0000;
	Vertices[179] = 0.0000;
	Vertices[180] = 0.0000;
	Vertices[181] = 0.0000;
	Vertices[182] = 0.0000;
	Vertices[183] = 1.1411;
	Vertices[184] = 0.0000;
	Vertices[185] = 0.0000;
	Vertices[186] = 0.5706;
	Vertices[187] = -1.1446;
	Vertices[188] = 0.0000;
	Vertices[189] = -0.5706;
	Vertices[190] = -1.1446;
	Vertices[191] = 0.0000;
	Vertices[192] = -1.1411;
	Vertices[193] = 0.0000;
	Vertices[194] = 0.0000;
	Vertices[195] = 0.0000;
	Vertices[196] = 0.0000;
	Vertices[197] = 0.0000;
	Vertices[198] = 1.1411;
	Vertices[199] = 0.0000;
	Vertices[200] = 0.0000;
	Vertices[201] = 0.5706;
	Vertices[202] = -1.1056;
	Vertices[203] = -0.4973;
	Vertices[204] = -0.5706;
	Vertices[205] = -1.1056;
	Vertices[206] = -0.4973;
	Vertices[207] = -1.1411;
	Vertices[208] = 0.0000;
	Vertices[209] = 0.0000;
	Vertices[210] = 0.0000;
	Vertices[211] = 0.0000;
	Vertices[212] = 0.0000;
	Vertices[213] = 1.1411;
	Vertices[214] = 0.0000;
	Vertices[215] = 0.0000;
	Vertices[216] = 0.5706;
	Vertices[217] = -0.9913;
	Vertices[218] = -0.9607;
	Vertices[219] = -0.5706;
	Vertices[220] = -0.9913;
	Vertices[221] = -0.9607;
	Vertices[222] = -1.1411;
	Vertices[223] = 0.0000;
	Vertices[224] = 0.0000;
	Vertices[225] = 0.0000;
	Vertices[226] = 0.0000;
	Vertices[227] = 0.0000;
	Vertices[228] = 1.1411;
	Vertices[229] = 0.0000;
	Vertices[230] = 0.0000;
	Vertices[231] = 0.5706;
	Vertices[232] = -0.8094;
	Vertices[233] = -1.3586;
	Vertices[234] = -0.5706;
	Vertices[235] = -0.8094;
	Vertices[236] = -1.3586;
	Vertices[237] = -1.1411;
	Vertices[238] = 0.0000;
	Vertices[239] = 0.0000;
	Vertices[240] = 0.0000;
	Vertices[241] = 0.0000;
	Vertices[242] = 0.0000;
	Vertices[243] = 1.1411;
	Vertices[244] = 0.0000;
	Vertices[245] = 0.0000;
	Vertices[246] = 0.5706;
	Vertices[247] = -0.5723;
	Vertices[248] = -1.6640;
	Vertices[249] = -0.5706;
	Vertices[250] = -0.5723;
	Vertices[251] = -1.6640;
	Vertices[252] = -1.1411;
	Vertices[253] = 0.0000;
	Vertices[254] = 0.0000;
	Vertices[255] = 0.0000;
	Vertices[256] = 0.0000;
	Vertices[257] = 0.0000;
	Vertices[258] = 1.1411;
	Vertices[259] = 0.0000;
	Vertices[260] = 0.0000;
	Vertices[261] = 0.5706;
	Vertices[262] = -0.2963;
	Vertices[263] = -1.8559;
	Vertices[264] = -0.5706;
	Vertices[265] = -0.2963;
	Vertices[266] = -1.8559;
	Vertices[267] = -1.1411;
	Vertices[268] = 0.0000;
	Vertices[269] = 0.0000;
	Vertices[270] = 0.0000;
	Vertices[271] = 0.0000;
	Vertices[272] = 0.0000;
	Vertices[273] = 1.1411;
	Vertices[274] = 0.0000;
	Vertices[275] = 0.0000;
	Vertices[276] = 0.5706;
	Vertices[277] = 0.0000;
	Vertices[278] = -1.9214;
	Vertices[279] = -0.5706;
	Vertices[280] = 0.0000;
	Vertices[281] = -1.9214;
	Vertices[282] = -1.1411;
	Vertices[283] = 0.0000;
	Vertices[284] = 0.0000;
	Vertices[285] = 0.0000;
	Vertices[286] = 0.0000;
	Vertices[287] = 0.0000;
	Vertices[288] = 1.1411;
	Vertices[289] = 0.0000;
	Vertices[290] = 0.0000;
	Vertices[291] = 0.5706;
	Vertices[292] = 0.2963;
	Vertices[293] = -1.8559;
	Vertices[294] = -0.5706;
	Vertices[295] = 0.2963;
	Vertices[296] = -1.8559;
	Vertices[297] = -1.1411;
	Vertices[298] = 0.0000;
	Vertices[299] = 0.0000;
	Vertices[300] = 0.0000;
	Vertices[301] = 0.0000;
	Vertices[302] = 0.0000;
	Vertices[303] = 1.1411;
	Vertices[304] = 0.0000;
	Vertices[305] = 0.0000;
	Vertices[306] = 0.5706;
	Vertices[307] = 0.5723;
	Vertices[308] = -1.6640;
	Vertices[309] = -0.5706;
	Vertices[310] = 0.5723;
	Vertices[311] = -1.6640;
	Vertices[312] = -1.1411;
	Vertices[313] = 0.0000;
	Vertices[314] = 0.0000;
	Vertices[315] = 0.0000;
	Vertices[316] = 0.0000;
	Vertices[317] = 0.0000;
	Vertices[318] = 1.1411;
	Vertices[319] = 0.0000;
	Vertices[320] = 0.0000;
	Vertices[321] = 0.5706;
	Vertices[322] = 0.8094;
	Vertices[323] = -1.3586;
	Vertices[324] = -0.5706;
	Vertices[325] = 0.8094;
	Vertices[326] = -1.3586;
	Vertices[327] = -1.1411;
	Vertices[328] = 0.0000;
	Vertices[329] = 0.0000;
	Vertices[330] = 0.0000;
	Vertices[331] = 0.0000;
	Vertices[332] = 0.0000;
	Vertices[333] = 1.1411;
	Vertices[334] = 0.0000;
	Vertices[335] = 0.0000;
	Vertices[336] = 0.5706;
	Vertices[337] = 0.9913;
	Vertices[338] = -0.9607;
	Vertices[339] = -0.5706;
	Vertices[340] = 0.9913;
	Vertices[341] = -0.9607;
	Vertices[342] = -1.1411;
	Vertices[343] = 0.0000;
	Vertices[344] = 0.0000;
	Vertices[345] = 0.0000;
	Vertices[346] = 0.0000;
	Vertices[347] = 0.0000;
	Vertices[348] = 1.1411;
	Vertices[349] = 0.0000;
	Vertices[350] = 0.0000;
	Vertices[351] = 0.5706;
	Vertices[352] = 1.1056;
	Vertices[353] = -0.4973;
	Vertices[354] = -0.5706;
	Vertices[355] = 1.1056;
	Vertices[356] = -0.4973;
	Vertices[357] = -1.1411;
	Vertices[358] = 0.0000;
	Vertices[359] = 0.0000;
	Vertices[360] = 0.0000;
	Vertices[361] = 0.0000;
	Vertices[362] = 0.0000;
	Vertices[363] = 1.1411;
	Vertices[364] = 0.0000;
	Vertices[365] = 0.0000;
	Vertices[366] = 0.5706;
	Vertices[367] = 1.1446;
	Vertices[368] = 0.0000;
	Vertices[369] = -0.5706;
	Vertices[370] = 1.1446;
	Vertices[371] = 0.0000;
	Vertices[372] = -1.1411;
	Vertices[373] = 0.0000;
	Vertices[374] = 0.0000;
}