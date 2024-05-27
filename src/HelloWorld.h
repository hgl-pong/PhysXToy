// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

// ****************************************************************************
// This snippet illustrates simple use of physx
//
// It creates a number of box stacks on a plane, and if rendering, allows the
// user to create new stacks and fire a ball from the camera position
// ****************************************************************************
#pragma once
#include <ctype.h>
#include "PxPhysicsAPI.h"
//#include "snippetutils/SnippetUtils.h"
#include "Physics/PhysicsEngine.h"
#include "Physics/PhysicsScene.h"
#include "Physics/PhysicsMaterial.h"
using namespace physx;

//static PxDefaultAllocator gAllocator;
//static PxDefaultErrorCallback gErrorCallback;
//static PxFoundation *gFoundation = NULL;
//static PxPhysics *gPhysics = NULL;
//static PxDefaultCpuDispatcher *gDispatcher = NULL;
static IPhysicsEngine* gPhysicsEngine = nullptr;
static IPhysicsMaterial *gMaterial = nullptr;

static IPhysicsScene* gScene = nullptr;
static PxPvd *gPvd = NULL;

static PxReal stackZ = 10.0f;

static PxRigidDynamic *createDynamic(const PxTransform &t, const PxGeometry &geometry, const PxVec3 &velocity = PxVec3(0))
{
	PxRigidDynamic *dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

template <PxConvexMeshCookingType::Enum convexMeshCookingType, bool directInsertion, PxU32 gaussMapLimit>
static PxConvexMesh *createConvex(PxU32 numVerts, const PxVec3 *verts)
{
	PxTolerancesScale tolerances;
	PxCookingParams params(tolerances);

	// Use the new (default) PxConvexMeshCookingType::eQUICKHULL
	params.convexMeshCookingType = convexMeshCookingType;

	// If the gaussMapLimit is chosen higher than the number of output vertices, no gauss map is added to the convex mesh data (here 256).
	// If the gaussMapLimit is chosen lower than the number of output vertices, a gauss map is added to the convex mesh data (here 16).
	params.gaussMapLimit = gaussMapLimit;

	// Setup the convex mesh descriptor
	PxConvexMeshDesc desc;

	// We provide points only, therefore the PxConvexFlag::eCOMPUTE_CONVEX flag must be specified
	desc.points.data = verts;
	desc.points.count = numVerts;
	desc.points.stride = sizeof(PxVec3);
	desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

	PxU32 meshSize = 0;
	PxConvexMesh *convex = NULL;

	PxU64 startTime = SnippetUtils::getCurrentTimeCounterValue();

	if (directInsertion)
	{
		// Directly insert mesh into PhysX
		convex = PxCreateConvexMesh(params, desc, gPhysics->getPhysicsInsertionCallback());
		PX_ASSERT(convex);
	}
	else
	{
		// Serialize the cooked mesh into a stream.
		PxDefaultMemoryOutputStream outStream;
		bool res = PxCookConvexMesh(params, desc, outStream);
		PX_UNUSED(res);
		PX_ASSERT(res);
		meshSize = outStream.getSize();

		// Create the mesh from a stream.
		PxDefaultMemoryInputData inStream(outStream.getData(), outStream.getSize());
		convex = gPhysics->createConvexMesh(inStream);
		PX_ASSERT(convex);
	}
	return convex;
}

static void createStack(const PxTransform &t, PxU32 size, PxReal halfExtent)
{
	auto *convex = createConvex<PxConvexMeshCookingType::eQUICKHULL, true, 256>(SnippetUtils::Bunny_getNbVerts(), SnippetUtils::Bunny_getVerts());
	printf("convex vertex count: %d\n", convex->getNbVertices());
	// PxBoxGeometry geo(halfExtent, halfExtent, halfExtent);
	// PxSphereGeometry geo(halfExtent);
	PxConvexMeshGeometry geo(convex, PxMeshScale(3.f));
	PxShape *shape = gPhysics->createShape(geo, *gMaterial);
	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
			PxRigidDynamic *body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
	printf("actor count: %d\n", gScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC));
	// auto *convex = createConvex<PxConvexMeshCookingType::eQUICKHULL, true, 256>(SnippetUtils::Bunny_getNbVerts(), SnippetUtils::Bunny_getVerts());
	// printf("convex vertex count: %d\n", convex->getNbVertices());
	//// PxBoxGeometry geo(halfExtent, halfExtent, halfExtent);
	// PxConvexMeshGeometry geo(convex, PxMeshScale(5.f));
	// auto *body = gPhysics->createRigidDynamic(t);
	// int counter = 0;
	// for (PxU32 i = 0; i < size; i++)
	//{
	//	for (PxU32 j = 0; j < size - i; j++)
	//	{
	//		PxShape *shape = gPhysics->createShape(geo, *gMaterial);
	//		PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);
	//		shape->setLocalPose(localTm);
	//		body->attachShape(*shape);
	//		shape->release();
	//		counter++;
	//	}
	// }
	// printf("total vertex count: %d\n", counter * convex->getNbVertices());
	// PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
	// gScene->addActor(*body);
}

void initPhysics(bool interactive)
{
	PhysicsEngineOptions options;
	gPhysicsEngine =new PhysicsEngine();
	gPhysicsEngine->Init(options);

	PhysicsSceneCreateOptions sceneOptions;
	sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
	sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);

	gScene = gPhysicsEngine->CreateScene(sceneOptions);

	PhysicsMaterialCreateOptions materialOptions;
	materialOptions.m_StaticFriction = 0.5f;
	materialOptions.m_DynamicFriction = 0.5f;
	materialOptions.m_Restitution = 0.6f;
	gMaterial = gPhysicsEngine->CreateMaterial(materialOptions);

	PxRigidStatic *groundPlane = PxCreatePlane(*gPhysics, PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	for (PxU32 i = 0; i < 5; i++)
		createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);

	if (!interactive)
		createDynamic(PxTransform(PxVec3(0, 40, 100)), PxSphereGeometry(10), PxVec3(0, -50, -100));
}

void stepPhysics(bool /*interactive*/)
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

void cleanupPhysics(bool /*interactive*/)
{
	PX_RELEASE(gScene);
	PX_RELEASE(gDispatcher);
	PX_RELEASE(gPhysics);
	if (gPvd)
	{
		PxPvdTransport *transport = gPvd->getTransport();
		gPvd->release();
		gPvd = NULL;
		PX_RELEASE(transport);
	}
	PX_RELEASE(gFoundation);

	printf("SnippetHelloWorld done.\n");
}

void keyPress(unsigned char key, const PxTransform &camera)
{
	switch (toupper(key))
	{
	case 'B':
		for (PxU32 i = 0; i < 15; i++)
			createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);
		break;
	case ' ':
		createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200);
		break;
	}
}

int snippetMain(int, const char *const *)
{
#ifndef RENDER_SNIPPET
	extern void renderLoop();
	renderLoop();
#else
	static const PxU32 frameCount = 100;
	initPhysics(false);
	for (PxU32 i = 0; i < frameCount; i++)
		stepPhysics(false);
	cleanupPhysics(false);
#endif

	return 0;
}