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
#include "Physics/PhysicsCommon.h"
#include "Physics/PhysicsEngine.h"
#include "Physics/PhysXUtils.h"
#include "Physics/PhysicsObject.h"

static IPhysicsMaterial *gMaterial = nullptr;
static IPhysicsScene* gScene = nullptr;
static PxReal stackZ = 10.0f;

static IPhysicsObject *createDynamic(const PxTransform &t, const IColliderGeometry &geometry, const PxVec3 &velocity = PxVec3(0))
{
	PhysicsObjectCreateOptions createOptions{};
	createOptions.m_ObjectType=PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
	createOptions.m_Transform = ConvertUtils::FromPxTransform(t);
	IPhysicsObject*physicsObject = gPhysicsEngine->CreateObject(createOptions);
	PhysicsRigidDynamic* rigidDynamic= dynamic_cast<PhysicsRigidDynamic*>(physicsObject);
	bool bIsKinematic = rigidDynamic->IsKinematic();
	rigidDynamic->SetAngularDamping(0.5);
	rigidDynamic->SetLinearVelocity(ConvertUtils::FromPxVec3(velocity));
	if (gScene)
		gScene->AddPhysicsObject(physicsObject);
	return physicsObject;
}

unsigned RandomUInt(unsigned range)
{
	return rand() % range;
}

static void createStack(const PxTransform &t, PxU32 size, PxReal halfExtent)
{
	CollisionGeometryCreateOptions options;
	options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
	options.m_SphereParams.m_Radius = halfExtent;
	IColliderGeometry* geo = gPhysicsEngine->CreateColliderGeometry(options);
	for (PxU32 i = 0; i < size; i++)
	{
		for (PxU32 j = 0; j < size - i; j++)
		{
			PxTransform localTm(PxVec3(PxReal(j * 2) - PxReal(size - i), PxReal(i * 2 + 1), 0) * halfExtent);

			PhysicsObjectCreateOptions objectOptions;
			objectOptions.m_ObjectType=PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
			objectOptions.m_Transform = ConvertUtils::FromPxTransform(t.transform(localTm));
			objectOptions.m_MaterialOptions.m_StaticFriction = 0.5f;
			objectOptions.m_MaterialOptions.m_DynamicFriction = 0.5f;
			objectOptions.m_MaterialOptions.m_Restitution = 0.6f;
			objectOptions.m_MaterialOptions.m_Density = 10.0f;

			if (RandomUInt(100) > 70)
			{
				objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
			}
			else
			{
				objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
				
			}				
			IPhysicsObject* physicsObject = gPhysicsEngine->CreateObject(objectOptions);
			gScene->AddPhysicsObject(physicsObject);
		}
	}
}

void initPhysics(bool interactive)
{
	PhysicsEngineOptions options;
	new PhysicsEngine();
	gPhysicsEngine->Init(options);

	PhysicsSceneCreateOptions sceneOptions;
	sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
	sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);

	gScene = gPhysicsEngine->CreateScene(sceneOptions);

	PhysicsMaterialCreateOptions materialOptions;
	materialOptions.m_StaticFriction = 0.5f;
	materialOptions.m_DynamicFriction = 0.5f;
	materialOptions.m_Restitution = 0.6f;
	materialOptions.m_Density = 10.0f;
	gMaterial = gPhysicsEngine->CreateMaterial(materialOptions);

	PxRigidStatic* groundPlane = PxCreatePlane(*GetPxPhysics(), PxPlane(0, 1, 0, 0), *reinterpret_cast<PxMaterial*>(reinterpret_cast<char*>(gMaterial) + gMaterial->GetOffset()));
	physx::PxScene* pxScene = reinterpret_cast<physx::PxScene*>(reinterpret_cast<char*>(gScene) + gScene->GetOffset());
	pxScene->addActor(*groundPlane);

	for (PxU32 i = 0; i < 5; i++)
		createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);

	if (!interactive)
	{
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = 10.0f;
		options.m_Scale = MathLib::HVector3(1.0f, 1.0f, 1.0f);

		IColliderGeometry* geometry = gPhysicsEngine->CreateColliderGeometry(options);
		createDynamic(PxTransform(PxVec3(0, 40, 100)), *geometry, PxVec3(0, -50, -100));
	}
}

void stepPhysics(bool /*interactive*/)
{
	gScene->Tick(1.f/60.f);
}

void cleanupPhysics(bool /*interactive*/)
{
	printf("SnippetHelloWorld done.\n");
}

void keyPress(unsigned char key, const MathLib::HTransform3& camera0)
{
	PxTransform camera =ConvertUtils::ToPxTransform(camera0);
	switch (toupper(key))
	{
	case 'B':
		for (PxU32 i = 0; i < 15; i++)
			createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);
		break;
	case ' ':
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = 3.0f;
		options.m_Scale = MathLib::HVector3(1.0f, 1.0f, 1.0f);

		IColliderGeometry* geometry =gPhysicsEngine->CreateColliderGeometry(options);
		createDynamic(camera, *geometry, camera.rotate(PxVec3(0, 0, -1)) * 200);
		break;
	}
}
