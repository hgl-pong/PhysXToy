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
#include "Physics/PhysicsCommon.h"
#include "Physics/PhysicsObject.h"
static IPhysicsMaterial *gMaterial = nullptr;
static IPhysicsScene* gScene = nullptr;
static MathLib::HReal stackZ = 10.0f;

static IPhysicsObject *createDynamic(const MathLib::HTransform3 &t, IColliderGeometry &geometry, const MathLib::HVector3 &velocity = MathLib::HVector3(0,0,0))
{
	PhysicsObjectCreateOptions createOptions{};
	createOptions.m_ObjectType=PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
	createOptions.m_Transform = t;
	IPhysicsObject*physicsObject = PhysicsEngineUtils::CreateObject(createOptions);
	PhysicsRigidDynamic* rigidDynamic= dynamic_cast<PhysicsRigidDynamic*>(physicsObject);
	physicsObject->AddColliderGeometry(&geometry, MathLib::HTransform3::Identity());
	rigidDynamic->SetAngularDamping(0.5);
	rigidDynamic->SetLinearVelocity(velocity);	
	if (gScene)
		gScene->AddPhysicsObject(physicsObject);
	return physicsObject;
}

unsigned RandomUInt(unsigned range)
{
	return rand() % range;
}

static void createStack(const MathLib::HTransform3 &t, uint32_t size, MathLib::HReal halfExtent)
{
	CollisionGeometryCreateOptions options;
	options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
	options.m_SphereParams.m_Radius = halfExtent;
	IColliderGeometry* geo = PhysicsEngineUtils::CreateColliderGeometry(options);
	for (uint32_t i = 0; i < size; i++)
	{
		for (uint32_t j = 0; j < size - i; j++)
		{
			MathLib::HTransform3 localTm(Eigen::Translation3f(MathLib::HVector3(MathLib::HReal(j * 2) - MathLib::HReal(size - i), MathLib::HReal(i * 2 + 1), 0) * halfExtent));

			PhysicsObjectCreateOptions objectOptions;
			objectOptions.m_ObjectType=PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
			objectOptions.m_Transform = t*localTm;

			if (RandomUInt(100) > 70)
			{
				objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
			}
			else
			{
				objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
				
			}				
			IPhysicsObject* physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
			physicsObject->AddColliderGeometry(geo,MathLib::HTransform3::Identity());
			gScene->AddPhysicsObject(physicsObject);
		}
	}
}

void initPhysics(bool interactive)
{
	PhysicsEngineOptions options;
	IPhysicsEngine* engine = PhysicsEngineUtils::CreatePhysicsEngine();
	engine->Init(options);

	PhysicsSceneCreateOptions sceneOptions;
	sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
	sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);

	gScene = PhysicsEngineUtils::CreateScene(sceneOptions);

	PhysicsMaterialCreateOptions materialOptions;
	materialOptions.m_StaticFriction = 0.5f;
	materialOptions.m_DynamicFriction = 0.5f;
	materialOptions.m_Restitution = 0.6f;
	materialOptions.m_Density = 10.0f;
	gMaterial = PhysicsEngineUtils::CreateMaterial(materialOptions);

	CollisionGeometryCreateOptions groundPlaneOptions;
	groundPlaneOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE;
	groundPlaneOptions.m_PlaneParams.m_Normal = MathLib::HVector3(0, 1, 0);
	groundPlaneOptions.m_PlaneParams.m_Distance = 0.0f;
	IColliderGeometry* groundPlane = PhysicsEngineUtils::CreateColliderGeometry(groundPlaneOptions);
	PhysicsObjectCreateOptions groundPlaneObjectOptions;
	groundPlaneObjectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
	groundPlaneObjectOptions.m_Transform = MathLib::HTransform3::Identity();
	IPhysicsObject* groundPlaneObject = PhysicsEngineUtils::CreateObject(groundPlaneObjectOptions);
	groundPlaneObject->AddColliderGeometry(groundPlane, MathLib::HTransform3::Identity());
	if (gScene)
		gScene->AddPhysicsObject(groundPlaneObject);


	for (uint32_t i = 0; i < 5; i++)
		createStack(MathLib::HTransform3(Eigen::Translation3f(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);

	if (!interactive)
	{
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = 10.0f;
		options.m_Scale = MathLib::HVector3(1.0f, 1.0f, 1.0f);

		IColliderGeometry* geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

		MathLib::HVector3 translation(0, 40, 100);
		MathLib::HTransform3 transform = Eigen::Affine3f::Identity();
		transform.translate(translation);
		createDynamic(transform, *geometry, MathLib::HVector3(0, -50, -100));
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

void keyPress(unsigned char key,const MathLib::HTransform3& camera)
{
	switch (toupper(key))
	{
	case 'B':
		for (uint32_t i = 0; i < 15; i++)
			createStack(MathLib::HTransform3(Eigen::Translation3f(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		break;
	case ' ':
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = 3.0f;

		IColliderGeometry* geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

		createDynamic(camera, *geometry, camera.rotation() * MathLib::HVector3(0, 0, -1) * 200);
		break;
	}
}
