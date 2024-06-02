#pragma once
#include "Physics/PhysicsCommon.h"
#include "TestMeshGenerator.h"
#include <filesystem>
static PhysicsPtr < IPhysicsMaterial>gMaterial;
static PhysicsPtr < IPhysicsScene>gScene;

#include "TestRigidBodyCreate.h"

void initPhysics(bool interactive)
{
	PhysicsEngineOptions options;
	options.m_iNumThreads = 10;
	IPhysicsEngine *engine = PhysicsEngineUtils::CreatePhysicsEngine(options);

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
	PhysicsPtr<IColliderGeometry> groundPlane = PhysicsEngineUtils::CreateColliderGeometry(groundPlaneOptions);

	PhysicsObjectCreateOptions groundPlaneObjectOptions;
	groundPlaneObjectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
	groundPlaneObjectOptions.m_Transform = MathLib::HTransform3::Identity();
	PhysicsPtr < IPhysicsObject> groundPlaneObject = PhysicsEngineUtils::CreateObject(groundPlaneObjectOptions);
	groundPlaneObject->AddColliderGeometry(groundPlane, MathLib::HTransform3::Identity());
	if (gScene)
		gScene->AddPhysicsObject(groundPlaneObject);

	TestRigidBody::CreateTestingMeshData();//Bunny
	//TestRigidBody::CreateTestingMeshData("..\\..\\asset\\model\\teapot.obj", 0.2);
	//TestRigidBody::CreateTestingMeshData("..\\..\\asset\\model\\banana.obj", 1);
	//TestRigidBody::CreateTestingMeshData("..\\..\\asset\\model\\armadillo.obj",0.4);
	TestRigidBody::TestRigidBodyCreate();

	if (!interactive)
	{
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = 10.0f;
		options.m_Scale = MathLib::HVector3(1.0f, 1.0f, 1.0f);

		PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

		MathLib::HVector3 translation(0, 40, 100);
		MathLib::HTransform3 transform = MathLib::HTransform3::Identity();
		transform.translate(translation);
		TestRigidBody::CreateDynamic(transform, geometry, MathLib::HVector3(0, -50, -100));
	}
}

void stepPhysics(bool /*interactive*/)
{
	gScene->Tick(1.f / 60.f);
}

void cleanupPhysics(bool /*interactive*/)
{
	PhysicsEngineUtils::DestroyPhysicsEngine();
	printf("SnippetHelloWorld done.\n");
}

void keyPress(unsigned char key, const MathLib::HTransform3 &camera)
{
	switch (toupper(key))
	{
	case 'B':
		for (int i = 0; i < 2; i++)
		{
			TestRigidBody::TestRigidBodyCreate();
		}
		break;
	case ' ':
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = 2.0f;

		PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

		TestRigidBody::CreateDynamic(camera, geometry, camera.rotation() * MathLib::HVector3(0, 0, -1) * 100);
		break;
	}
}
