#pragma once
#include "Physics/PhysicsCommon.h"
#include "TestMeshGenerator.h"
#include <filesystem>
static PhysicsPtr < IPhysicsMaterial>gMaterial;
static PhysicsPtr < IPhysicsScene>gScene;

#include "TestRigidBodyCreate.h"

void CreateJointChain(int numLinks, const MathLib::HVector3& startPos, const MathLib::HVector3& linkSize)
{
	if (numLinks <= 0)
		return;
	
	std::vector<PhysicsPtr<IPhysicsObject>> links;
	
	{
		PhysicsObjectCreateOptions anchorOptions;
		anchorOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
		anchorOptions.m_Transform = MathLib::HTransform3::Identity();
		anchorOptions.m_Transform.translate(startPos);
		
		PhysicsPtr<IPhysicsObject> anchor = PhysicsEngineUtils::CreateObject(anchorOptions);
		
		CollisionGeometryCreateOptions boxOptions;
		boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
		boxOptions.m_BoxParams.m_HalfExtents = linkSize * 0.5f;
		
		PhysicsPtr<IColliderGeometry> boxGeom = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);
		anchor->AddColliderGeometry(boxGeom, MathLib::HTransform3::Identity());
		
		if (gScene)
			gScene->AddPhysicsObject(anchor);
		
		links.push_back(anchor);
	}
	
	for (int i = 0; i < numLinks; i++)
	{
		PhysicsObjectCreateOptions linkOptions;
		linkOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
		linkOptions.m_Transform = MathLib::HTransform3::Identity();
		linkOptions.m_Transform.translate(startPos + MathLib::HVector3(0, -linkSize.y * (i + 1) * 2.0f, 0));
		linkOptions.m_Mass = 1.0f;
		
		PhysicsPtr<IPhysicsObject> linkObj = PhysicsEngineUtils::CreateObject(linkOptions);
		
		CollisionGeometryCreateOptions boxOptions;
		boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
		boxOptions.m_BoxParams.m_HalfExtents = linkSize * 0.5f;
		
		PhysicsPtr<IColliderGeometry> boxGeom = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);
		linkObj->AddColliderGeometry(boxGeom, MathLib::HTransform3::Identity());
		
		if (gScene)
			gScene->AddPhysicsObject(linkObj);
		
		links.push_back(linkObj);
		
		if (i > 0 || links.size() > 1)
		{
			JointCreateOptions jointOptions;
			jointOptions.type = JointType::SPHERICAL;
			jointOptions.objectA = links[i];
			jointOptions.objectB = links[i+1];
			jointOptions.collisionEnabled = false;
			
			jointOptions.localFrameA = MathLib::HTransform3::Identity();
			jointOptions.localFrameA.translate(MathLib::HVector3(0, linkSize.y, 0));
			
			jointOptions.localFrameB = MathLib::HTransform3::Identity();
			jointOptions.localFrameB.translate(MathLib::HVector3(0, -linkSize.y, 0));
			
			PhysicsPtr<IPhysicsJoint> joint = PhysicsEngineUtils::CreateJoint(jointOptions);
			
			if (gScene && joint)
				gScene->AddJoint(joint);
		}
	}
}

void CreateJointTests()
{
	{
		MathLib::HVector3 pos(10.0f, 10.0f, 0.0f);
		
		PhysicsObjectCreateOptions staticOptions;
		staticOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
		staticOptions.m_Transform = MathLib::HTransform3::Identity();
		staticOptions.m_Transform.translate(pos);
		
		PhysicsPtr<IPhysicsObject> staticObj = PhysicsEngineUtils::CreateObject(staticOptions);
		
		CollisionGeometryCreateOptions boxOptions;
		boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
		boxOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(1.0f, 1.0f, 1.0f);
		
		PhysicsPtr<IColliderGeometry> boxGeom = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);
		staticObj->AddColliderGeometry(boxGeom, MathLib::HTransform3::Identity());
		
		PhysicsObjectCreateOptions dynamicOptions;
		dynamicOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
		dynamicOptions.m_Transform = MathLib::HTransform3::Identity();
		dynamicOptions.m_Transform.translate(pos + MathLib::HVector3(0.0f, -3.0f, 0.0f));
		dynamicOptions.m_Mass = 1.0f;
		
		PhysicsPtr<IPhysicsObject> dynamicObj = PhysicsEngineUtils::CreateObject(dynamicOptions);
		
		dynamicObj->AddColliderGeometry(boxGeom, MathLib::HTransform3::Identity());
		
		if (gScene)
		{
			gScene->AddPhysicsObject(staticObj);
			gScene->AddPhysicsObject(dynamicObj);
		}
		
		JointCreateOptions jointOptions;
		jointOptions.type = JointType::FIXED;
		jointOptions.objectA = staticObj;
		jointOptions.objectB = dynamicObj;
		jointOptions.collisionEnabled = false;
		
		jointOptions.localFrameA = MathLib::HTransform3::Identity();
		jointOptions.localFrameA.translate(MathLib::HVector3(0.0f, -1.5f, 0.0f));
		
		jointOptions.localFrameB = MathLib::HTransform3::Identity();
		jointOptions.localFrameB.translate(MathLib::HVector3(0.0f, 1.5f, 0.0f));
		
		PhysicsPtr<IPhysicsJoint> joint = PhysicsEngineUtils::CreateJoint(jointOptions);
		
		if (gScene && joint)
			gScene->AddJoint(joint);
	}
	
	{
		MathLib::HVector3 pos(-10.0f, 10.0f, 0.0f);
		
		PhysicsObjectCreateOptions staticOptions;
		staticOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
		staticOptions.m_Transform = MathLib::HTransform3::Identity();
		staticOptions.m_Transform.translate(pos);
		
		PhysicsPtr<IPhysicsObject> staticObj = PhysicsEngineUtils::CreateObject(staticOptions);
		
		CollisionGeometryCreateOptions boxOptions;
		boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
		boxOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(1.0f, 1.0f, 1.0f);
		
		PhysicsPtr<IColliderGeometry> boxGeom = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);
		staticObj->AddColliderGeometry(boxGeom, MathLib::HTransform3::Identity());
		
		PhysicsObjectCreateOptions dynamicOptions;
		dynamicOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
		dynamicOptions.m_Transform = MathLib::HTransform3::Identity();
		dynamicOptions.m_Transform.translate(pos + MathLib::HVector3(0.0f, -5.0f, 0.0f));
		dynamicOptions.m_Mass = 1.0f;
		
		PhysicsPtr<IPhysicsObject> dynamicObj = PhysicsEngineUtils::CreateObject(dynamicOptions);
		
		boxOptions.m_SphereParams.m_Radius = 1.0f;
		boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		PhysicsPtr<IColliderGeometry> sphereGeom = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);
		
		dynamicObj->AddColliderGeometry(sphereGeom, MathLib::HTransform3::Identity());
		
		if (gScene)
		{
			gScene->AddPhysicsObject(staticObj);
			gScene->AddPhysicsObject(dynamicObj);
		}
		
		JointCreateOptions jointOptions;
		jointOptions.type = JointType::DISTANCE;
		jointOptions.objectA = staticObj;
		jointOptions.objectB = dynamicObj;
		jointOptions.collisionEnabled = false;
		
		jointOptions.localFrameA = MathLib::HTransform3::Identity();
		jointOptions.localFrameB = MathLib::HTransform3::Identity();
		
		PhysicsPtr<IPhysicsJoint> joint = PhysicsEngineUtils::CreateJoint(jointOptions);
		
		if (gScene && joint)
			gScene->AddJoint(joint);
	}
	
	{
		MathLib::HVector3 pos(0.0f, 10.0f, 10.0f);
		
		PhysicsObjectCreateOptions staticOptions;
		staticOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
		staticOptions.m_Transform = MathLib::HTransform3::Identity();
		staticOptions.m_Transform.translate(pos);
		
		PhysicsPtr<IPhysicsObject> staticObj = PhysicsEngineUtils::CreateObject(staticOptions);
		
		CollisionGeometryCreateOptions boxOptions;
		boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
		boxOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(0.5f, 0.5f, 0.5f);
		
		PhysicsPtr<IColliderGeometry> boxGeom = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);
		staticObj->AddColliderGeometry(boxGeom, MathLib::HTransform3::Identity());
		
		PhysicsObjectCreateOptions dynamicOptions;
		dynamicOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
		dynamicOptions.m_Transform = MathLib::HTransform3::Identity();
		dynamicOptions.m_Transform.translate(pos + MathLib::HVector3(0.0f, -3.0f, 0.0f));
		dynamicOptions.m_Mass = 1.0f;
		
		PhysicsPtr<IPhysicsObject> dynamicObj = PhysicsEngineUtils::CreateObject(dynamicOptions);
		
		boxOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(0.2f, 3.0f, 0.2f);
		PhysicsPtr<IColliderGeometry> barGeom = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);
		
		dynamicObj->AddColliderGeometry(barGeom, MathLib::HTransform3::Identity());
		
		if (gScene)
		{
			gScene->AddPhysicsObject(staticObj);
			gScene->AddPhysicsObject(dynamicObj);
		}
		
		JointCreateOptions jointOptions;
		jointOptions.type = JointType::REVOLUTE;
		jointOptions.objectA = staticObj;
		jointOptions.objectB = dynamicObj;
		jointOptions.collisionEnabled = false;
		
		jointOptions.localFrameA = MathLib::HTransform3::Identity();
		jointOptions.localFrameA.translate(MathLib::HVector3(0.0f, -0.5f, 0.0f));
		
		jointOptions.localFrameB = MathLib::HTransform3::Identity();
		jointOptions.localFrameB.translate(MathLib::HVector3(0.0f, 2.5f, 0.0f));
		
		PhysicsPtr<IPhysicsJoint> joint = PhysicsEngineUtils::CreateJoint(jointOptions);
		
		if (gScene && joint)
			gScene->AddJoint(joint);
	}
}

void initPhysics(bool interactive)
{
	PhysicsEngineOptions options;
	options.m_NumThreads = 10;
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
	//TestRigidBody::CreateTestingMeshData("..\\..\\asset\\models\\teapot.obj", 0.2);
	//TestRigidBody::CreateTestingMeshData("..\\..\\asset\\models\\banana.obj", 1);
	//TestRigidBody::CreateTestingMeshData("..\\..\\asset\\models\\armadillo.obj",0.4);
	auto physicsObjects= TestRigidBody::TestRigidBodyCreate();
	if (gScene)
	for (auto& physicsObject : physicsObjects)
	{
		gScene->AddPhysicsObject(physicsObject);
	}

	CreateJointTests();
	
	CreateJointChain(5, MathLib::HVector3(0.0f, 20.0f, 0.0f), MathLib::HVector3(0.5f, 0.5f, 0.5f));

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
		auto dynamic = TestRigidBody::CreateDynamic(transform, geometry, MathLib::HVector3(0, -50, -100));
		if (gScene)
			gScene->AddPhysicsObject(dynamic);
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
		auto dynamic = TestRigidBody::CreateDynamic(camera, geometry, camera.rotation() * MathLib::HVector3(0, 0, -1) * 100);
		if(gScene)
		  gScene->AddPhysicsObject(dynamic);
		break;
	}
}
