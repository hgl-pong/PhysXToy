#pragma once
#include "Physics/PhysicsCommon.h"
#include "TestMeshGenerator.h"
#include "Physics/TestSoftBody.h"
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
	options.m_EnableCCD = true;
	options.m_bEnablePVD = true;
	options.m_EnableDebugVisualization = true;
	IPhysicsEngine *engine = PhysicsEngineUtils::CreatePhysicsEngine(options);

	PhysicsSceneCreateOptions sceneOptions;
	sceneOptions.m_FilterShaderType = PhysicsSceneFilterShaderType::eDEFAULT;
	sceneOptions.m_Gravity = MathLib::HVector3(0.0f, -9.81f, 0.0f);
	
	gScene = PhysicsEngineUtils::CreateScene(sceneOptions);
	if(engine)
		engine->SetActiveScene(gScene);
	
	PhysicsMaterialCreateOptions materialOptions;
	materialOptions.m_StaticFriction = 0.5f;
	materialOptions.m_DynamicFriction = 0.5f;
	materialOptions.m_Restitution = 0.5f;
	gMaterial = PhysicsEngineUtils::CreateMaterial(materialOptions);
	{
		PhysicsObjectCreateOptions options;
		options.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
		options.m_Transform = MathLib::HTransform3::Identity();
		options.m_Transform.translate(MathLib::HVector3(0, -10, 0));
		
		PhysicsPtr<IPhysicsObject> physicsObject = PhysicsEngineUtils::CreateObject(options);
		
		CollisionGeometryCreateOptions boxOptions;
		boxOptions.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
		boxOptions.m_BoxParams.m_HalfExtents = MathLib::HVector3(50, 0.5, 50);
		
		PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(boxOptions);
		physicsObject->AddColliderGeometry(geometry, MathLib::HTransform3::Identity());
		
		gScene->AddPhysicsObject(physicsObject);
	}
	
	{
		static bool testSoftBody = false;
		if (testSoftBody)
		{
			CreateSoftBodyTestScene();
			return;
		}
	}
	
	auto physicsObjects= TestRigidBody::TestRigidBodyCreate();
	
	for (auto& physicsObject : physicsObjects)
	{
		gScene->AddPhysicsObject(physicsObject);
	}
	
	float dist = 20.0f;
	
	CreateJointChain(7, MathLib::HVector3(0, 10, 0), MathLib::HVector3(0.5f, 1.0f, 0.5f));
	// CreateJointTests();
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
