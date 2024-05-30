#pragma once
#include "Physics/PhysicsCommon.h"
#include "Physics/PhysicsObject.h"
#include "Physics/TestMeshGenerator.h"
#include "Physics/PhysicsConvexUtils.h"
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

	options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
	options.m_BoxParams.m_HalfExtents = MathLib::HVector3(halfExtent, halfExtent, halfExtent);

	std::vector<MathLib::HVector3> triVerts;
	std::vector<uint32_t> triIndices;

	uint32_t numVerts = MeshGenerateUtils::Bunny_getNbVerts();
	uint32_t numFaces = MeshGenerateUtils::Bunny_getNbFaces();

	triVerts.resize(numVerts);
	triIndices.resize(numFaces * 3);

	memcpy(triVerts.data(), MeshGenerateUtils::Bunny_getVerts(), sizeof(MathLib::HVector3) * numVerts);
	memcpy(triIndices.data(), MeshGenerateUtils::Bunny_getFaces(), sizeof(uint32_t) * numFaces * 3);
	PhysicsMeshData meshdata;

	if (false)
	{
		PhysicsConvexUtils::BuildConvexMesh(triVerts, triIndices, meshdata);
	}
	else
	{
		meshdata.m_Vertices = triVerts;
		meshdata.m_Indices = triIndices;
	}

	//options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
	//options.m_CapsuleParams.m_HalfHeight = halfExtent/2;
	//options.m_CapsuleParams.m_Radius = halfExtent/2;

	options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH;
	options.m_ConvexMeshParams.m_Vertices = meshdata.m_Vertices;
	options.m_Scale = MathLib::HVector3(3.0f, 3.0f, 3.0f);

	options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH;
	options.m_TriangleMeshParams.m_Vertices = meshdata.m_Vertices;
	options.m_TriangleMeshParams.m_Indices = meshdata.m_Indices;
	options.m_Scale = MathLib::HVector3(3.0f, 3.0f, 3.0f);

	PhysicsPtr<IColliderGeometry> geo1 = make_physics_ptr<IColliderGeometry>(PhysicsEngineUtils::CreateColliderGeometry(options));

	options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH;
	options.m_ConvexMeshParams.m_Vertices = meshdata.m_Vertices;
	options.m_Scale = MathLib::HVector3(3.0f, 3.0f, 3.0f);
	PhysicsPtr<IColliderGeometry> geo2 = make_physics_ptr<IColliderGeometry>(PhysicsEngineUtils::CreateColliderGeometry(options));

	for (uint32_t i = 0; i < size; i++)
	{
		for (uint32_t j = 0; j < size - i; j++)
		{
			MathLib::HTransform3 localTm(MathLib::HTranslation3(MathLib::HVector3(MathLib::HReal(j * 2) - MathLib::HReal(size - i), MathLib::HReal(i * 2 + 1), 0) * halfExtent));

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
			if (!physicsObject->AddColliderGeometry(geo1.get(), MathLib::HTransform3::Identity()))
			{
				physicsObject->AddColliderGeometry(geo2.get(), MathLib::HTransform3::Identity());
			}
			gScene->AddPhysicsObject(physicsObject);
		}
	}
}

void initPhysics(bool interactive)
{
	PhysicsEngineOptions options;
	options.m_iNumThreads = 10;
	IPhysicsEngine* engine = PhysicsEngineUtils::CreatePhysicsEngine(options);

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
	PhysicsPtr<IColliderGeometry> groundPlane = make_physics_ptr<IColliderGeometry>(PhysicsEngineUtils::CreateColliderGeometry(groundPlaneOptions));

	PhysicsObjectCreateOptions groundPlaneObjectOptions;
	groundPlaneObjectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
	groundPlaneObjectOptions.m_Transform = MathLib::HTransform3::Identity();
	IPhysicsObject* groundPlaneObject = PhysicsEngineUtils::CreateObject(groundPlaneObjectOptions);
	groundPlaneObject->AddColliderGeometry(groundPlane.get(), MathLib::HTransform3::Identity());
	if (gScene)
		gScene->AddPhysicsObject(groundPlaneObject);


	for (uint32_t i = 0; i < 5; i++)
		createStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);

	if (!interactive)
	{
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = 10.0f;
		options.m_Scale = MathLib::HVector3(1.0f, 1.0f, 1.0f);

		PhysicsPtr<IColliderGeometry> geometry = make_physics_ptr<IColliderGeometry>(PhysicsEngineUtils::CreateColliderGeometry(options));

		MathLib::HVector3 translation(0, 40, 100);
		MathLib::HTransform3 transform = MathLib::HTransform3::Identity();
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
			createStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		break;
	case ' ':
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = 2.0f;

		PhysicsPtr<IColliderGeometry> geometry = make_physics_ptr<IColliderGeometry>(PhysicsEngineUtils::CreateColliderGeometry(options));

		createDynamic(camera, *geometry, camera.rotation() * MathLib::HVector3(0, 0, -1) * 200);
		break;
	}
}
