#pragma once
#include "MeshDataLoader.h"
#include "TestMeshGenerator.h"
inline  unsigned RandomUInt(unsigned range)
{
	return rand() % range;
}
namespace TestRigidBody
{
	static MathLib::HReal stackZ = 15.0f;
	static PhysicsMeshData TriangleMeshData;
	static PhysicsMeshData ConvexMeshData;
	static std::vector<PhysicsMeshData> ConvexDecomposedMeshData;

	static void CreateTestingMeshData(const char* path =nullptr,const MathLib::HReal scale =1)
	{
		if (path == nullptr || (!LoadObj(path,TriangleMeshData,scale)))
		{		
			uint32_t numVerts = 0;
			uint32_t numFaces = 0;

			numVerts = MeshGenerateUtils::Bunny_getNbVerts();
			numFaces = MeshGenerateUtils::Bunny_getNbFaces();

			TriangleMeshData.m_Vertices.resize(numVerts);
			TriangleMeshData.m_Indices.resize(numFaces * 3);

			memcpy(TriangleMeshData.m_Vertices.data(), MeshGenerateUtils::Bunny_getVerts(), sizeof(MathLib::HVector3) * numVerts);
			memcpy(TriangleMeshData.m_Indices.data(), MeshGenerateUtils::Bunny_getFaces(), sizeof(uint32_t) * numFaces * 3);
		}
		PhysicsEngineUtils::BuildConvexMesh(TriangleMeshData.m_Vertices, TriangleMeshData.m_Indices, ConvexMeshData);
		ConvexDecomposeOptions decomposeOptions;
		decomposeOptions.m_VoxelGridResolution = 1000;
		decomposeOptions.m_MaximumNumberOfHulls = 16;
		PhysicsEngineUtils::ConvexDecomposition(TriangleMeshData, decomposeOptions, ConvexDecomposedMeshData);
	}

	static PhysicsPtr<IPhysicsObject> CreateDynamic(const MathLib::HTransform3& t, PhysicsPtr < IColliderGeometry>& geometry, const MathLib::HVector3& velocity = MathLib::HVector3(0, 0, 0))
	{
		PhysicsObjectCreateOptions createOptions{};
		createOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
		createOptions.m_Transform = t;
		PhysicsPtr< IPhysicsObject> physicsObject = PhysicsEngineUtils::CreateObject(createOptions);
		IDynamicObject* rigidDynamic = dynamic_cast<IDynamicObject*>(physicsObject.get());
		physicsObject->AddColliderGeometry(geometry, MathLib::HTransform3::Identity());
		rigidDynamic->SetAngularDamping(0.5);
		rigidDynamic->SetLinearVelocity(velocity);
		return physicsObject;
	}

	static void RandomRigidBodyType(PhysicsObjectType& objectType)
	{
		if (RandomUInt(100) > 70)
		{
			objectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
		}
		else
		{
			objectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
		}
	}

	static std::vector<PhysicsPtr<IPhysicsObject>> CreateDeComposeConvexStack(const MathLib::HTransform3& t, uint32_t size, MathLib::HReal halfExtent)
	{
		std::vector<PhysicsPtr<IPhysicsObject>> objects;
		std::vector<PhysicsPtr<IColliderGeometry>> geos(ConvexDecomposedMeshData.size());
		for (size_t i = 0; i < ConvexDecomposedMeshData.size(); i++)
		{
			CollisionGeometryCreateOptions options;
			options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH;
			options.m_ConvexMeshParams.m_Vertices = ConvexDecomposedMeshData[i].m_Vertices;
			options.m_Scale = MathLib::HVector3(3.0f, 3.0f, 3.0f);
			geos[i] = PhysicsEngineUtils::CreateColliderGeometry(options);
		}

		for (uint32_t i = 0; i < size; i++)
		{
			for (uint32_t j = 0; j < size - i; j++)
			{
				MathLib::HTransform3 localTm(MathLib::HTranslation3(MathLib::HVector3(MathLib::HReal(j * 2) - MathLib::HReal(size - i), MathLib::HReal(i * 2 + 1), 0) * halfExtent));

				PhysicsObjectCreateOptions objectOptions;
				objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
				objectOptions.m_Transform = t * localTm;
				RandomRigidBodyType(objectOptions.m_ObjectType);
				PhysicsPtr < IPhysicsObject> physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
				for (size_t i = 0; i < geos.size(); i++)
				{
					if (!physicsObject->AddColliderGeometry(geos[i], MathLib::HTransform3::Identity()))
					{
						physicsObject->AddColliderGeometry(geos[0], MathLib::HTransform3::Identity());
					}
				}
				objects.push_back(physicsObject);
			}
		}
		return objects;
	}

	static std::vector<PhysicsPtr<IPhysicsObject>> CreateBoxStack(const MathLib::HTransform3& t, uint32_t size, MathLib::HReal halfExtent)
	{
		std::vector<PhysicsPtr<IPhysicsObject>> objects;
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
		options.m_BoxParams.m_HalfExtents = MathLib::HVector3(halfExtent, halfExtent, halfExtent);

		PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

		for (uint32_t i = 0; i < size; i++)
		{
			for (uint32_t j = 0; j < size - i; j++)
			{
				MathLib::HTransform3 localTm(MathLib::HTranslation3(MathLib::HVector3(MathLib::HReal(j * 2) - MathLib::HReal(size - i), MathLib::HReal(i * 2 + 1), 0) * halfExtent));

				PhysicsObjectCreateOptions objectOptions;
				objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
				objectOptions.m_Transform = t * localTm;

				if (RandomUInt(100) > 70)
				{
					objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
				}
				else
				{
					objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
				}
				PhysicsPtr<IPhysicsObject> physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
				physicsObject->AddColliderGeometry(geometry, MathLib::HTransform3::Identity());
				objects.push_back(physicsObject);
			}
		}
		return objects;
	}

	static std::vector<PhysicsPtr<IPhysicsObject>> CreateSphereStack(const MathLib::HTransform3& t, uint32_t size, MathLib::HReal halfExtent)
	{
		std::vector<PhysicsPtr<IPhysicsObject>> objects;
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = halfExtent;

		PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

		for (uint32_t i = 0; i < size; i++)
		{
			for (uint32_t j = 0; j < size - i; j++)
			{
				MathLib::HTransform3 localTm(MathLib::HTranslation3(MathLib::HVector3(MathLib::HReal(j * 2) - MathLib::HReal(size - i), MathLib::HReal(i * 2 + 1), 0) * halfExtent));

				PhysicsObjectCreateOptions objectOptions;
				objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
				objectOptions.m_Transform = t * localTm;
				RandomRigidBodyType(objectOptions.m_ObjectType);
				PhysicsPtr < IPhysicsObject> physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
				physicsObject->AddColliderGeometry(geometry, MathLib::HTransform3::Identity());
				objects.push_back(physicsObject);
			}
		}
		return objects;
	}

	static std::vector<PhysicsPtr<IPhysicsObject>> CreateCapsuleStack(const MathLib::HTransform3& t, uint32_t size, MathLib::HReal halfExtent)
	{
		std::vector<PhysicsPtr<IPhysicsObject>> objects;
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
		options.m_CapsuleParams.m_HalfHeight = halfExtent / 2;
		options.m_CapsuleParams.m_Radius = halfExtent / 2;

		PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

		for (uint32_t i = 0; i < size; i++)
		{
			for (uint32_t j = 0; j < size - i; j++)
			{
				MathLib::HTransform3 localTm(MathLib::HTranslation3(MathLib::HVector3(MathLib::HReal(j * 2) - MathLib::HReal(size - i), MathLib::HReal(i * 2 + 1), 0) * halfExtent));

				PhysicsObjectCreateOptions objectOptions;
				objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
				objectOptions.m_Transform = t * localTm;
				RandomRigidBodyType(objectOptions.m_ObjectType);
				PhysicsPtr < IPhysicsObject> physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
				physicsObject->AddColliderGeometry(geometry, MathLib::HTransform3::Identity());
				objects.push_back(physicsObject);
			}
		}
		return objects;
	}

	static std::vector<PhysicsPtr<IPhysicsObject>> CreateTriangleMeshStack(const MathLib::HTransform3& t, uint32_t size, MathLib::HReal halfExtent)
	{
		std::vector<PhysicsPtr<IPhysicsObject>> objects;
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH;
		options.m_TriangleMeshParams.m_Vertices = TriangleMeshData.m_Vertices;
		options.m_TriangleMeshParams.m_Indices = TriangleMeshData.m_Indices;
		options.m_Scale = MathLib::HVector3(3.0f, 3.0f, 3.0f);

		PhysicsPtr<IColliderGeometry> geometry0 = PhysicsEngineUtils::CreateColliderGeometry(options);
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH;
		options.m_ConvexMeshParams.m_Vertices = ConvexMeshData.m_Vertices;
		options.m_Scale = MathLib::HVector3(3.0f, 3.0f, 3.0f);
		PhysicsPtr<IColliderGeometry> geometry1 = PhysicsEngineUtils::CreateColliderGeometry(options);

		for (uint32_t i = 0; i < size; i++)
		{
			for (uint32_t j = 0; j < size - i; j++)
			{
				MathLib::HTransform3 localTm(MathLib::HTranslation3(MathLib::HVector3(MathLib::HReal(j * 2) - MathLib::HReal(size - i), MathLib::HReal(i * 2 + 1), 0) * halfExtent));

				PhysicsObjectCreateOptions objectOptions;
				objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
				objectOptions.m_Transform = t * localTm;
				RandomRigidBodyType(objectOptions.m_ObjectType);
				PhysicsPtr < IPhysicsObject> physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
				if (!physicsObject->AddColliderGeometry(geometry0, MathLib::HTransform3::Identity()))
					physicsObject->AddColliderGeometry(geometry1, MathLib::HTransform3::Identity());
				objects.push_back(physicsObject);
			}
		}
		return objects;
	}

	static std::vector<PhysicsPtr<IPhysicsObject>> CreateSimpleConvexMeshStack(const MathLib::HTransform3& t, uint32_t size, MathLib::HReal halfExtent)
	{
		std::vector<PhysicsPtr<IPhysicsObject>> objects;
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH;
		options.m_ConvexMeshParams.m_Vertices = ConvexMeshData.m_Vertices;
		options.m_Scale = MathLib::HVector3(3.0f, 3.0f, 3.0f);

		PhysicsPtr<IColliderGeometry> geometry = PhysicsEngineUtils::CreateColliderGeometry(options);

		for (uint32_t i = 0; i < size; i++)
		{
			for (uint32_t j = 0; j < size - i; j++)
			{
				MathLib::HTransform3 localTm(MathLib::HTranslation3(MathLib::HVector3(MathLib::HReal(j * 2) - MathLib::HReal(size - i), MathLib::HReal(i * 2 + 1), 0) * halfExtent));

				PhysicsObjectCreateOptions objectOptions;
				objectOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
				objectOptions.m_Transform = t * localTm;
				RandomRigidBodyType(objectOptions.m_ObjectType);
				PhysicsPtr < IPhysicsObject> physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
				physicsObject->AddColliderGeometry(geometry, MathLib::HTransform3::Identity());
				objects.push_back(physicsObject);
			}
		}
		return objects;
	}

	static std::vector<PhysicsPtr<IPhysicsObject>> TestRigidBodyCreate()
	{
		std::vector<PhysicsPtr<IPhysicsObject>> objects;
		auto decomposeConvexStack= CreateDeComposeConvexStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		auto boxStack=CreateBoxStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		auto sphereStack= CreateSphereStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		auto capsuleStack=CreateCapsuleStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		auto triangleMeshStack=CreateTriangleMeshStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		auto simpleConvexMeshStack=CreateSimpleConvexMeshStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		objects.insert(objects.end(), decomposeConvexStack.begin(), decomposeConvexStack.end());
		objects.insert(objects.end(), boxStack.begin(), boxStack.end());
		objects.insert(objects.end(), sphereStack.begin(), sphereStack.end());
		objects.insert(objects.end(), capsuleStack.begin(), capsuleStack.end());
		objects.insert(objects.end(), triangleMeshStack.begin(), triangleMeshStack.end());
		objects.insert(objects.end(), simpleConvexMeshStack.begin(), simpleConvexMeshStack.end());
		return objects;
	}
};