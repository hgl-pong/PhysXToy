#pragma once

inline  unsigned RandomUInt(unsigned range)
{
	return rand() % range;
}
namespace TestRigidBody
{
	static MathLib::HReal stackZ = 10.0f;
	static IPhysicsObject* CreateDynamic(const MathLib::HTransform3& t, IColliderGeometry& geometry, const MathLib::HVector3& velocity = MathLib::HVector3(0, 0, 0))
	{
		PhysicsObjectCreateOptions createOptions{};
		createOptions.m_ObjectType = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
		createOptions.m_Transform = t;
		IPhysicsObject* physicsObject = PhysicsEngineUtils::CreateObject(createOptions);
		IDynamicObject* rigidDynamic = dynamic_cast<IDynamicObject*>(physicsObject);
		physicsObject->AddColliderGeometry(&geometry, MathLib::HTransform3::Identity());
		rigidDynamic->SetAngularDamping(0.5);
		rigidDynamic->SetLinearVelocity(velocity);
		if (gScene)
			gScene->AddPhysicsObject(physicsObject);
		return physicsObject;
	}

	static void CreateDeComposeConvexStack(const MathLib::HTransform3& t, uint32_t size, MathLib::HReal halfExtent)
	{
		std::vector<MathLib::HVector3> triVerts;
		std::vector<uint32_t> triIndices;

		uint32_t numVerts = MeshGenerateUtils::Bunny_getNbVerts();
		uint32_t numFaces = MeshGenerateUtils::Bunny_getNbFaces();

		triVerts.resize(numVerts);
		triIndices.resize(numFaces * 3);

		memcpy(triVerts.data(), MeshGenerateUtils::Bunny_getVerts(), sizeof(MathLib::HVector3) * numVerts);
		memcpy(triIndices.data(), MeshGenerateUtils::Bunny_getFaces(), sizeof(uint32_t) * numFaces * 3);
		PhysicsMeshData meshdata;

		meshdata.m_Vertices = triVerts;
		meshdata.m_Indices = triIndices;

		ConvexDecomposeOptions decomposeOptions;
		decomposeOptions.m_VoxelGridResolution = 1000;
		decomposeOptions.m_MaximumNumberOfHulls = 5;
		std::vector<PhysicsMeshData> decomposedMeshes;
		PhysicsEngineUtils::ConvexDecomposition(meshdata, decomposeOptions, decomposedMeshes);

		std::vector<PhysicsPtr<IColliderGeometry>> geos(decomposedMeshes.size());
		for (size_t i = 0; i < decomposedMeshes.size(); i++)
		{
			CollisionGeometryCreateOptions options;
			options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH;
			options.m_ConvexMeshParams.m_Vertices = decomposedMeshes[i].m_Vertices;
			options.m_Scale = MathLib::HVector3(3.0f, 3.0f, 3.0f);
			geos[i] = make_physics_ptr<IColliderGeometry>(PhysicsEngineUtils::CreateColliderGeometry(options));
		}

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
				IPhysicsObject* physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
				for (size_t i = 0; i < geos.size(); i++)
				{
					if (!physicsObject->AddColliderGeometry(geos[i].get(), MathLib::HTransform3::Identity()))
					{
						physicsObject->AddColliderGeometry(geos[0].get(), MathLib::HTransform3::Identity());
					}
				}
				gScene->AddPhysicsObject(physicsObject);
			}
		}
	}

	static void CreateBoxStack(const MathLib::HTransform3& t, uint32_t size, MathLib::HReal halfExtent)
	{
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX;
		options.m_BoxParams.m_HalfExtents = MathLib::HVector3(halfExtent, halfExtent, halfExtent);

		PhysicsPtr<IColliderGeometry> geometry = make_physics_ptr<IColliderGeometry>(PhysicsEngineUtils::CreateColliderGeometry(options));

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
				IPhysicsObject* physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
				physicsObject->AddColliderGeometry(geometry.get(), MathLib::HTransform3::Identity());
				gScene->AddPhysicsObject(physicsObject);
			}
		}
	}

	static void CreateSphereStack(const MathLib::HTransform3& t, uint32_t size, MathLib::HReal halfExtent)
	{
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE;
		options.m_SphereParams.m_Radius = halfExtent;

		PhysicsPtr<IColliderGeometry> geometry = make_physics_ptr<IColliderGeometry>(PhysicsEngineUtils::CreateColliderGeometry(options));

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

				IPhysicsObject* physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
				physicsObject->AddColliderGeometry(geometry.get(), MathLib::HTransform3::Identity());
				gScene->AddPhysicsObject(physicsObject);
			}
		}
	}

	static void CreateCapsuleStack(const MathLib::HTransform3& t, uint32_t size, MathLib::HReal halfExtent)
	{
		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE;
		options.m_CapsuleParams.m_HalfHeight = halfExtent / 2;
		options.m_CapsuleParams.m_Radius = halfExtent / 2;

		PhysicsPtr<IColliderGeometry> geometry = make_physics_ptr<IColliderGeometry>(PhysicsEngineUtils::CreateColliderGeometry(options));

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

				IPhysicsObject* physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
				physicsObject->AddColliderGeometry(geometry.get(), MathLib::HTransform3::Identity());
				gScene->AddPhysicsObject(physicsObject);
			}
		}
	}

	static void CreateTriangleMeshStack(const MathLib::HTransform3& t, uint32_t size, MathLib::HReal halfExtent)
	{
		std::vector<MathLib::HVector3> triVerts;
		std::vector<uint32_t> triIndices;

		uint32_t numVerts = MeshGenerateUtils::Bunny_getNbVerts();
		uint32_t numFaces = MeshGenerateUtils::Bunny_getNbFaces();

		triVerts.resize(numVerts);
		triIndices.resize(numFaces * 3);

		memcpy(triVerts.data(), MeshGenerateUtils::Bunny_getVerts(), sizeof(MathLib::HVector3) * numVerts);
		memcpy(triIndices.data(), MeshGenerateUtils::Bunny_getFaces(), sizeof(uint32_t) * numFaces * 3);
		PhysicsMeshData meshdata;

		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH;
		options.m_TriangleMeshParams.m_Vertices = triVerts;
		options.m_TriangleMeshParams.m_Indices = triIndices;
		options.m_Scale = MathLib::HVector3(3.0f, 3.0f, 3.0f);

		PhysicsPtr<IColliderGeometry> geometry0 = make_physics_ptr<IColliderGeometry>(PhysicsEngineUtils::CreateColliderGeometry(options));
		PhysicsEngineUtils::BuildConvexMesh(triVerts, triIndices, meshdata);
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH;
		options.m_ConvexMeshParams.m_Vertices = meshdata.m_Vertices;
		options.m_Scale = MathLib::HVector3(3.0f, 3.0f, 3.0f);
		PhysicsPtr<IColliderGeometry> geometry1 = make_physics_ptr<IColliderGeometry>(PhysicsEngineUtils::CreateColliderGeometry(options));

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

				IPhysicsObject* physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
				if (!physicsObject->AddColliderGeometry(geometry0.get(), MathLib::HTransform3::Identity()))
					physicsObject->AddColliderGeometry(geometry1.get(), MathLib::HTransform3::Identity());
				gScene->AddPhysicsObject(physicsObject);
			}
		}
	}

	static void CreateSimpleConvexMeshStack(const MathLib::HTransform3& t, uint32_t size, MathLib::HReal halfExtent)
	{
		std::vector<MathLib::HVector3> triVerts;
		std::vector<uint32_t> triIndices;

		uint32_t numVerts = MeshGenerateUtils::Bunny_getNbVerts();
		uint32_t numFaces = MeshGenerateUtils::Bunny_getNbFaces();

		triVerts.resize(numVerts);
		triIndices.resize(numFaces * 3);

		memcpy(triVerts.data(), MeshGenerateUtils::Bunny_getVerts(), sizeof(MathLib::HVector3) * numVerts);
		memcpy(triIndices.data(), MeshGenerateUtils::Bunny_getFaces(), sizeof(uint32_t) * numFaces * 3);
		PhysicsMeshData meshdata;

		PhysicsEngineUtils::BuildConvexMesh(triVerts, triIndices, meshdata);

		CollisionGeometryCreateOptions options;
		options.m_GeometryType = CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH;
		options.m_ConvexMeshParams.m_Vertices = meshdata.m_Vertices;
		options.m_Scale = MathLib::HVector3(3.0f, 3.0f, 3.0f);

		PhysicsPtr<IColliderGeometry> geometry = make_physics_ptr<IColliderGeometry>(PhysicsEngineUtils::CreateColliderGeometry(options));

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

				IPhysicsObject* physicsObject = PhysicsEngineUtils::CreateObject(objectOptions);
				physicsObject->AddColliderGeometry(geometry.get(), MathLib::HTransform3::Identity());
				gScene->AddPhysicsObject(physicsObject);
			}
		}
	}

	static void TestRigidBodyCreate()
	{
		CreateDeComposeConvexStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		CreateBoxStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		CreateSphereStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		CreateCapsuleStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		CreateTriangleMeshStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		CreateSimpleConvexMeshStack(MathLib::HTransform3(MathLib::HTranslation3(MathLib::HVector3(0, 0, stackZ -= 10.0f))), 10, 2.0f);
		printf("Number of Actor:%d\n",gScene->GetPhysicsObjectCount());
	}
};