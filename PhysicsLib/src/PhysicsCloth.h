#pragma once
#include "Physics/PhysicsCommon.h"

namespace physx
{
	class PxPBDParticleSystem;
	class PxParticleClothBuffer;
	class PxTriangleMesh;
	class PxPBDMaterial;
}

class PhysicsCloth : public ICloth
{
public:
	PhysicsCloth(PhysicsPtr<IPhysicsMaterial>& material);
	~PhysicsCloth();

public:	
	void Release() override;
	void Update() override;
	bool IsValid() const override { return m_ParticleSystem != nullptr && m_ClothBuffer != nullptr; }
	bool AddColliderGeometry(PhysicsPtr<IColliderGeometry>& colliderGeometry, const MathLib::HTransform3& localTrans) override;
	bool RemoveColliderGeometry(PhysicsPtr<IColliderGeometry>& colliderGeometry) override;
	void GetColliderGeometries(std::vector<PhysicsPtr<IColliderGeometry>>& geomeries, std::vector<MathLib::HTransform3>* geoLocalPos = nullptr) override;
	PhysicsObjectType GetType() const override { return m_Type; }
	size_t GetOffset() const override;
	void SetTransform(const MathLib::HTransform3& trans) override;
	const MathLib::HTransform3& GetTransform() const override { return m_Transform; }
	MathLib::HAABBox3D GetLocalBoundingBox() const override { return m_BoundingBox; }
	MathLib::HAABBox3D GetWorldBoundingBox() const override;
	void SetUserData(void* userData) override { m_UserData = userData; }
	void* GetUserData() const override { return m_UserData; }
	void SetCollisionLayer(uint32_t layer) override { m_CollisionLayer = layer; }
	uint32_t GetCollisionLayer() const override { return m_CollisionLayer; }
	void SetCollisionMask(uint32_t mask) override { m_CollisionMask = mask; }
	uint32_t GetCollisionMask() const override { return m_CollisionMask; }
	void SetCollisionCallback(ICollisionCallback* callback) override { m_CollisionCallback = callback; }
	ICollisionCallback* GetCollisionCallback() const override { return m_CollisionCallback; }

	void SetParameter(const ClothParams& params) override;
	ClothParams GetParameter() const override { return m_Params; }
	uint32_t GetVertexCount() const override;
	uint32_t GetTriangleCount() const override;
	void GetDeformedVertexPositions(std::vector<MathLib::HVector3>& positions) const override;
	void GetTriangleIndices(std::vector<uint32_t>& indices) const override;
	void SetVertexFixed(uint32_t vertexIndex, bool fixed) override;
	bool IsVertexFixed(uint32_t vertexIndex) const override;
	void ApplyForceToVertex(uint32_t vertexIndex, const MathLib::HVector3& force) override;
	void ApplyWindForce(const MathLib::HVector3& windDirection, MathLib::HReal strength) override;
	void SetMass(MathLib::HReal mass) override;
	MathLib::HReal GetMass() const override { return m_Params.m_Mass; }
	void GetOriginalVertexPositions(std::vector<MathLib::HVector3>& positions) const override;
	void* GetClothData() const override;
	void Commit() override;
	void GetRenderMesh(std::vector<MathLib::HVector3>& vertices, std::vector<uint32_t>& indices) const override;

	bool CreateFromMesh(const ClothCreateOptions& options);

private:
	bool InitializeCloth(const ClothCreateOptions& options);
	bool CreateClothMesh(const ClothMeshDesc& meshDesc);
	void UpdateBoundingBox();
	void CreateGridMesh(int width, int height, MathLib::HReal spacing);
	void UpdateDeformedVertices();

private:
	PhysicsObjectType m_Type;
	PhysXPtr<physx::PxPBDParticleSystem> m_ParticleSystem;
	PhysXPtr<physx::PxParticleClothBuffer> m_ClothBuffer;
	PhysXPtr<physx::PxTriangleMesh> m_TriangleMesh;
	PhysXPtr<physx::PxPBDMaterial> m_PBDMaterial;
	PhysicsPtr<IPhysicsMaterial> m_Material;
	std::vector<PhysicsPtr<IColliderGeometry>> m_ColliderGeometries;
	std::vector<MathLib::HTransform3> m_ColliderLocalPos;
	std::vector<bool> m_FixedVertices;
	std::vector<MathLib::HVector3> m_OriginalVertices;
	std::vector<MathLib::HVector3> m_DeformedVertices;
	std::vector<uint32_t> m_TriangleIndices;
	ClothParams m_Params;
	MathLib::HTransform3 m_Transform;
	MathLib::HAABBox3D m_BoundingBox;
	void* m_UserData = nullptr;
	uint32_t m_CollisionLayer = 1;
	uint32_t m_CollisionMask = 0xFFFFFFFF;
	ICollisionCallback* m_CollisionCallback = nullptr;
	int m_Width = 0;
	int m_Height = 0;
}; 