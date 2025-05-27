#pragma once
#include "Physics/PhysicsCommon.h"

namespace physx
{
	class PxSoftBody;
	class PxTetrahedronMesh;
	class PxSoftBodyMesh;
	class PxFEMSoftBodyMaterial;
}

class PhysicsSoftBody : public ISoftBody
{
public:
	PhysicsSoftBody(PhysicsPtr<IPhysicsMaterial>& material);
	~PhysicsSoftBody();

public:	
	void Release() override;
	void Update() override;
	bool IsValid() const override { return m_SoftBody != nullptr; }
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

	void SetParameter(const SoftBodyParams& params) override;
	SoftBodyParams GetParameter() const override { return m_Params; }
	uint32_t GetVertexCount() const override;
	uint32_t GetTetrahedronCount() const override;
	void GetDeformedVertexPositions(std::vector<MathLib::HVector3>& positions) const override;
	void GetTetrahedronIndices(std::vector<uint32_t>& indices) const override;
	void SetVertexFixed(uint32_t vertexIndex, bool fixed) override;
	bool IsVertexFixed(uint32_t vertexIndex) const override;
	void ApplyForceToVertex(uint32_t vertexIndex, const MathLib::HVector3& force) override;
	void SetMass(MathLib::HReal mass) override;
	MathLib::HReal GetMass() const override { return m_Params.m_Mass; }
	void UpdateMass(MathLib::HReal density) override;
	void Transform(const MathLib::HTransform3& transform, const MathLib::HVector3& scale = MathLib::HVector3(1.0f, 1.f, 1.f)) override;
	void GetOriginalVertexPositions(std::vector<MathLib::HVector3>& positions) const override;
	void* GetTetrahedronMesh() const override;
	void* GetSoftBodyData() const override;
	void Commit() override;
	void GetRenderMesh(std::vector<MathLib::HVector3>& vertices, std::vector<uint32_t>& indices) const override;

	bool CreateFromMesh(const SoftBodyCreateOptions& options);

private:
	bool InitializeSoftBody(const SoftBodyCreateOptions& options);
	bool CreateTetrahedronMesh(const SoftBodyMeshDesc& meshDesc, bool isCollisionMesh);
	void UpdateBoundingBox();

private:
	PhysicsObjectType m_Type;
	PhysXPtr<physx::PxSoftBody> m_SoftBody;
	PhysXPtr<physx::PxTetrahedronMesh> m_SimulationMesh;
	PhysXPtr<physx::PxTetrahedronMesh> m_CollisionMesh;
	PhysXPtr<physx::PxSoftBodyMesh> m_SoftBodyMesh;
	PhysXPtr<physx::PxFEMSoftBodyMaterial> m_FEMMaterial;
	PhysicsPtr<IPhysicsMaterial> m_Material;
	std::vector<PhysicsPtr<IColliderGeometry>> m_ColliderGeometries;
	std::vector<MathLib::HTransform3> m_ColliderLocalPos;
	std::vector<bool> m_FixedVertices;
	std::vector<MathLib::HVector3> m_OriginalVertices;
	std::vector<uint32_t> m_TetrahedronIndices;
	SoftBodyParams m_Params;
	MathLib::HTransform3 m_Transform;
	MathLib::HAABBox3D m_BoundingBox;
	void* m_UserData = nullptr;
	uint32_t m_CollisionLayer = 1;
	uint32_t m_CollisionMask = 0xFFFFFFFF;
	ICollisionCallback* m_CollisionCallback = nullptr;
}; 