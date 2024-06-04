#pragma once
#include "Physics/PhysicsTypes.h"
#include <memory>
#include <unordered_set>
#include <unordered_map>

class IPhysicsEngine;
class IPhysicsScene;
class IColliderGeometry;
class IPhysicsObject;
class IPhysicsMaterial;

class IPhysicsEngine
{
public:
	virtual PhysicsPtr<IPhysicsObject> CreateObject(const PhysicsObjectCreateOptions &options) = 0;
	virtual PhysicsPtr<IPhysicsMaterial> CreateMaterial(const PhysicsMaterialCreateOptions &options) = 0;
	virtual PhysicsPtr<IPhysicsScene> CreateScene(const PhysicsSceneCreateOptions &options) = 0;
	virtual PhysicsPtr<IColliderGeometry> CreateColliderGeometry(const CollisionGeometryCreateOptions &options) = 0;
};

class IPhysicsScene
{
public:
	virtual void Release() = 0;
	virtual void Tick(MathLib::HReal deltaTime) = 0;
	virtual bool AddPhysicsObject(PhysicsPtr < IPhysicsObject >&physicsObject) = 0;
	virtual void RemovePhysicsObject(PhysicsPtr < IPhysicsObject >&physicsObject) = 0;
	virtual uint32_t GetPhysicsObjectCount() const = 0;
	virtual uint32_t GetPhysicsRigidDynamicCount() const = 0;
	virtual uint32_t GetPhysicsRigidStaticCount() const = 0;
	virtual size_t GetOffset() const = 0;
};

class IColliderGeometry
{
public:
	virtual void Release() = 0;
	virtual CollierGeometryType GetType() const = 0;
	virtual void SetScale(const MathLib::HVector3 &scale) = 0;
	virtual void GetParams(CollisionGeometryCreateOptions& options) = 0;
};

class IPhysicsObject
{
public:
	virtual void Release() = 0;
	virtual void Update() = 0;
	virtual bool AddColliderGeometry(PhysicsPtr < IColliderGeometry >&colliderGeometry, const MathLib::HTransform3 &localTrans) = 0;
	virtual void GetColliderGeometries(std::vector<PhysicsPtr<IColliderGeometry>>& geomeries) = 0;
	virtual PhysicsObjectType GetType() const = 0;
	virtual size_t GetOffset() const = 0;
	virtual void SetTransform(const MathLib::HTransform3 &trans) = 0;
	virtual const MathLib::HTransform3 &GetTransform() const = 0;
	virtual bool IsValid() const = 0;
};

class IDynamicObject
{
public:
	virtual void SetAngularDamping(const MathLib::HReal &damping) = 0;
	virtual void SetLinearVelocity(const MathLib::HVector3 &velocity) = 0;
	virtual void SetAngularVelocity(const MathLib::HVector3 &velocity) = 0;
	virtual void SetKinematic(bool bKinematic) = 0;
	virtual bool IsKinematic() const = 0;
	virtual MathLib::HReal GetMass() const = 0;
	virtual MathLib::HVector3 GetLinearVelocity() const = 0;
	virtual MathLib::HReal GetAngularDamping() const = 0;
	virtual MathLib::HVector3 GetAngularVelocity() const = 0;
};

class IPhysicsMaterial
{
public:
	virtual void Release() = 0;
	virtual MathLib::HReal GetStaticFriction() const = 0;
	virtual MathLib::HReal GetDynamicFriction() const = 0;
	virtual MathLib::HReal GetRestitution() const = 0;
	virtual MathLib::HReal SetStaticFriction(const MathLib::HReal &value) = 0;
	virtual MathLib::HReal SetDynamicFriction(const MathLib::HReal &value) = 0;
	virtual MathLib::HReal SetRestitution(const MathLib::HReal &value) = 0;
	virtual MathLib::HReal GetDensity() const = 0;
	virtual MathLib::HReal SetDensity(const MathLib::HReal &value) = 0;
	virtual size_t GetOffset() const = 0;
};

class PhysicsEngineUtils
{
public:
	static IPhysicsEngine *CreatePhysicsEngine(const PhysicsEngineOptions &options, const bool createConvexDecomposer = true);
	static void DestroyPhysicsEngine();
	static IPhysicsEngine *GetPhysicsEngine();
	static PhysicsPtr<IPhysicsObject> CreateObject(const PhysicsObjectCreateOptions &options);
	static PhysicsPtr<IPhysicsMaterial> CreateMaterial(const PhysicsMaterialCreateOptions &options);
	static PhysicsPtr<IPhysicsScene> CreateScene(const PhysicsSceneCreateOptions &options);
	static PhysicsPtr<IColliderGeometry> CreateColliderGeometry(const CollisionGeometryCreateOptions &options);
	static void BuildConvexMesh(const std::vector<MathLib::HVector3> &vertices, const std::vector<uint32_t> &indices, PhysicsMeshData &meshdata);
	static bool ConvexDecomposition(const PhysicsMeshData &meshData, const ConvexDecomposeOptions &params, std::vector<PhysicsMeshData> &convexMeshesData);
};
