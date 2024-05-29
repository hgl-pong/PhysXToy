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
	virtual void Init(const PhysicsEngineOptions &options) = 0;
	virtual void UnInit() = 0;
	virtual IPhysicsObject *CreateObject(const PhysicsObjectCreateOptions &options) = 0;
	virtual IPhysicsMaterial *CreateMaterial(const PhysicsMaterialCreateOptions &options) = 0;
	virtual IPhysicsScene *CreateScene(const PhysicsSceneCreateOptions &options) = 0;
	virtual IColliderGeometry *CreateColliderGeometry(const CollisionGeometryCreateOptions &options) = 0;
};

class IPhysicsScene
{
public:
	virtual void Tick(float deltaTime) = 0;
	virtual bool AddPhysicsObject(IPhysicsObject *physicsObject) = 0;
	virtual void RemovePhysicsObject(IPhysicsObject *physicsObject) = 0;
	virtual uint32_t GetPhysicsObjectCount() const = 0;
	virtual uint32_t GetOffset()const = 0;
};

class IColliderGeometry
{
public:
	virtual CollierGeometryType GetType() const = 0;
	virtual void SetScale(const MathLib::HVector3 &scale) = 0;
};

class IPhysicsObject
{
public:
	virtual void AddColliderGeometry(IColliderGeometry *colliderGeometry, const MathLib::HTransform3 &localTrans) = 0;
	virtual PhysicsObjectType GetType() const = 0;
	virtual uint32_t GetOffset() const = 0;
	virtual void SetTransform(const MathLib::HTransform3 &trans) = 0;
};

class IPhysicsMaterial
{
public:
	virtual MathLib::HReal GetStaticFriction() const = 0;
	virtual MathLib::HReal GetDynamicFriction() const = 0;
	virtual MathLib::HReal GetRestitution() const = 0;
	virtual MathLib::HReal SetStaticFriction(const MathLib::HReal &value) = 0;
	virtual MathLib::HReal SetDynamicFriction(const MathLib::HReal &value) = 0;
	virtual MathLib::HReal SetRestitution(const MathLib::HReal &value) = 0;
	virtual MathLib::HReal GetDensity() const = 0;
	virtual MathLib::HReal SetDensity(const MathLib::HReal &value) = 0;
	virtual uint32_t GetOffset() const = 0;
};

class PhysicsEngineUtils
{
public:
	static IPhysicsEngine* CreatePhysicsEngine();
	static void DestroyPhysicsEngine();
};

