#pragma once
#include "Physics/PhysicsCommon.h"
namespace physx
{
	class PxMaterial;
}

class PhysicsMaterial : public IPhysicsMaterial
{
public:
	PhysicsMaterial();
	~PhysicsMaterial();

	MathLib::HReal GetStaticFriction() const override;
	MathLib::HReal GetDynamicFriction() const override;
	MathLib::HReal GetRestitution() const override;
	MathLib::HReal SetStaticFriction(const MathLib::HReal &value) override;
	MathLib::HReal SetDynamicFriction(const MathLib::HReal &value) override;
	MathLib::HReal SetRestitution(const MathLib::HReal &value) override;
	MathLib::HReal GetDensity() const override;
	MathLib::HReal SetDensity(const MathLib::HReal &value) override;
	size_t GetOffset() const override;

private:
	friend class PhysicsEngine;
	physx::PxMaterial* m_Material = nullptr;
	MathLib::HReal m_Density;
};