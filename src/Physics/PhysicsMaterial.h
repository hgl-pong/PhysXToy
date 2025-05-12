#pragma once
#include "Physics/PhysicsCommon.h"
#include "Base/ObjectCache.h"
namespace physx
{
	class PxMaterial;
}

class PhysicsMaterial : public IPhysicsMaterial
{
public:
	PhysicsMaterial(const PhysicsMaterialCreateOptions& options);
	void Release()override;
	MathLib::HReal GetStaticFriction() const override;
	MathLib::HReal GetDynamicFriction() const override;
	MathLib::HReal GetRestitution() const override;
	MathLib::HReal SetStaticFriction(const MathLib::HReal &value) override;
	MathLib::HReal SetDynamicFriction(const MathLib::HReal &value) override;
	MathLib::HReal SetRestitution(const MathLib::HReal &value) override;
	MathLib::HReal GetDensity() const override;
	MathLib::HReal SetDensity(const MathLib::HReal &value) override;
	size_t GetOffset() const override;
	
	physx::PxMaterial* GetMaterial() const { return m_Material.get(); }

private:
	PhysXPtr<physx::PxMaterial> m_Material;
	MathLib::HReal m_Density;
};

namespace PhysicsBase
{
    template<>
    struct Creator<PhysicsMaterialCreateOptions, IPhysicsMaterial>
    {
        PhysicsPtr<IPhysicsMaterial> Create(const PhysicsMaterialCreateOptions& options)
        {
            return make_physics_ptr(new PhysicsMaterial(options));
        }
    };
}

namespace std
{
    template <>
    struct hash<PhysicsMaterialCreateOptions>
    {
        size_t operator()(const PhysicsMaterialCreateOptions& material) const
        {
            return hash<MathLib::HReal>()(material.m_Restitution) ^ 
                   hash<MathLib::HReal>()(material.m_StaticFriction) ^ 
                   hash<MathLib::HReal>()(material.m_DynamicFriction) ^ 
                   hash<MathLib::HReal>()(material.m_Density);
        }
    };

    template <>
    struct equal_to<PhysicsMaterialCreateOptions>
    {
        bool operator()(const PhysicsMaterialCreateOptions& lhs, const PhysicsMaterialCreateOptions& rhs) const
        {
            return lhs.m_Restitution == rhs.m_Restitution && 
                   lhs.m_StaticFriction == rhs.m_StaticFriction && 
                   lhs.m_DynamicFriction == rhs.m_DynamicFriction && 
                   lhs.m_Density == rhs.m_Density;
        }
    };
}

using PhysicsMaterialCache = ObjectCache<IPhysicsMaterial, PhysicsMaterialCreateOptions>;