#include "PhysicsMaterial.h"
#include "PxPhysicsAPI.h"
using namespace physx;

PhysicsMaterial::PhysicsMaterial(const PhysicsMaterialCreateOptions& options)
{
    m_Material = make_physx_ptr<PxMaterial>(PxGetPhysics().createMaterial(options.m_StaticFriction, options.m_DynamicFriction, options.m_Restitution));
    m_Density = options.m_Density;
}

void PhysicsMaterial::Release()
{
    m_Material.reset();
}

MathLib::HReal PhysicsMaterial::GetStaticFriction() const
{
    return m_Material->getStaticFriction();
}

MathLib::HReal PhysicsMaterial::GetDynamicFriction() const
{
    return m_Material->getDynamicFriction();
}

MathLib::HReal PhysicsMaterial::GetRestitution() const
{
    return m_Material->getRestitution();
}

MathLib::HReal PhysicsMaterial::SetStaticFriction(const MathLib::HReal &value)
{
    m_Material->setStaticFriction(value);
    return value;
}

MathLib::HReal PhysicsMaterial::SetDynamicFriction(const MathLib::HReal &value)
{
    m_Material->setDynamicFriction(value);
    return value;
}

MathLib::HReal PhysicsMaterial::SetRestitution(const MathLib::HReal &value)
{
    m_Material->setRestitution(value);
    return value;
}

MathLib::HReal PhysicsMaterial::GetDensity() const 
{
    return m_Density;
}

MathLib::HReal PhysicsMaterial::SetDensity(const MathLib::HReal &value)
{
    m_Density = value;
    return value;
}

size_t PhysicsMaterial::GetOffset() const
{
    return offsetof(PhysicsMaterial, m_Material);
}
