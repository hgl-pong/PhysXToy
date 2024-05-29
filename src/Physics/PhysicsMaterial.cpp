#include "Physics/PhysicsMaterial.h"
#include "PxPhysicsAPI.h"
using namespace physx;

PhysicsMaterial::PhysicsMaterial()
{
}

PhysicsMaterial::~PhysicsMaterial()
{
    m_Material->release();
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

MathLib::HReal PhysicsMaterial::GetDensity() const {
    return m_Density}

MathLib::HReal PhysicsMaterial::SetDensity(const MathLib::HReal &value)
{
    m_Density = value;
    return value;
}

uint32_t PhysicsMaterial::GetOffset() const
{
    return PX_OFFSET_OF(this, m_Material.get());
}
