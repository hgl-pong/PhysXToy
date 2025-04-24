#include "PhysicsJoint.h"
#include "PhysicsRigid.h"
#include <PxPhysicsAPI.h>
#include "Utility/PhysXUtils.h"

using namespace physx;

PhysicsJoint::PhysicsJoint(JointType type, PhysicsPtr<IPhysicsObject> objectA, PhysicsPtr<IPhysicsObject> objectB,
                           const MathLib::HTransform3& localFrameA, const MathLib::HTransform3& localFrameB, 
                           bool collisionEnabled)
    : m_Type(type)
    , m_ObjectA(objectA)
    , m_ObjectB(objectB)
    , m_LocalFrameA(localFrameA)
    , m_LocalFrameB(localFrameB)
    , m_CollisionEnabled(collisionEnabled)
    , m_IsBroken(false)
    , m_BreakForce(FLT_MAX)
    , m_BreakTorque(FLT_MAX)
    , m_PxJoint(nullptr)
{
    switch (m_Type)
    {
    case JointType::FIXED:
        CreateFixedJoint();
        break;
    case JointType::DISTANCE:
        CreateDistanceJoint();
        break;
    case JointType::SPHERICAL:
        CreateSphericalJoint();
        break;
    case JointType::REVOLUTE:
        CreateRevoluteJoint();
        break;
    case JointType::PRISMATIC:
        CreatePrismaticJoint();
        break;
    case JointType::D6:
        CreateD6Joint();
        break;
    default:
        break;
    }

    if (m_PxJoint)
    {
        m_PxJoint->setBreakForce(m_BreakForce, m_BreakTorque);
        m_PxJoint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, m_CollisionEnabled);
    }
}

PhysicsJoint::~PhysicsJoint()
{
    Release();
}

void PhysicsJoint::Release()
{
    if (m_PxJoint)
    {
        m_PxJoint->release();
        m_PxJoint = nullptr;
    }
    m_ObjectA = nullptr;
    m_ObjectB = nullptr;
}

JointType PhysicsJoint::GetType() const
{
    return m_Type;
}

void PhysicsJoint::SetBreakForce(const MathLib::HReal& force)
{
    m_BreakForce = force;
    if (m_PxJoint)
    {
        m_PxJoint->setBreakForce(m_BreakForce, m_BreakTorque);
    }
}

MathLib::HReal PhysicsJoint::GetBreakForce() const
{
    return m_BreakForce;
}

void PhysicsJoint::SetBreakTorque(const MathLib::HReal& torque)
{
    m_BreakTorque = torque;
    if (m_PxJoint)
    {
        m_PxJoint->setBreakForce(m_BreakForce, m_BreakTorque);
    }
}

MathLib::HReal PhysicsJoint::GetBreakTorque() const
{
    return m_BreakTorque;
}

PhysicsPtr<IPhysicsObject> PhysicsJoint::GetObjectA() const
{
    return m_ObjectA;
}

PhysicsPtr<IPhysicsObject> PhysicsJoint::GetObjectB() const
{
    return m_ObjectB;
}

void PhysicsJoint::SetLocalPose(bool isObjectA, const MathLib::HTransform3& pose)
{
    if (isObjectA)
    {
        m_LocalFrameA = pose;
    }
    else
    {
        m_LocalFrameB = pose;
    }

    UpdateJointPose();
}

MathLib::HTransform3 PhysicsJoint::GetLocalPose(bool isObjectA) const
{
    return isObjectA ? m_LocalFrameA : m_LocalFrameB;
}

bool PhysicsJoint::IsBroken() const
{
    return m_IsBroken;
}

size_t PhysicsJoint::GetOffset() const
{
    return reinterpret_cast<size_t>(m_PxJoint);
}

void PhysicsJoint::SetJointLimits(const JointLimitOptions& limitOptions)
{
    m_LimitOptions = limitOptions;

    switch (m_Type)
    {
    case JointType::DISTANCE:
        if (m_PxDistanceJoint)
        {
            m_PxDistanceJoint->setMinDistance(m_LimitOptions.m_XAxis.m_LowerLimit);
            m_PxDistanceJoint->setMaxDistance(m_LimitOptions.m_XAxis.m_UpperLimit);
            m_PxDistanceJoint->setStiffness(m_LimitOptions.m_XAxis.m_Stiffness);
            m_PxDistanceJoint->setDamping(m_LimitOptions.m_XAxis.m_Damping);
            m_PxDistanceJoint->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, m_LimitOptions.m_XAxis.m_IsLimited);
            m_PxDistanceJoint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, m_LimitOptions.m_XAxis.m_IsLimited);
        }
        break;
    case JointType::REVOLUTE:
        if (m_PxRevoluteJoint)
        {
            if (m_LimitOptions.m_Twist.m_IsLimited)
            {
                m_PxRevoluteJoint->setLimit(PxJointAngularLimitPair(
                    m_LimitOptions.m_Twist.m_LowerLimit, 
                    m_LimitOptions.m_Twist.m_UpperLimit, 
                    PxSpring(m_LimitOptions.m_Twist.m_Stiffness, m_LimitOptions.m_Twist.m_Damping)));
                m_PxRevoluteJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
            }
            else
            {
                m_PxRevoluteJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, false);
            }
        }
        break;
    case JointType::PRISMATIC:
        if (m_PxPrismaticJoint)
        {
            if (m_LimitOptions.m_XAxis.m_IsLimited)
            {
                m_PxPrismaticJoint->setLimit(PxJointLinearLimitPair(
                    PxTolerancesScale(), 
                    m_LimitOptions.m_XAxis.m_LowerLimit, 
                    m_LimitOptions.m_XAxis.m_UpperLimit));
                m_PxPrismaticJoint->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, true);
            }
            else
            {
                m_PxPrismaticJoint->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, false);
            }
        }
        break;
    case JointType::D6:
        if (m_PxD6Joint)
        {
            if (m_LimitOptions.m_XAxis.m_IsLimited)
            {
                m_PxD6Joint->setMotion(PxD6Axis::eX, PxD6Motion::eLIMITED);
                m_PxD6Joint->setLinearLimit(PxD6Axis::eX, PxJointLinearLimitPair(
                    PxTolerancesScale(), 
                    m_LimitOptions.m_XAxis.m_LowerLimit, 
                    m_LimitOptions.m_XAxis.m_UpperLimit));
            }
            else
            {
                m_PxD6Joint->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
            }

            if (m_LimitOptions.m_YAxis.m_IsLimited)
            {
                m_PxD6Joint->setMotion(PxD6Axis::eY, PxD6Motion::eLIMITED);
                m_PxD6Joint->setLinearLimit(PxD6Axis::eY, PxJointLinearLimitPair(
                    PxTolerancesScale(), 
                    m_LimitOptions.m_YAxis.m_LowerLimit, 
                    m_LimitOptions.m_YAxis.m_UpperLimit));
            }
            else
            {
                m_PxD6Joint->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
            }

            if (m_LimitOptions.m_ZAxis.m_IsLimited)
            {
                m_PxD6Joint->setMotion(PxD6Axis::eZ, PxD6Motion::eLIMITED);
                m_PxD6Joint->setLinearLimit(PxD6Axis::eZ, PxJointLinearLimitPair(
                    PxTolerancesScale(), 
                    m_LimitOptions.m_ZAxis.m_LowerLimit, 
                    m_LimitOptions.m_ZAxis.m_UpperLimit));
            }
            else
            {
                m_PxD6Joint->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
            }

            if (m_LimitOptions.m_Twist.m_IsLimited)
            {
                m_PxD6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
                m_PxD6Joint->setTwistLimit(PxJointAngularLimitPair(
                    m_LimitOptions.m_Twist.m_LowerLimit, 
                    m_LimitOptions.m_Twist.m_UpperLimit));
            }
            else
            {
                m_PxD6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
            }

            if (m_LimitOptions.m_Swing1.m_IsLimited)
            {
                m_PxD6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLIMITED);
                m_PxD6Joint->setSwingLimit(PxJointLimitCone(
                    m_LimitOptions.m_Swing1.m_UpperLimit,
                    m_LimitOptions.m_Swing2.m_UpperLimit));
            }
            else
            {
                m_PxD6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
            }

            if (m_LimitOptions.m_Swing2.m_IsLimited)
            {
                m_PxD6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLIMITED);
            }
            else
            {
                m_PxD6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
            }
        }
        break;
    default:
        break;
    }
}

JointLimitOptions PhysicsJoint::GetJointLimits() const
{
    return m_LimitOptions;
}

physx::PxJoint* PhysicsJoint::GetPxJoint() const
{
    return m_PxJoint;
}

bool PhysicsJoint::CreateFixedJoint()
{
    if (!m_ObjectA || !m_ObjectB)
        return false;

    IRigidBody* objA = static_cast<IRigidBody*>(m_ObjectA.get());
    IRigidBody* objB = static_cast<IRigidBody*>(m_ObjectB.get());

    if (!objA || !objB)
        return false;

    PxRigidActor* actorA = static_cast<PxRigidActor*>(objA->GetNativeActor());
    PxRigidActor* actorB = static_cast<PxRigidActor*>(objB->GetNativeActor());

    if (!actorA || !actorB)
        return false;

    PxTransform pxLocalFrameA = ConvertUtils::ToPx(m_LocalFrameA);
    PxTransform pxLocalFrameB = ConvertUtils::ToPx(m_LocalFrameB);

    PxPhysics* physics = &PxGetPhysics();
    if (!physics)
        return false;

    m_PxFixedJoint = PxFixedJointCreate(*physics, actorA, pxLocalFrameA, actorB, pxLocalFrameB);
    if (!m_PxFixedJoint)
        return false;

    m_PxJoint = m_PxFixedJoint;
    return true;
}

bool PhysicsJoint::CreateDistanceJoint()
{
    if (!m_ObjectA || !m_ObjectB)
        return false;

    IRigidBody* objA = static_cast<IRigidBody*>(m_ObjectA.get());
    IRigidBody* objB = static_cast<IRigidBody*>(m_ObjectB.get());

    if (!objA || !objB)
        return false;

    PxRigidActor* actorA = static_cast<PxRigidActor*>(objA->GetNativeActor());
    PxRigidActor* actorB = static_cast<PxRigidActor*>(objB->GetNativeActor());

    if (!actorA || !actorB)
        return false;

    PxTransform pxLocalFrameA = ConvertUtils::ToPx(m_LocalFrameA);
    PxTransform pxLocalFrameB = ConvertUtils::ToPx(m_LocalFrameB);

    PxPhysics* physics = &PxGetPhysics();
    if (!physics)
        return false;

    m_PxDistanceJoint = PxDistanceJointCreate(*physics, actorA, pxLocalFrameA, actorB, pxLocalFrameB);
    if (!m_PxDistanceJoint)
        return false;

    m_PxJoint = m_PxDistanceJoint;
    return true;
}

bool PhysicsJoint::CreateSphericalJoint()
{
    if (!m_ObjectA || !m_ObjectB)
        return false;

    IRigidBody* objA = static_cast<IRigidBody*>(m_ObjectA.get());
    IRigidBody* objB = static_cast<IRigidBody*>(m_ObjectB.get());

    if (!objA || !objB)
        return false;

    PxRigidActor* actorA = static_cast<PxRigidActor*>(objA->GetNativeActor());
    PxRigidActor* actorB = static_cast<PxRigidActor*>(objB->GetNativeActor());

    if (!actorA || !actorB)
        return false;

    PxTransform pxLocalFrameA = ConvertUtils::ToPx(m_LocalFrameA);
    PxTransform pxLocalFrameB = ConvertUtils::ToPx(m_LocalFrameB);

    physx::PxPhysics* physics = &PxGetPhysics();
    if (!physics)
        return false;

    m_PxSphericalJoint = PxSphericalJointCreate(*physics, actorA, pxLocalFrameA, actorB, pxLocalFrameB);
    if (!m_PxSphericalJoint)
        return false;

    m_PxJoint = m_PxSphericalJoint;
    return true;
}

bool PhysicsJoint::CreateRevoluteJoint()
{
    if (!m_ObjectA || !m_ObjectB)
        return false;

    IRigidBody* objA = static_cast<IRigidBody*>(m_ObjectA.get());
    IRigidBody* objB = static_cast<IRigidBody*>(m_ObjectB.get());

    if (!objA || !objB)
        return false;

    PxRigidActor* actorA = static_cast<PxRigidActor*>(objA->GetNativeActor());
    PxRigidActor* actorB = static_cast<PxRigidActor*>(objB->GetNativeActor());

    if (!actorA || !actorB)
        return false;

    PxTransform pxLocalFrameA = ConvertUtils::ToPx(m_LocalFrameA);
    PxTransform pxLocalFrameB = ConvertUtils::ToPx(m_LocalFrameB);

    PxPhysics* physics = &PxGetPhysics();
    if (!physics)
        return false;

    m_PxRevoluteJoint = PxRevoluteJointCreate(*physics, actorA, pxLocalFrameA, actorB, pxLocalFrameB);
    if (!m_PxRevoluteJoint)
        return false;

    m_PxJoint = m_PxRevoluteJoint;
    return true;
}

bool PhysicsJoint::CreatePrismaticJoint()
{
    if (!m_ObjectA || !m_ObjectB)
        return false;

    IRigidBody* objA = static_cast<IRigidBody*>(m_ObjectA.get());
    IRigidBody* objB = static_cast<IRigidBody*>(m_ObjectB.get());

    if (!objA || !objB)
        return false;

    PxRigidActor* actorA = static_cast<PxRigidActor*>(objA->GetNativeActor());
    PxRigidActor* actorB = static_cast<PxRigidActor*>(objB->GetNativeActor());

    if (!actorA || !actorB)
        return false;

    PxTransform pxLocalFrameA = ConvertUtils::ToPx(m_LocalFrameA);
    PxTransform pxLocalFrameB = ConvertUtils::ToPx(m_LocalFrameB);

    PxPhysics* physics = &PxGetPhysics();
    if (!physics)
        return false;

    m_PxPrismaticJoint = PxPrismaticJointCreate(*physics, actorA, pxLocalFrameA, actorB, pxLocalFrameB);
    if (!m_PxPrismaticJoint)
        return false;

    m_PxJoint = m_PxPrismaticJoint;
    return true;
}

bool PhysicsJoint::CreateD6Joint()
{
    if (!m_ObjectA || !m_ObjectB)
        return false;

    IRigidBody* objA = static_cast<IRigidBody*>(m_ObjectA.get());
    IRigidBody* objB = static_cast<IRigidBody*>(m_ObjectB.get());

    if (!objA || !objB)
        return false;

    PxRigidActor* actorA = static_cast<PxRigidActor*>(objA->GetNativeActor());
    PxRigidActor* actorB = static_cast<PxRigidActor*>(objB->GetNativeActor());

    if (!actorA || !actorB)
        return false;

    PxTransform pxLocalFrameA = ConvertUtils::ToPx(m_LocalFrameA);
    PxTransform pxLocalFrameB = ConvertUtils::ToPx(m_LocalFrameB);

    PxPhysics* physics = &PxGetPhysics();
    if (!physics)
        return false;

    m_PxD6Joint = PxD6JointCreate(*physics, actorA, pxLocalFrameA, actorB, pxLocalFrameB);
    if (!m_PxD6Joint)
        return false;

    m_PxJoint = m_PxD6Joint;
    return true;
}

void PhysicsJoint::UpdateJointPose()
{
    if (!m_PxJoint)
        return;

    PxTransform pxLocalFrameA = ConvertUtils::ToPx(m_LocalFrameA);
    PxTransform pxLocalFrameB = ConvertUtils::ToPx(m_LocalFrameB);

    m_PxJoint->setLocalPose(PxJointActorIndex::eACTOR0, pxLocalFrameA);
    m_PxJoint->setLocalPose(PxJointActorIndex::eACTOR1, pxLocalFrameB);
} 