#include "PhysicsJoint.h"
#include "PhysicsRigid.h"
#include <PxPhysicsAPI.h>
#include "Utility/PhysXUtils.h"
#include "Utility/PhysX/JointUtils.h"

using namespace physx;

template<typename TJoint>
using PxJointCreateFn = std::function<TJoint*(PxPhysics&, PxRigidActor*, const PxTransform&, PxRigidActor*, const PxTransform&)>;

template<typename TJoint>
inline TJoint* CreatePxJoint(IPhysicsObject* objectA, IPhysicsObject* objectB, const MathLib::HTransform3& localFrameA, const MathLib::HTransform3& localFrameB, PxJointCreateFn<TJoint> createFn)
{
    if (!objectA || !objectB)
        return nullptr;

    IRigidBody* objA = static_cast<IRigidBody*>(objectA);
    IRigidBody* objB = static_cast<IRigidBody*>(objectB);

    if (!objA || !objB)
        return nullptr;

    PxRigidActor* actorA = static_cast<PxRigidActor*>(objA->GetNativeActor());
    PxRigidActor* actorB = static_cast<PxRigidActor*>(objB->GetNativeActor());

    if (!actorA || !actorB)
        return nullptr;

    PxTransform pxLocalFrameA = ConvertUtils::ToPx(localFrameA);
    PxTransform pxLocalFrameB = ConvertUtils::ToPx(localFrameB);

    PxPhysics* physics = &PxGetPhysics();
    if (!physics)
        return nullptr;

    return createFn(PxGetPhysics(), actorA, pxLocalFrameA, actorB, pxLocalFrameB);
}

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
        m_PxFixedJoint = CreatePxJoint<PxFixedJoint>(m_ObjectA.get(), m_ObjectB.get(), m_LocalFrameA, m_LocalFrameB, PxFixedJointCreate);
        m_PxJoint = m_PxFixedJoint;
        break;
    case JointType::DISTANCE:
        m_PxDistanceJoint = CreatePxJoint<PxDistanceJoint>(m_ObjectA.get(), m_ObjectB.get(), m_LocalFrameA, m_LocalFrameB, PxDistanceJointCreate);
        m_PxJoint = m_PxDistanceJoint;
        break;
    case JointType::SPHERICAL:
        m_PxSphericalJoint = CreatePxJoint<PxSphericalJoint>(m_ObjectA.get(), m_ObjectB.get(), m_LocalFrameA, m_LocalFrameB, PxSphericalJointCreate);
        m_PxJoint = m_PxSphericalJoint;
        break;
    case JointType::REVOLUTE:
        m_PxRevoluteJoint = CreatePxJoint<PxRevoluteJoint>(m_ObjectA.get(), m_ObjectB.get(), m_LocalFrameA, m_LocalFrameB, PxRevoluteJointCreate);
        m_PxJoint = m_PxRevoluteJoint;
        break;
    case JointType::REVOLUTE2:
        m_PxRevoluteJoint = CreatePxJoint<PxRevoluteJoint>(m_ObjectA.get(), m_ObjectB.get(), m_LocalFrameA, m_LocalFrameB, PxRevoluteJointCreate);
        m_PxJoint = m_PxRevoluteJoint;
        break;
    case JointType::PRISMATIC:
        m_PxPrismaticJoint = CreatePxJoint<PxPrismaticJoint>(m_ObjectA.get(), m_ObjectB.get(), m_LocalFrameA, m_LocalFrameB, PxPrismaticJointCreate);
        m_PxJoint = m_PxPrismaticJoint;
        break;
    case JointType::D6:
        m_PxD6Joint = CreatePxJoint<PxD6Joint>(m_ObjectA.get(), m_ObjectB.get(), m_LocalFrameA, m_LocalFrameB, PxD6JointCreate);
        m_PxJoint = m_PxD6Joint;
        break;
    case JointType::HINGE:
        CreateHingeJoint();
        break;
    case JointType::GEAR:
        m_PxGearJoint = CreatePxJoint<PxGearJoint>(m_ObjectA.get(), m_ObjectB.get(), m_LocalFrameA, m_LocalFrameB, PxGearJointCreate);
        m_PxJoint = m_PxGearJoint;
        break;
    case JointType::RACK_AND_PINION:
        m_PxRackAndPinionJoint = CreatePxJoint<PxRackAndPinionJoint>(m_ObjectA.get(), m_ObjectB.get(), m_LocalFrameA, m_LocalFrameB, PxRackAndPinionJointCreate);
        m_PxJoint = m_PxRackAndPinionJoint;
        break;
    case JointType::CHAIN:
        CreateChainJoint();
        break;
    case JointType::PORTAL:
        CreatePortalJoint();
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
    case JointType::SPHERICAL:
        if (m_PxSphericalJoint)
        {
            if (m_LimitOptions.m_Swing1.m_IsLimited && m_LimitOptions.m_Swing2.m_IsLimited)
            {
                PxJointLimitCone limitCone(
                    m_LimitOptions.m_Swing1.m_UpperLimit, 
                    m_LimitOptions.m_Swing2.m_UpperLimit,
                    PxSpring(m_LimitOptions.m_Swing1.m_Stiffness, m_LimitOptions.m_Swing1.m_Damping));
                
                m_PxSphericalJoint->setLimitCone(limitCone);
                m_PxSphericalJoint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
            }
            else
            {
                m_PxSphericalJoint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, false);
            }
        }
        break;
    case JointType::FIXED:
        if (m_PxFixedJoint)
        {
            m_PxFixedJoint->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
            m_PxFixedJoint->setConstraintFlag(PxConstraintFlag::eDISABLE_PREPROCESSING, true);
        }
        break;
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
    case JointType::REVOLUTE2:
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
            
            m_PxRevoluteJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, true);
            m_PxRevoluteJoint->setDriveVelocity(0.0f);
            m_PxRevoluteJoint->setDriveForceLimit(1000.0f);
            
            m_PxRevoluteJoint->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, true);
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
                    m_LimitOptions.m_Twist.m_UpperLimit, 
                    PxSpring(m_LimitOptions.m_Twist.m_Stiffness, m_LimitOptions.m_Twist.m_Damping)));
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
                    m_LimitOptions.m_Swing2.m_UpperLimit, 
                    PxSpring(m_LimitOptions.m_Swing1.m_Stiffness, m_LimitOptions.m_Swing1.m_Damping)));
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
    case JointType::HINGE:
        if (m_PxHingeJoint)
        {
            if (m_LimitOptions.m_Twist.m_IsLimited)
            {
                m_PxHingeJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
                m_PxHingeJoint->setTwistLimit(PxJointAngularLimitPair(
                    m_LimitOptions.m_Twist.m_LowerLimit, 
                    m_LimitOptions.m_Twist.m_UpperLimit, 
                    PxSpring(m_LimitOptions.m_Twist.m_Stiffness, m_LimitOptions.m_Twist.m_Damping)));
            }
            else
            {
                m_PxHingeJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
            }
        }
        break;
    case JointType::GEAR:
        if (m_PxGearJoint && m_PxGearJoint->is<PxD6Joint>())
        {
            PxD6Joint* d6Joint = static_cast<PxD6Joint*>(m_PxGearJoint);
            
            if (m_LimitOptions.m_Twist.m_IsLimited)
            {
                d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
                d6Joint->setTwistLimit(PxJointAngularLimitPair(
                    m_LimitOptions.m_Twist.m_LowerLimit, 
                    m_LimitOptions.m_Twist.m_UpperLimit, 
                    PxSpring(m_LimitOptions.m_Twist.m_Stiffness, m_LimitOptions.m_Twist.m_Damping)));
            }
            else
            {
                d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
            }
        }
        break;
    case JointType::RACK_AND_PINION:
        if (m_PxRackAndPinionJoint && m_PxRackAndPinionJoint->is<PxD6Joint>())
        {
            PxD6Joint* d6Joint = static_cast<PxD6Joint*>(m_PxRackAndPinionJoint);
            
            if (m_LimitOptions.m_ZAxis.m_IsLimited)
            {
                d6Joint->setMotion(PxD6Axis::eZ, PxD6Motion::eLIMITED);
                d6Joint->setLinearLimit(PxD6Axis::eZ, PxJointLinearLimitPair(
                    PxTolerancesScale(), 
                    m_LimitOptions.m_ZAxis.m_LowerLimit, 
                    m_LimitOptions.m_ZAxis.m_UpperLimit));
            }
            else
            {
                d6Joint->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
            }
            
            if (m_LimitOptions.m_Twist.m_IsLimited)
            {
                d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eLIMITED);
                d6Joint->setTwistLimit(PxJointAngularLimitPair(
                    m_LimitOptions.m_Twist.m_LowerLimit, 
                    m_LimitOptions.m_Twist.m_UpperLimit, 
                    PxSpring(m_LimitOptions.m_Twist.m_Stiffness, m_LimitOptions.m_Twist.m_Damping)));
            }
            else
            {
                d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
            }
        }
        break;
    case JointType::CHAIN:
        if (m_PxChainJoint && m_PxChainJoint->is<PxSphericalJoint>())
        {
            PxSphericalJoint* sphericalJoint = static_cast<PxSphericalJoint*>(m_PxChainJoint);
            
            if (m_LimitOptions.m_Swing1.m_IsLimited && m_LimitOptions.m_Swing2.m_IsLimited)
            {
                PxJointLimitCone limitCone(
                    m_LimitOptions.m_Swing1.m_UpperLimit, 
                    m_LimitOptions.m_Swing2.m_UpperLimit, 
                    PxSpring(m_LimitOptions.m_Swing1.m_Stiffness, m_LimitOptions.m_Swing1.m_Damping));
                
                sphericalJoint->setLimitCone(limitCone);
                sphericalJoint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);
            }
            else
            {
                sphericalJoint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, false);
            }
        }
        break;
    case JointType::PORTAL:
        if (m_PxPortalJoint && m_PxPortalJoint->is<PxD6Joint>())
        {
            PxD6Joint* d6Joint = static_cast<PxD6Joint*>(m_PxPortalJoint);
            
            d6Joint->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
            d6Joint->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
            d6Joint->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
            d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
            d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
            d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);
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

bool PhysicsJoint::CreateHingeJoint()
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

    m_PxHingeJoint = PxD6JointCreate(*physics, actorA, pxLocalFrameA, actorB, pxLocalFrameB);
    if (!m_PxHingeJoint)
        return false;

    m_PxHingeJoint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
    m_PxHingeJoint->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
    m_PxHingeJoint->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
    m_PxHingeJoint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
    m_PxHingeJoint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLOCKED);
    m_PxHingeJoint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);

    m_PxJoint = m_PxHingeJoint;
    return true;
}

bool PhysicsJoint::CreateGearJoint()
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

    PxD6Joint* d6Joint = CreatePxJoint<PxD6Joint>(m_ObjectA.get(), m_ObjectB.get(), m_LocalFrameA, m_LocalFrameB, PxD6JointCreate);
    if (!d6Joint)
        return false;

    d6Joint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
    d6Joint->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
    d6Joint->setMotion(PxD6Axis::eZ, PxD6Motion::eLOCKED);
    d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
    d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLOCKED);
    d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);

    m_PxGearJoint = d6Joint;
    m_PxJoint = m_PxGearJoint;
    return true;
}

bool PhysicsJoint::CreateRackAndPinionJoint()
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

    PxD6Joint* d6Joint = CreatePxJoint<PxD6Joint>(m_ObjectA.get(), m_ObjectB.get(), m_LocalFrameA, m_LocalFrameB, PxD6JointCreate);
    if (!d6Joint)
        return false;

    d6Joint->setMotion(PxD6Axis::eX, PxD6Motion::eLOCKED);
    d6Joint->setMotion(PxD6Axis::eY, PxD6Motion::eLOCKED);
    d6Joint->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
    d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
    d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eLOCKED);
    d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eLOCKED);

    m_PxRackAndPinionJoint = d6Joint;
    m_PxJoint = m_PxRackAndPinionJoint;
    return true;
}

bool PhysicsJoint::CreateChainJoint()
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

    PxSphericalJoint* sphericalJoint = CreatePxJoint<PxSphericalJoint>(m_ObjectA.get(), m_ObjectB.get(), m_LocalFrameA, m_LocalFrameB, PxSphericalJointCreate);
    if (!sphericalJoint)
        return false;

    PxJointLimitCone limitCone(MathLib::H_PI / 6.0f, MathLib::H_PI / 6.0f);
    sphericalJoint->setLimitCone(limitCone);
    sphericalJoint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, true);

    m_PxChainJoint = sphericalJoint;
    m_PxJoint = m_PxChainJoint;
    return true;
}

bool PhysicsJoint::CreatePortalJoint()
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

    PxD6Joint* d6Joint = PxD6JointCreate(*physics, actorA, pxLocalFrameA, actorB, pxLocalFrameB);
    if (!d6Joint)
        return false;

    d6Joint->setMotion(PxD6Axis::eX, PxD6Motion::eFREE);
    d6Joint->setMotion(PxD6Axis::eY, PxD6Motion::eFREE);
    d6Joint->setMotion(PxD6Axis::eZ, PxD6Motion::eFREE);
    d6Joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
    d6Joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
    d6Joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

    m_PxPortalJoint = d6Joint;
    m_PxJoint = m_PxPortalJoint;
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

void PhysicsJoint::SetDrive(JointAxis axis, const JointDriveSettings& driveSettings)
{
    switch (axis)
    {
    case JointAxis::X:
        m_DriveConfig.m_LinearX = driveSettings;
        break;
    case JointAxis::Y:
        m_DriveConfig.m_LinearY = driveSettings;
        break;
    case JointAxis::Z:
        m_DriveConfig.m_LinearZ = driveSettings;
        break;
    case JointAxis::TWIST:
        m_DriveConfig.m_AngularX = driveSettings;
        break;
    case JointAxis::SWING1:
        m_DriveConfig.m_AngularY = driveSettings;
        break;
    case JointAxis::SWING2:
        m_DriveConfig.m_AngularZ = driveSettings;
        break;
    case JointAxis::SLERP:
        m_DriveConfig.m_SlerpDrive = driveSettings;
        break;
    }

    UpdateJointDrive();
}

JointDriveSettings PhysicsJoint::GetDrive(JointAxis axis) const
{
    switch (axis)
    {
    case JointAxis::X:
        return m_DriveConfig.m_LinearX;
    case JointAxis::Y:
        return m_DriveConfig.m_LinearY;
    case JointAxis::Z:
        return m_DriveConfig.m_LinearZ;
    case JointAxis::TWIST:
        return m_DriveConfig.m_AngularX;
    case JointAxis::SWING1:
        return m_DriveConfig.m_AngularY;
    case JointAxis::SWING2:
        return m_DriveConfig.m_AngularZ;
    case JointAxis::SLERP:
        return m_DriveConfig.m_SlerpDrive;
    default:
        return JointDriveSettings();
    }
}

void PhysicsJoint::SetDriveConfig(const JointDriveConfig& driveConfig)
{
    m_DriveConfig = driveConfig;
    UpdateJointDrive();
}

JointDriveConfig PhysicsJoint::GetDriveConfig() const
{
    return m_DriveConfig;
}

void PhysicsJoint::SetDriveVelocity(JointAxis axis, MathLib::HReal velocity)
{
    JointDriveSettings* driveSettings = nullptr;
    
    switch (axis)
    {
    case JointAxis::X:
        driveSettings = &m_DriveConfig.m_LinearX;
        break;
    case JointAxis::Y:
        driveSettings = &m_DriveConfig.m_LinearY;
        break;
    case JointAxis::Z:
        driveSettings = &m_DriveConfig.m_LinearZ;
        break;
    case JointAxis::TWIST:
        driveSettings = &m_DriveConfig.m_AngularX;
        break;
    case JointAxis::SWING1:
        driveSettings = &m_DriveConfig.m_AngularY;
        break;
    case JointAxis::SWING2:
        driveSettings = &m_DriveConfig.m_AngularZ;
        break;
    case JointAxis::SLERP:
        driveSettings = &m_DriveConfig.m_SlerpDrive;
        break;
    }

    if (driveSettings)
    {
        driveSettings->m_TargetVelocity = velocity;
        driveSettings->m_DriveType = JointDriveType::VELOCITY;
        driveSettings->m_Enabled = (velocity != 0.0f);

        UpdateJointDrive();
    }
}

void PhysicsJoint::SetDrivePosition(JointAxis axis, MathLib::HReal position)
{
    JointDriveSettings* driveSettings = nullptr;
    
    switch (axis)
    {
    case JointAxis::X:
        driveSettings = &m_DriveConfig.m_LinearX;
        break;
    case JointAxis::Y:
        driveSettings = &m_DriveConfig.m_LinearY;
        break;
    case JointAxis::Z:
        driveSettings = &m_DriveConfig.m_LinearZ;
        break;
    case JointAxis::TWIST:
        driveSettings = &m_DriveConfig.m_AngularX;
        break;
    case JointAxis::SWING1:
        driveSettings = &m_DriveConfig.m_AngularY;
        break;
    case JointAxis::SWING2:
        driveSettings = &m_DriveConfig.m_AngularZ;
        break;
    case JointAxis::SLERP:
        driveSettings = &m_DriveConfig.m_SlerpDrive;
        break;
    }

    if (driveSettings)
    {
        driveSettings->m_TargetPosition = position;
        driveSettings->m_DriveType = JointDriveType::POSITION;
        driveSettings->m_Enabled = true;
        UpdateJointDrive();
    }
}

void PhysicsJoint::SetDriveForceLimit(JointAxis axis, MathLib::HReal forceLimit)
{
    JointDriveSettings* driveSettings = nullptr;
    
    switch (axis)
    {
    case JointAxis::X:
        driveSettings = &m_DriveConfig.m_LinearX;
        break;
    case JointAxis::Y:
        driveSettings = &m_DriveConfig.m_LinearY;
        break;
    case JointAxis::Z:
        driveSettings = &m_DriveConfig.m_LinearZ;
        break;
    case JointAxis::TWIST:
        driveSettings = &m_DriveConfig.m_AngularX;
        break;
    case JointAxis::SWING1:
        driveSettings = &m_DriveConfig.m_AngularY;
        break;
    case JointAxis::SWING2:
        driveSettings = &m_DriveConfig.m_AngularZ;
        break;
    case JointAxis::SLERP:
        driveSettings = &m_DriveConfig.m_SlerpDrive;
        break;
    }

    if (driveSettings)
    {
        driveSettings->m_ForceLimit = forceLimit;
        UpdateJointDrive();
    }
}

void PhysicsJoint::SetDriveEnabled(JointAxis axis, bool enabled)
{
    JointDriveSettings* driveSettings = nullptr;
    
    switch (axis)
    {
    case JointAxis::X:
        driveSettings = &m_DriveConfig.m_LinearX;
        break;
    case JointAxis::Y:
        driveSettings = &m_DriveConfig.m_LinearY;
        break;
    case JointAxis::Z:
        driveSettings = &m_DriveConfig.m_LinearZ;
        break;
    case JointAxis::TWIST:
        driveSettings = &m_DriveConfig.m_AngularX;
        break;
    case JointAxis::SWING1:
        driveSettings = &m_DriveConfig.m_AngularY;
        break;
    case JointAxis::SWING2:
        driveSettings = &m_DriveConfig.m_AngularZ;
        break;
    case JointAxis::SLERP:
        driveSettings = &m_DriveConfig.m_SlerpDrive;
        break;
    }

    if (driveSettings)
    {
        driveSettings->m_Enabled = enabled;
        UpdateJointDrive();
    }
}

void PhysicsJoint::UpdateJointDrive()
{
    if (!m_PxJoint)
        return;

    switch (m_Type)
    {
    case JointType::REVOLUTE:
    case JointType::REVOLUTE2:
        if (m_PxRevoluteJoint)
        {
            const JointDriveSettings& twistDrive = m_DriveConfig.m_AngularX;
            
            m_PxRevoluteJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, twistDrive.m_Enabled);
            
            if (twistDrive.m_Enabled)
            {
                if (twistDrive.m_DriveType == JointDriveType::VELOCITY)
                {
                    m_PxRevoluteJoint->setDriveVelocity(twistDrive.m_TargetVelocity);
                }
                else if (twistDrive.m_DriveType == JointDriveType::POSITION)
                {
                }
                
                m_PxRevoluteJoint->setDriveForceLimit(twistDrive.m_ForceLimit);
                m_PxRevoluteJoint->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES, !twistDrive.m_IsAcceleration);
            }
        }
        break;

    case JointType::PRISMATIC:
        break;

    case JointType::D6:
    case JointType::HINGE:
    case JointType::GEAR:
    case JointType::RACK_AND_PINION:
    case JointType::PORTAL:
        {
            PxD6Joint* d6Joint = nullptr;
            
            if (m_Type == JointType::D6) d6Joint = m_PxD6Joint;
            else if (m_Type == JointType::HINGE) d6Joint = m_PxHingeJoint;
            else if (m_Type == JointType::GEAR) d6Joint = static_cast<PxD6Joint*>(m_PxGearJoint);
            else if (m_Type == JointType::RACK_AND_PINION) d6Joint = static_cast<PxD6Joint*>(m_PxRackAndPinionJoint);
            else if (m_Type == JointType::PORTAL) d6Joint = static_cast<PxD6Joint*>(m_PxPortalJoint);
            
            if (d6Joint)
            {
                if (m_DriveConfig.m_LinearX.m_Enabled)
                {
                    PxD6JointDrive drive(m_DriveConfig.m_LinearX.m_Stiffness, 
                                       m_DriveConfig.m_LinearX.m_Damping, 
                                       m_DriveConfig.m_LinearX.m_ForceLimit, 
                                       m_DriveConfig.m_LinearX.m_IsAcceleration);
                    d6Joint->setDrive(PxD6Drive::eX, drive);
                }

                if (m_DriveConfig.m_LinearY.m_Enabled)
                {
                    PxD6JointDrive drive(m_DriveConfig.m_LinearY.m_Stiffness, 
                                       m_DriveConfig.m_LinearY.m_Damping, 
                                       m_DriveConfig.m_LinearY.m_ForceLimit, 
                                       m_DriveConfig.m_LinearY.m_IsAcceleration);
                    d6Joint->setDrive(PxD6Drive::eY, drive);
                }

                if (m_DriveConfig.m_LinearZ.m_Enabled)
                {
                    PxD6JointDrive drive(m_DriveConfig.m_LinearZ.m_Stiffness, 
                                       m_DriveConfig.m_LinearZ.m_Damping, 
                                       m_DriveConfig.m_LinearZ.m_ForceLimit, 
                                       m_DriveConfig.m_LinearZ.m_IsAcceleration);
                    d6Joint->setDrive(PxD6Drive::eZ, drive);
                }

                if (m_DriveConfig.m_AngularX.m_Enabled)
                {
                    PxD6JointDrive drive(m_DriveConfig.m_AngularX.m_Stiffness, 
                                       m_DriveConfig.m_AngularX.m_Damping, 
                                       m_DriveConfig.m_AngularX.m_ForceLimit, 
                                       m_DriveConfig.m_AngularX.m_IsAcceleration);
                    d6Joint->setDrive(PxD6Drive::eTWIST, drive);
                }

                if (m_DriveConfig.m_AngularY.m_Enabled)
                {
                    PxD6JointDrive drive(m_DriveConfig.m_AngularY.m_Stiffness, 
                                       m_DriveConfig.m_AngularY.m_Damping, 
                                       m_DriveConfig.m_AngularY.m_ForceLimit, 
                                       m_DriveConfig.m_AngularY.m_IsAcceleration);
                    d6Joint->setDrive(PxD6Drive::eSWING, drive);
                }

                if (m_DriveConfig.m_AngularZ.m_Enabled)
                {
                    PxD6JointDrive drive(m_DriveConfig.m_AngularZ.m_Stiffness, 
                                       m_DriveConfig.m_AngularZ.m_Damping, 
                                       m_DriveConfig.m_AngularZ.m_ForceLimit, 
                                       m_DriveConfig.m_AngularZ.m_IsAcceleration);
                    d6Joint->setDrive(PxD6Drive::eSWING, drive);
                }

                if (m_DriveConfig.m_SlerpDrive.m_Enabled)
                {
                    PxD6JointDrive drive(m_DriveConfig.m_SlerpDrive.m_Stiffness, 
                                       m_DriveConfig.m_SlerpDrive.m_Damping, 
                                       m_DriveConfig.m_SlerpDrive.m_ForceLimit, 
                                       m_DriveConfig.m_SlerpDrive.m_IsAcceleration);
                    d6Joint->setDrive(PxD6Drive::eSLERP, drive);
                }

                PxTransform driveTransform(PxIdentity);
                driveTransform.p = ConvertUtils::ToPx(m_DriveConfig.m_TargetPosition);
                driveTransform.q = PxQuat(m_DriveConfig.m_TargetOrientation.x(),
                                        m_DriveConfig.m_TargetOrientation.y(),
                                        m_DriveConfig.m_TargetOrientation.z(),
                                        m_DriveConfig.m_TargetOrientation.w());

                d6Joint->setDrivePosition(driveTransform);

                PxVec3 linearVel(m_DriveConfig.m_LinearX.m_TargetVelocity, 
                               m_DriveConfig.m_LinearY.m_TargetVelocity, 
                               m_DriveConfig.m_LinearZ.m_TargetVelocity);
                PxVec3 angularVel(m_DriveConfig.m_AngularX.m_TargetVelocity, 
                                m_DriveConfig.m_AngularY.m_TargetVelocity, 
                                m_DriveConfig.m_AngularZ.m_TargetVelocity);

                d6Joint->setDriveVelocity(linearVel, angularVel);
            }
        }
        break;

    default:
        break;
    }
} 