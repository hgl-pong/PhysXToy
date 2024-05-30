#include "Physics/PhysicsObject.h"
#include "PxPhysicsAPI.h"
#include "PxRigidDynamic.h"
#include "Physics/ColliderGeometry.h"
#include "Physics/PhysicsEngine.h"
#include "Physics/PhysicsMaterial.h"
#include "PhysXUtils.h"
using namespace physx;
class ShapeFactory
{
public:
	static physx::PxShape *CreateShape(const IColliderGeometry *cGeo, IPhysicsMaterial *material)
	{
		if (cGeo == nullptr)
			return nullptr;
		physx::PxShape *shape = nullptr;
		physx::PxPhysics* physics = &PxGetPhysics();
		const physx::PxMaterial *pxMaterial = *reinterpret_cast<physx::PxMaterial**>(reinterpret_cast<char *>(material) + material->GetOffset());
		switch (cGeo->GetType())
		{
		case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
		{
			const BoxColliderGeometry *box = static_cast<const BoxColliderGeometry *>(cGeo);
			const MathLib::HVector3 &halfSize = box->GetHalfSize();
			const MathLib::HVector3 &scale = box->GetScale();
			PxBoxGeometry geometry(halfSize[0] * scale[0], halfSize[1] * scale[1], halfSize[2] * scale[2]);
			shape = physics->createShape(geometry, *pxMaterial);
			break;
		}
		case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
		{
			const SphereColliderGeometry *sphere = static_cast<const SphereColliderGeometry *>(cGeo);
			const MathLib::HReal &radius = sphere->GetRadius();
			const MathLib::HVector3 &scale = sphere->GetScale();
			PxSphereGeometry geometry(radius * scale[0]);
			shape = physics->createShape(geometry, *pxMaterial);
			break;
		}
		case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
{
			shape = physics->createShape(PxPlaneGeometry(), *pxMaterial);
			break;
		}
		default:
			break;
		};
		return shape;
	};
};

PhysicsRigidDynamic::PhysicsRigidDynamic(IPhysicsMaterial *material)
{
	m_RigidDynamic = PxGetPhysics().createRigidDynamic(PxTransform(PxIdentity));
	m_Material = material;
	m_bIsKinematic = false;
	m_Mass = 0.0f;
	m_LinearVelocity.setZero();
	m_AngularVelocity.setZero();
	m_Transform.setIdentity();
m_Type = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
}

PhysicsRigidDynamic::~PhysicsRigidDynamic()
{
	PX_RELEASE(m_RigidDynamic);
}

void PhysicsRigidDynamic::Update()
{
	if (m_RigidDynamic == nullptr)
		return;
}

void PhysicsRigidDynamic::SetKinematic(bool bKinematic)
{
	if (m_RigidDynamic == nullptr)
		return;
	m_RigidDynamic->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, bKinematic);
	m_bIsKinematic = bKinematic;
}

void PhysicsRigidDynamic::AddColliderGeometry(IColliderGeometry *colliderGeometry, const MathLib::HTransform3 &localTrans)
{
	if (m_RigidDynamic == nullptr)
		return;
	physx::PxShape *shape = ShapeFactory::CreateShape(colliderGeometry, m_Material);
	if (shape == nullptr)
		return;
	shape->setLocalPose(ConvertUtils::ToPxTransform(localTrans));
	m_RigidDynamic->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*m_RigidDynamic, m_Material->GetDensity());
}

size_t PhysicsRigidDynamic::GetOffset() const
{
	return offsetof(PhysicsRigidDynamic, m_RigidDynamic);
}

void PhysicsRigidDynamic::SetTransform(const MathLib::HTransform3 &transform)
{
	if (m_RigidDynamic == nullptr)
		return;
	m_RigidDynamic->setGlobalPose(ConvertUtils::ToPxTransform(transform));
	m_Transform = transform;
}

void PhysicsRigidDynamic::SetAngularDamping(const MathLib::HReal &damping)
{
	if (m_RigidDynamic == nullptr)
		return;
	m_RigidDynamic->setAngularDamping(damping);
	m_AngularDamping = damping;
}

void PhysicsRigidDynamic::SetLinearVelocity(const MathLib::HVector3 &velocity)
{
	if (m_RigidDynamic == nullptr)
		return;
	m_RigidDynamic->setLinearVelocity(ConvertUtils::ToPxVec3(velocity));
	m_LinearVelocity = velocity;
}

/////////////////RigidStatic////////////////////////
PhysicsRigidStatic::PhysicsRigidStatic(IPhysicsMaterial *material)
{
	m_RigidStatic = PxGetPhysics().createRigidStatic(PxTransform(PxIdentity));
	m_Material = material;
	m_Transform.setIdentity();
	m_Type = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
}

PhysicsRigidStatic::~PhysicsRigidStatic()
{
	PX_RELEASE(m_RigidStatic);
}

void PhysicsRigidStatic::AddColliderGeometry(IColliderGeometry *colliderGeometry, const MathLib::HTransform3 &localTrans)
{
	if (m_RigidStatic == nullptr)
		return;
	physx::PxShape *shape = ShapeFactory::CreateShape(colliderGeometry, m_Material);
	if (shape == nullptr)
		return;
	if(colliderGeometry->GetType()==CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE)
	{
		const PlaneColliderGeometry *plane = static_cast<const PlaneColliderGeometry *>(colliderGeometry);
		const MathLib::HVector3 &normal = plane->GetNormal();
		const MathLib::HReal &distance = plane->GetDistance();
		auto trans = PxTransformFromPlaneEquation(PxPlane(normal[0], normal[1], normal[2], distance));
		shape->setLocalPose(ConvertUtils::ToPxTransform(localTrans).transform(trans));
	}
	else
	shape->setLocalPose(ConvertUtils::ToPxTransform(localTrans));
	m_RigidStatic->attachShape(*shape);
}

size_t PhysicsRigidStatic::GetOffset() const
{
	return offsetof(PhysicsRigidStatic, m_RigidStatic);
}

void PhysicsRigidStatic::SetTransform(const MathLib::HTransform3 &transform)
{
	if (m_RigidStatic == nullptr)
		return;
	m_RigidStatic->setGlobalPose(ConvertUtils::ToPxTransform(transform));
m_Transform = transform;
}