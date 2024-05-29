#include "Physics/PhysicsObject.h"
#include "PxPhysicsAPI.h"
#include "PxRigidDynamic.h"
#include "Physics/ColliderGeometry.h"
#include "Physics/PhysicsEngine.h"
#include "Physics/PhysXUtils.h"
using namespace physx;
class ShapeFactory
{
public:
	static physx::PxShape *CreateShape(const IColliderGeometry *cGeo, const IPhysicsMaterial *material)
	{
		if (cGeo == nullptr)
			return nullptr;
		physx::PxShape *shape = nullptr;
		physx::PxPhysics *physics = GetPxPhysics();
		physx::PxMaterial *pxMaterial = reinterpret_cast<physx::PxMaterial *>(reinterpret_cast<char *>(material) + material->GetOffset());
		switch (cGeo->GetType())
		{
		case CollierGeometryType::Box:
		{
			const BoxColliderGeometry *box = static_cast<const BoxColliderGeometry *>(cGeo);
			const MathLib::HVector3 &halfSize = box->GetHalfSize();
			const MathLib::HVector3 &scale = box->GetScale();
			PxBoxGeometry geometry(halfSize[0] * scale[0], halfSize[1] * scale[1], halfSize[2] * scale[2]);
			shape = physics->createShape(geometry, pxMaterial);
			break;
		}
		case CollierGeometryType::Sphere:
		{
			const SphereColliderGeometry *sphere = static_cast<const SphereColliderGeometry *>(cGeo);
			const MathLib::HReal &radius = sphere->GetRadius();
			const MathLib::HVector3 &scale = sphere->GetScale();
			PxSphereGeometry geometry(radius * scale[0]);
			shape = physics->createShape(geometry, pxMaterial);
			break;
		}
		default:
			break;
		};
	};
};

PhysicsRigidDynamic::PhysicsRigidDynamic(IPhysicsMaterial *material)
{
	m_pRigidDynamic = std::make_unique<physx::PxRigidDynamic>(GetPxPhysics()->createRigidDynamic(PxTransform(PxIdentity)));
	m_Material = std::make_unique<IPhysicsMaterial>(material);
	m_bIsKinematic = false;
	m_Mass = 0.0f;
	m_LinearVelocity = 0.0f;
	m_AngularVelocity = 0.0f;
	m_Transform = MathLib::HTransform3::Identity();
}

void PhysicsRigidDynamic::Update()
{
	if (m_pRigidDynamic == nullptr)
		return;
}

void PhysicsRigidDynamic::SetKinematic(bool bKinematic)
{
	if (m_pRigidDynamic == nullptr)
		return;
	m_pRigidDynamic->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, bKinematic);
	m_bIsKinematic = bKinematic;
}

void PhysicsRigidDynamic::AddColliderGeometry(IColliderGeometry *colliderGeometry, const MathLib::HTransform3 &localTrans)
{
	if (m_pRigidDynamic == nullptr)
		return;
	physx::PxShape *shape = ShapeFactory::CreateShape(colliderGeometry, m_Material.get());
	if (shape == nullptr)
		return;
	shape->setLocalPose(ConvertUtils::ToPxTransform(localTrans));
	m_pRigidDynamic->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*m_pRigidDynamic, m_Material->GetDensity());
}

uint32_t PhysicsRigidDynamic::GetOffset() const
{
	return PX_CUT_OFF(this, m_pRigidDynamic.get());
}

void PhysicsRigidDynamic::SetTransform(const MathLib::HTransform3 &transform)
{
	if (m_pRigidDynamic == nullptr)
		return;
	m_pRigidDynamic->setGlobalPose(ConvertUtils::ToPxTransform(transform));
	m_Transform = transform;
}

void PhysicsRigidDynamic::SetAngularDamping(const MathLib::HReal &damping)
{
	if (m_pRigidDynamic == nullptr)
		return;
	m_pRigidDynamic->setAngularDamping(damping);
	m_AngularDamping = damping;
}

void PhysicsRigidDynamic::SetLinearVelocity(const MathLib::HVector3 &velocity)
{
	if (m_pRigidDynamic == nullptr)
		return;
	m_pRigidDynamic->setLinearVelocity(ConvertUtils::ToPxVec3(velocity));
	m_LinearVelocity = velocity;
}