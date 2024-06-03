#include "PhysicsObject.h"
#include "PxPhysicsAPI.h"
#include "PxRigidDynamic.h"
#include "ColliderGeometry.h"
#include "PhysicsMaterial.h"
#include "Utility/PhysXUtils.h"
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
		const PhysXPtr<physx::PxMaterial>* pxMaterial = reinterpret_cast<const PhysXPtr<physx::PxMaterial>*>(reinterpret_cast<char *>(material) + material->GetOffset());
		switch (cGeo->GetType())
		{
		case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
		{
			const BoxColliderGeometry *box = static_cast<const BoxColliderGeometry *>(cGeo);
			const MathLib::HVector3 &halfSize = box->GetHalfSize();
			const MathLib::HVector3 &scale = box->GetScale();
			PxBoxGeometry geometry(halfSize[0] * scale[0], halfSize[1] * scale[1], halfSize[2] * scale[2]);
			shape = physics->createShape(geometry, *pxMaterial->get());
			break;
		}
		case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
		{
			const SphereColliderGeometry *sphere = static_cast<const SphereColliderGeometry *>(cGeo);
			const MathLib::HReal &radius = sphere->GetRadius();
			const MathLib::HVector3 &scale = sphere->GetScale();
			PxSphereGeometry geometry(radius * scale[0]);
			shape = physics->createShape(geometry, *pxMaterial->get());
			break;
		}
		case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
{
			shape = physics->createShape(PxPlaneGeometry(), *pxMaterial->get());
			break;
		}
		case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
		{
			const CapsuleColliderGeometry *capsule = static_cast<const CapsuleColliderGeometry *>(cGeo);
			const MathLib::HReal &radius = capsule->GetRadius();
			const MathLib::HReal &halfHeight = capsule->GetHalfHeight();
			const MathLib::HVector3 &scale = capsule->GetScale();
			PxCapsuleGeometry geometry(radius * scale[0], halfHeight * scale[0]);
			shape = physics->createShape(geometry, *pxMaterial->get());
			break;
		}
		case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
		{
			const TriangleMeshColliderGeometry *triangleMesh = static_cast<const TriangleMeshColliderGeometry *>(cGeo);
			const std::vector<MathLib::HVector3>& vertices = triangleMesh->GetVertices();
			const std::vector<uint32_t>& indices = triangleMesh->GetIndices();
			PxTriangleMesh *mesh = PhysXConstructTools::CreatePxTriangleMesh<true>(vertices.size(), vertices.data(), indices.size()/3, indices.data());
			const MathLib::HVector3& scale = triangleMesh->GetScale();
			PxTriangleMeshGeometry geometry(mesh ,PxMeshScale(ConvertUtils::ToPxVec3(scale)));
			shape = physics->createShape(geometry, *pxMaterial->get());
			PX_RELEASE(mesh);
			break;
		}
		case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
		{
			const ConvexMeshColliderGeometry *convexMesh = static_cast<const ConvexMeshColliderGeometry *>(cGeo);
			const std::vector<MathLib::HVector3>& vertices = convexMesh->GetVertices();
			PxConvexMesh *mesh = PhysXConstructTools::CreatePxConvexMesh<true, 256>(vertices.size(), vertices.data());
			const MathLib::HVector3& scale = convexMesh->GetScale();
			PxConvexMeshGeometry geometry(mesh, PxMeshScale(ConvertUtils::ToPxVec3(scale)));
			shape = physics->createShape(geometry, *pxMaterial->get());
			PX_RELEASE(mesh);
			break;
		}
		default:
			break;
		};
		return shape;
	};
};

PhysicsRigidDynamic::PhysicsRigidDynamic(PhysicsPtr < IPhysicsMaterial >&material)
{
	m_RigidDynamic = make_physx_ptr<PxRigidDynamic>(PxGetPhysics().createRigidDynamic(PxTransform(PxIdentity)));
	m_Material = material;
	m_bIsKinematic = false;
	m_Mass = 0.0f;
	m_LinearVelocity.setZero();
	m_AngularVelocity.setZero();
	m_Transform.setIdentity();
	m_Type = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
}

void PhysicsRigidDynamic::Release()
{
	m_RigidDynamic.reset();
	m_Material.reset();
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

bool PhysicsRigidDynamic::AddColliderGeometry(PhysicsPtr < IColliderGeometry >&colliderGeometry, const MathLib::HTransform3 &localTrans)
{
	if (m_RigidDynamic == nullptr || colliderGeometry == nullptr || colliderGeometry->GetType() == CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH)
		return false;
	physx::PxShape *shape = ShapeFactory::CreateShape(colliderGeometry.get(), m_Material.get());
	if (shape == nullptr)
		return false;
	shape->setLocalPose(ConvertUtils::ToPxTransform(localTrans));
	m_RigidDynamic->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*m_RigidDynamic, m_Material->GetDensity());
	PX_RELEASE(shape);
	m_Mass= m_RigidDynamic->getMass();
	m_ColliderGeometries.push_back(colliderGeometry);
	return true;
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

void PhysicsRigidDynamic::SetAngularVelocity(const MathLib::HVector3& velocity)
{
	if (m_RigidDynamic == nullptr)
		return;
	m_RigidDynamic->setAngularVelocity(ConvertUtils::ToPxVec3(velocity));
	m_AngularVelocity = velocity;
}



/////////////////RigidStatic////////////////////////
PhysicsRigidStatic::PhysicsRigidStatic(PhysicsPtr < IPhysicsMaterial >&material)
{
	m_RigidStatic = make_physx_ptr<PxRigidStatic>(PxGetPhysics().createRigidStatic(PxTransform(PxIdentity)));
	m_Material = material;
	m_Transform.setIdentity();
	m_Type = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
}

void PhysicsRigidStatic::Release()
{
	PX_RELEASE(m_RigidStatic);
}

bool PhysicsRigidStatic::AddColliderGeometry(PhysicsPtr < IColliderGeometry >&colliderGeometry, const MathLib::HTransform3 &localTrans)
{
	if (m_RigidStatic == nullptr || colliderGeometry == nullptr ) 
		return false;
	physx::PxShape *shape = ShapeFactory::CreateShape(colliderGeometry.get(), m_Material.get());
	if (shape == nullptr)
		return false;
	if(colliderGeometry->GetType()==CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE)
	{
		const PlaneColliderGeometry *plane = static_cast<const PlaneColliderGeometry *>(colliderGeometry.get());
		const MathLib::HVector3 &normal = plane->GetNormal();
		const MathLib::HReal &distance = plane->GetDistance();
		auto trans = PxTransformFromPlaneEquation(PxPlane(normal[0], normal[1], normal[2], distance));
		shape->setLocalPose(ConvertUtils::ToPxTransform(localTrans).transform(trans));
	}
	else
	shape->setLocalPose(ConvertUtils::ToPxTransform(localTrans));
	m_RigidStatic->attachShape(*shape);
	PX_RELEASE(shape);
	m_ColliderGeometries.push_back(colliderGeometry);
	return true;
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