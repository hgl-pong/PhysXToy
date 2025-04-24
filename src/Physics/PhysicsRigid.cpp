#include "PhysicsRigid.h"
#include "PxPhysicsAPI.h"
#include "PxRigidDynamic.h"
#include "ColliderGeometry.h"
#include "PhysicsMaterial.h"
#include "Utility/PhysXUtils.h"
#include "Utility/PhysicsUtils.h"
using namespace physx;
class ShapeFactory
{
public:
	static physx::PxShape *CreateShape(const IColliderGeometry *cGeo, IPhysicsMaterial *material)
	{
		if (cGeo == nullptr)
			return nullptr;
		physx::PxShape *shape = nullptr;
		physx::PxPhysics *physics = &PxGetPhysics();
		const PhysXPtr<physx::PxMaterial> *pxMaterial = reinterpret_cast<const PhysXPtr<physx::PxMaterial> *>(reinterpret_cast<char *>(material) + material->GetOffset());
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
			const std::vector<MathLib::HVector3> &vertices = triangleMesh->GetVertices();
			const std::vector<uint32_t> &indices = triangleMesh->GetIndices();
			PxTriangleMesh *mesh = PhysXConstructTools::CreatePxTriangleMesh<true>(vertices.size(), vertices.data(), indices.size() / 3, indices.data());
			const MathLib::HVector3 &scale = triangleMesh->GetScale();
			PxTriangleMeshGeometry geometry(mesh, PxMeshScale(ConvertUtils::ToPx(scale)));
			shape = physics->createShape(geometry, *pxMaterial->get());
			PX_RELEASE(mesh);
			break;
		}
		case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
		{
			const ConvexMeshColliderGeometry *convexMesh = static_cast<const ConvexMeshColliderGeometry *>(cGeo);
			const std::vector<MathLib::HVector3> &vertices = convexMesh->GetVertices();
			PxConvexMesh *mesh = PhysXConstructTools::CreatePxConvexMesh<true, 256>(vertices.size(), vertices.data());
			const MathLib::HVector3 &scale = convexMesh->GetScale();
			PxConvexMeshGeometry geometry(mesh, PxMeshScale(ConvertUtils::ToPx(scale)));
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

PhysicsRigidDynamic::PhysicsRigidDynamic(PhysicsPtr<IPhysicsMaterial> &material)
{
	m_RigidDynamic = make_physx_ptr<PxRigidDynamic>(PxGetPhysics().createRigidDynamic(PxTransform(PxIdentity)));
	m_RigidDynamic->setSolverIterationCounts(PhysicsEngineUtils::GetPhysicsEngine()->GetSolverIterationCount());
	m_Material = material;
	m_bIsKinematic = false;
	m_Mass = 0.0f;
	m_LinearVelocity.setZero();
	m_AngularVelocity.setZero();
	m_AngularDamping = 0.0f;
	m_LinearDamping = 0.0f;
	m_Transform.setIdentity();
	m_BoundingBox.setEmpty();
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
	m_RigidDynamic->setSolverIterationCounts(PhysicsEngineUtils::GetPhysicsEngine()->GetSolverIterationCount());
	m_AngularDamping = m_RigidDynamic->getAngularDamping();
	m_LinearDamping = m_RigidDynamic->getLinearDamping();
	m_LinearVelocity = ConvertUtils::FromPx(m_RigidDynamic->getLinearVelocity());
	m_AngularVelocity = ConvertUtils::FromPx(m_RigidDynamic->getAngularVelocity());
	m_Transform = ConvertUtils::FromPx(m_RigidDynamic->getGlobalPose());
}

void PhysicsRigidDynamic::SetKinematic(bool bKinematic)
{
	if (m_RigidDynamic == nullptr)
		return;
	m_RigidDynamic->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, bKinematic);
	m_bIsKinematic = bKinematic;
}

bool PhysicsRigidDynamic::AddColliderGeometry(PhysicsPtr<IColliderGeometry> &colliderGeometry, const MathLib::HTransform3 &localTrans)
{
	if (m_RigidDynamic == nullptr || colliderGeometry == nullptr || colliderGeometry->GetType() == CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH)
		return false;
	physx::PxShape *shape = ShapeFactory::CreateShape(colliderGeometry.get(), m_Material.get());
	if (shape == nullptr)
		return false;
	shape->setLocalPose(ConvertUtils::ToPx(localTrans));
	m_RigidDynamic->attachShape(*shape);
	PxRigidBodyExt::updateMassAndInertia(*m_RigidDynamic, m_Material->GetDensity());
	PX_RELEASE(shape);
	m_Mass = m_RigidDynamic->getMass();
	m_ColliderGeometries.push_back(colliderGeometry);
	m_ColliderLocalPos.push_back(localTrans);
	m_BoundingBox = ComputeBoundingBox(this);
	return true;
}

bool PhysicsRigidDynamic::RemoveColliderGeometry(PhysicsPtr<IColliderGeometry> &colliderGeometry)
{
	if (m_RigidDynamic == nullptr || colliderGeometry == nullptr)
		return false;

	auto it = std::find(m_ColliderGeometries.begin(), m_ColliderGeometries.end(), colliderGeometry);
	if (it == m_ColliderGeometries.end())
		return false;

	size_t index = std::distance(m_ColliderGeometries.begin(), it);
	PxU32 numShapes = m_RigidDynamic->getNbShapes();
	if (index >= numShapes)
		return false;

	std::vector<PxShape*> shapes(numShapes);
	m_RigidDynamic->getShapes(shapes.data(), numShapes);
	
	m_RigidDynamic->detachShape(*shapes[index]);
	
	if (m_RigidDynamic->getNbShapes() > 0)
		PxRigidBodyExt::updateMassAndInertia(*m_RigidDynamic, m_Material->GetDensity());
	
	m_ColliderGeometries.erase(m_ColliderGeometries.begin() + index);
	m_ColliderLocalPos.erase(m_ColliderLocalPos.begin() + index);
	
	m_BoundingBox = ComputeBoundingBox(this);
	m_Mass = m_RigidDynamic->getMass();
	
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
	m_RigidDynamic->setGlobalPose(ConvertUtils::ToPx(transform));
	m_Transform = transform;
}

void PhysicsRigidDynamic::SetAngularDamping(const MathLib::HReal &damping)
{
	if (m_RigidDynamic == nullptr)
		return;
	m_RigidDynamic->setAngularDamping(damping);
	m_AngularDamping = damping;
}

void PhysicsRigidDynamic::SetLinearDamping(const MathLib::HReal &damping)
{
	if (m_RigidDynamic == nullptr)
		return;
	m_RigidDynamic->setLinearDamping(damping);
	m_LinearDamping = damping;
}

void PhysicsRigidDynamic::SetLinearVelocity(const MathLib::HVector3 &velocity)
{
	if (m_RigidDynamic == nullptr)
		return;
	m_RigidDynamic->setLinearVelocity(ConvertUtils::ToPx(velocity));
	m_LinearVelocity = velocity;
}

void PhysicsRigidDynamic::SetAngularVelocity(const MathLib::HVector3 &velocity)
{
	if (m_RigidDynamic == nullptr)
		return;
	m_RigidDynamic->setAngularVelocity(ConvertUtils::ToPx(velocity));
	m_AngularVelocity = velocity;
}

void PhysicsRigidDynamic::SetMass(const MathLib::HReal &mass)
{
	if (m_RigidDynamic == nullptr || mass <= 0.0f)
		return;
		
	PxRigidBodyExt::setMassAndUpdateInertia(*m_RigidDynamic, mass);
	m_Mass = mass;
}

void PhysicsRigidDynamic::AddForce(const MathLib::HVector3 &force, ForceMode mode)
{
	if (m_RigidDynamic == nullptr)
		return;
		
	PxForceMode::Enum pxMode;
	switch (mode)
	{
	case ForceMode::FORCE:
		pxMode = PxForceMode::eFORCE;
		break;
	case ForceMode::IMPULSE:
		pxMode = PxForceMode::eIMPULSE;
		break;
	case ForceMode::VELOCITY_CHANGE:
		pxMode = PxForceMode::eVELOCITY_CHANGE;
		break;
	case ForceMode::ACCELERATION:
		pxMode = PxForceMode::eACCELERATION;
		break;
	default:
		pxMode = PxForceMode::eFORCE;
	}
	
	m_RigidDynamic->addForce(ConvertUtils::ToPx(force), pxMode);
}

void PhysicsRigidDynamic::AddTorque(const MathLib::HVector3 &torque, ForceMode mode)
{
	if (m_RigidDynamic == nullptr)
		return;
		
	PxForceMode::Enum pxMode;
	switch (mode)
	{
	case ForceMode::FORCE:
		pxMode = PxForceMode::eFORCE;
		break;
	case ForceMode::IMPULSE:
		pxMode = PxForceMode::eIMPULSE;
		break;
	case ForceMode::VELOCITY_CHANGE:
		pxMode = PxForceMode::eVELOCITY_CHANGE;
		break;
	case ForceMode::ACCELERATION:
		pxMode = PxForceMode::eACCELERATION;
		break;
	default:
		pxMode = PxForceMode::eFORCE;
	}
	
	m_RigidDynamic->addTorque(ConvertUtils::ToPx(torque), pxMode);
}

void PhysicsRigidDynamic::AddForceAtLocalPosition(const MathLib::HVector3 &force, const MathLib::HVector3 &pos, ForceMode mode)
{
	if (m_RigidDynamic == nullptr)
		return;
		
	PxForceMode::Enum pxMode;
	switch (mode)
	{
	case ForceMode::FORCE:
		pxMode = PxForceMode::eFORCE;
		break;
	case ForceMode::IMPULSE:
		pxMode = PxForceMode::eIMPULSE;
		break;
	case ForceMode::VELOCITY_CHANGE:
		pxMode = PxForceMode::eVELOCITY_CHANGE;
		break;
	case ForceMode::ACCELERATION:
		pxMode = PxForceMode::eACCELERATION;
		break;
	default:
		pxMode = PxForceMode::eFORCE;
	}
	
	PxRigidBodyExt::addForceAtLocalPos(*m_RigidDynamic, ConvertUtils::ToPx(force), ConvertUtils::ToPx(pos), pxMode);
}

void PhysicsRigidDynamic::AddForceAtPosition(const MathLib::HVector3 &force, const MathLib::HVector3 &pos, ForceMode mode)
{
	if (m_RigidDynamic == nullptr)
		return;
		
	PxForceMode::Enum pxMode;
	switch (mode)
	{
	case ForceMode::FORCE:
		pxMode = PxForceMode::eFORCE;
		break;
	case ForceMode::IMPULSE:
		pxMode = PxForceMode::eIMPULSE;
		break;
	case ForceMode::VELOCITY_CHANGE:
		pxMode = PxForceMode::eVELOCITY_CHANGE;
		break;
	case ForceMode::ACCELERATION:
		pxMode = PxForceMode::eACCELERATION;
		break;
	default:
		pxMode = PxForceMode::eFORCE;
	}
	
	PxRigidBodyExt::addForceAtPos(*m_RigidDynamic, ConvertUtils::ToPx(force), ConvertUtils::ToPx(pos), pxMode);
}

void PhysicsRigidDynamic::ClearForce(bool clearVelocity)
{
	if (m_RigidDynamic == nullptr)
		return;
		
	m_RigidDynamic->clearForce(PxForceMode::eFORCE);
	m_RigidDynamic->clearTorque(PxForceMode::eFORCE);
	
	if (clearVelocity)
	{
		m_RigidDynamic->setLinearVelocity(physx::PxVec3(0.0f, 0.0f, 0.0f));
		m_RigidDynamic->setAngularVelocity(physx::PxVec3(0.0f, 0.0f, 0.0f));
		m_LinearVelocity.setZero();
		m_AngularVelocity.setZero();
	}
}

MathLib::HMatrix3 PhysicsRigidDynamic::GetInertiaTensor() const
{
	if (m_RigidDynamic == nullptr)
		return MathLib::HMatrix3::Identity();
		
	physx::PxVec3 moi = m_RigidDynamic->getMassSpaceInertiaTensor();
	
	MathLib::HMatrix3 inertiaTensor = MathLib::HMatrix3::Zero();
	inertiaTensor(0, 0) = moi.x;
	inertiaTensor(1, 1) = moi.y;
	inertiaTensor(2, 2) = moi.z;
	
	return inertiaTensor;
}

bool PhysicsRigidDynamic::IsSleeping() const
{
	if (m_RigidDynamic == nullptr)
		return false;
	return m_RigidDynamic->isSleeping();
}

void PhysicsRigidDynamic::WakeUp()
{
	if (m_RigidDynamic == nullptr)
		return;
	m_RigidDynamic->wakeUp();
}

void PhysicsRigidDynamic::PutToSleep()
{
	if (m_RigidDynamic == nullptr)
		return;
	m_RigidDynamic->putToSleep();
}

MathLib::HAABBox3D PhysicsRigidDynamic::GetWorldBoundingBox() const
{
	if (m_RigidDynamic == nullptr)
		return MathLib::HAABBox3D();
	MathLib::HAABBox3D box = m_BoundingBox;
	box.transform(m_Transform);
	return box;
}

void PhysicsRigidDynamic::SetMassProperties(const MathLib::HReal& mass, const MathLib::HVector3& centerOfMass, const MathLib::HMatrix3& inertiaTensor)
{
	if (m_RigidDynamic == nullptr)
		return;
	
	physx::PxVec3 px_centerOfMass = ConvertUtils::ToPx(centerOfMass);
	physx::PxVec3 px_inertiaTensor(inertiaTensor(0, 0), inertiaTensor(1, 1), inertiaTensor(2, 2));
	
	m_RigidDynamic->setCMassLocalPose(physx::PxTransform(px_centerOfMass));
	m_RigidDynamic->setMass(mass);
	m_RigidDynamic->setMassSpaceInertiaTensor(px_inertiaTensor);
	
	m_Mass = mass;
	m_CenterOfMass = centerOfMass;
}

void PhysicsRigidDynamic::SetGravityEnabled(bool enabled)
{
	if (m_RigidDynamic == nullptr)
		return;
	
	m_RigidDynamic->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, !enabled);
	m_GravityEnabled = enabled;
}

void* PhysicsRigidDynamic::GetNativeActor() const 
{
	return m_RigidDynamic.get();
}

void PhysicsRigidDynamic::SetSleepThreshold(const MathLib::HReal& threshold)
{
	if (m_RigidDynamic == nullptr)
		return;
	
	m_RigidDynamic->setSleepThreshold(threshold);
	m_SleepThreshold = threshold;
}

void PhysicsRigidDynamic::SetFriction(const MathLib::HReal& staticFriction, const MathLib::HReal& dynamicFriction)
{
	if (m_RigidDynamic == nullptr || m_Material == nullptr)
		return;
	
	m_Material->SetStaticFriction(staticFriction);
	m_Material->SetDynamicFriction(dynamicFriction);
	PxU32 numShapes = m_RigidDynamic->getNbShapes();
	std::vector<PxShape*> shapes(numShapes);
	m_RigidDynamic->getShapes(shapes.data(), numShapes);
	
	for (PxU32 i = 0; i < numShapes; i++)
	{
		PxMaterial* material = static_cast<PxMaterial*>(shapes[i]->getMaterialFromInternalFaceIndex(0));
		if (material)
		{
			material->setStaticFriction(staticFriction);
			material->setDynamicFriction(dynamicFriction);
		}
	}
}

MathLib::HReal PhysicsRigidDynamic::GetStaticFriction() const
{
	if (m_Material == nullptr)
		return 0.0f;
	
	return m_Material->GetStaticFriction();
}

MathLib::HReal PhysicsRigidDynamic::GetDynamicFriction() const
{
	if (m_Material == nullptr)
		return 0.0f;
	
	return m_Material->GetDynamicFriction();
}

void PhysicsRigidDynamic::SetRestitution(const MathLib::HReal& restitution)
{
	if (m_RigidDynamic == nullptr || m_Material == nullptr)
		return;
	
	m_Material->SetRestitution(restitution);
	
	PxU32 numShapes = m_RigidDynamic->getNbShapes();
	std::vector<PxShape*> shapes(numShapes);
	m_RigidDynamic->getShapes(shapes.data(), numShapes);
	
	for (PxU32 i = 0; i < numShapes; i++)
	{
		PxMaterial* material = static_cast<PxMaterial*>(shapes[i]->getMaterialFromInternalFaceIndex(0));
		if (material)
		{
			material->setRestitution(restitution);
		}
	}
}

MathLib::HReal PhysicsRigidDynamic::GetRestitution() const
{
	if (m_Material == nullptr)
		return 0.0f;
	
	return m_Material->GetRestitution();
}


PhysicsObjectType PhysicsRigidDynamic::GetRigidBodyType() const
{
	return PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC;
}


/////////////////RigidStatic////////////////////////
PhysicsRigidStatic::PhysicsRigidStatic(PhysicsPtr<IPhysicsMaterial> &material)
{
	m_RigidStatic = make_physx_ptr<PxRigidStatic>(PxGetPhysics().createRigidStatic(PxTransform(PxIdentity)));
	m_Material = material;
	m_Transform.setIdentity();
	m_BoundingBox.setEmpty();
	m_Type = PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
}

void PhysicsRigidStatic::Release()
{
	m_RigidStatic.reset();
	m_Material.reset();
}

bool PhysicsRigidStatic::AddColliderGeometry(PhysicsPtr<IColliderGeometry> &colliderGeometry, const MathLib::HTransform3 &localTrans)
{
	if (m_RigidStatic == nullptr || colliderGeometry == nullptr)
		return false;
	physx::PxShape *shape = ShapeFactory::CreateShape(colliderGeometry.get(), m_Material.get());
	if (shape == nullptr)
		return false;
	if (colliderGeometry->GetType() == CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE)
	{
		const PlaneColliderGeometry *plane = static_cast<const PlaneColliderGeometry *>(colliderGeometry.get());
		const MathLib::HVector3 &normal = plane->GetNormal();
		const MathLib::HReal &distance = plane->GetDistance();
		auto trans = PxTransformFromPlaneEquation(PxPlane(normal[0], normal[1], normal[2], distance));
		shape->setLocalPose(ConvertUtils::ToPx(localTrans).transform(trans));
	}
	else
		shape->setLocalPose(ConvertUtils::ToPx(localTrans));
	m_RigidStatic->attachShape(*shape);
	PX_RELEASE(shape);
	m_ColliderGeometries.push_back(colliderGeometry);
	m_ColliderLocalPos.push_back(localTrans);
	m_BoundingBox = ComputeBoundingBox(this);
	return true;
}

bool PhysicsRigidStatic::RemoveColliderGeometry(PhysicsPtr<IColliderGeometry> &colliderGeometry)
{
	if (m_RigidStatic == nullptr || colliderGeometry == nullptr)
		return false;

	auto it = std::find(m_ColliderGeometries.begin(), m_ColliderGeometries.end(), colliderGeometry);
	if (it == m_ColliderGeometries.end())
		return false;

	size_t index = std::distance(m_ColliderGeometries.begin(), it);
	
	PxU32 numShapes = m_RigidStatic->getNbShapes();
	if (index >= numShapes)
		return false;

	std::vector<PxShape*> shapes(numShapes);
	m_RigidStatic->getShapes(shapes.data(), numShapes);
	
	m_RigidStatic->detachShape(*shapes[index]);
	
	m_ColliderGeometries.erase(m_ColliderGeometries.begin() + index);
	m_ColliderLocalPos.erase(m_ColliderLocalPos.begin() + index);

	m_BoundingBox = ComputeBoundingBox(this);
	
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
	m_RigidStatic->setGlobalPose(ConvertUtils::ToPx(transform));
	m_Transform = transform;
}

MathLib::HAABBox3D PhysicsRigidStatic::GetWorldBoundingBox() const
{
	if (m_RigidStatic == nullptr)
		return MathLib::HAABBox3D();
	MathLib::HAABBox3D box = m_BoundingBox;
	box.transform(m_Transform);
	return box;
}

void PhysicsRigidStatic::SetMassProperties(const MathLib::HReal& mass, const MathLib::HVector3& centerOfMass, const MathLib::HMatrix3& inertiaTensor)
{
}

MathLib::HReal PhysicsRigidStatic::GetMass() const
{
	return 0.0f;
}

MathLib::HVector3 PhysicsRigidStatic::GetCenterOfMass() const
{
	return MathLib::HVector3::Zero();
}

MathLib::HMatrix3 PhysicsRigidStatic::GetInertiaTensor() const
{
	return MathLib::HMatrix3::Identity();
}

void PhysicsRigidStatic::SetGravityEnabled(bool enabled)
{
}

bool PhysicsRigidStatic::IsGravityEnabled() const
{
	return false;
}

void PhysicsRigidStatic::SetSleepThreshold(const MathLib::HReal& threshold)
{
}

MathLib::HReal PhysicsRigidStatic::GetSleepThreshold() const
{
	return 0.0f;
}

bool PhysicsRigidStatic::IsSleeping() const
{
	return true;
}

void PhysicsRigidStatic::WakeUp()
{
}

void PhysicsRigidStatic::PutToSleep()
{
}

void PhysicsRigidStatic::SetFriction(const MathLib::HReal& staticFriction, const MathLib::HReal& dynamicFriction)
{
	if (m_Material == nullptr)
		return;
	
	m_Material->SetStaticFriction(staticFriction);
	m_Material->SetDynamicFriction(dynamicFriction);
}

MathLib::HReal PhysicsRigidStatic::GetStaticFriction() const
{
	if (m_Material == nullptr)
		return 0.0f;
	
	return m_Material->GetStaticFriction();
}

MathLib::HReal PhysicsRigidStatic::GetDynamicFriction() const
{
	if (m_Material == nullptr)
		return 0.0f;
	
	return m_Material->GetDynamicFriction();
}

void PhysicsRigidStatic::SetRestitution(const MathLib::HReal& restitution)
{
	if (m_Material == nullptr)
		return;
	
	m_Material->SetRestitution(restitution);
}

MathLib::HReal PhysicsRigidStatic::GetRestitution() const
{
	if (m_Material == nullptr)
		return 0.0f;
	
	return m_Material->GetRestitution();
}

void PhysicsRigidStatic::SetLinearDamping(const MathLib::HReal& damping)
{
}

MathLib::HReal PhysicsRigidStatic::GetLinearDamping() const
{
	return 0.0f;
}

void PhysicsRigidStatic::SetAngularDamping(const MathLib::HReal& damping)
{
}

MathLib::HReal PhysicsRigidStatic::GetAngularDamping() const
{
	return 0.0f;
}

void* PhysicsRigidStatic::GetNativeActor() const
{
	return m_RigidStatic.get();
}

PhysicsObjectType PhysicsRigidStatic::GetRigidBodyType() const
{
	return PhysicsObjectType::PHYSICS_OBJECT_TYPE_RIGID_STATIC;
}
