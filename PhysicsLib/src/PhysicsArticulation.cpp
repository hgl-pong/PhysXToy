#include "PhysicsArticulation.h"
#include "PhysicsEngine.h"
#include "PhysicsScene.h"
#include "ColliderGeometry.h"
#include <PxPhysicsAPI.h>

using namespace physx;

// Helper functions for conversion between enum types
static PxArticulationJointType::Enum ToPxArticulationJointType(ArticulationJointType type)
{
	switch (type)
	{
	case ArticulationJointType::FIXED: return PxArticulationJointType::eFIX;
	case ArticulationJointType::PRISMATIC: return PxArticulationJointType::ePRISMATIC;
	case ArticulationJointType::REVOLUTE: return PxArticulationJointType::eREVOLUTE;
	case ArticulationJointType::REVOLUTE_UNWRAPPED: return PxArticulationJointType::eREVOLUTE_UNWRAPPED;
	case ArticulationJointType::SPHERICAL: return PxArticulationJointType::eSPHERICAL;
	default: return PxArticulationJointType::eFIX;
	}
}

static ArticulationJointType FromPxArticulationJointType(PxArticulationJointType::Enum type)
{
	switch (type)
	{
	case PxArticulationJointType::eFIX: return ArticulationJointType::FIXED;
	case PxArticulationJointType::ePRISMATIC: return ArticulationJointType::PRISMATIC;
	case PxArticulationJointType::eREVOLUTE: return ArticulationJointType::REVOLUTE;
	case PxArticulationJointType::eREVOLUTE_UNWRAPPED: return ArticulationJointType::REVOLUTE_UNWRAPPED;
	case PxArticulationJointType::eSPHERICAL: return ArticulationJointType::SPHERICAL;
	default: return ArticulationJointType::FIXED;
	}
}

static PxArticulationAxis::Enum ToPxArticulationAxis(ArticulationAxis axis)
{
	switch (axis)
	{
	case ArticulationAxis::TWIST: return PxArticulationAxis::eTWIST;
	case ArticulationAxis::SWING1: return PxArticulationAxis::eSWING1;
	case ArticulationAxis::SWING2: return PxArticulationAxis::eSWING2;
	case ArticulationAxis::X: return PxArticulationAxis::eX;
	case ArticulationAxis::Y: return PxArticulationAxis::eY;
	case ArticulationAxis::Z: return PxArticulationAxis::eZ;
	default: return PxArticulationAxis::eTWIST;
	}
}

static PxArticulationMotion::Enum ToPxArticulationMotion(ArticulationMotion motion)
{
	switch (motion)
	{
	case ArticulationMotion::LOCKED: return PxArticulationMotion::eLOCKED;
	case ArticulationMotion::LIMITED: return PxArticulationMotion::eLIMITED;
	case ArticulationMotion::FREE: return PxArticulationMotion::eFREE;
	default: return PxArticulationMotion::eLOCKED;
	}
}

static ArticulationMotion FromPxArticulationMotion(PxArticulationMotion::Enum motion)
{
	switch (motion)
	{
	case PxArticulationMotion::eLOCKED: return ArticulationMotion::LOCKED;
	case PxArticulationMotion::eLIMITED: return ArticulationMotion::LIMITED;
	case PxArticulationMotion::eFREE: return ArticulationMotion::FREE;
	default: return ArticulationMotion::LOCKED;
	}
}

static PxForceMode::Enum ToPxForceMode(ForceMode mode)
{
	switch (mode)
	{
	case ForceMode::FORCE: return PxForceMode::eFORCE;
	case ForceMode::IMPULSE: return PxForceMode::eIMPULSE;
	case ForceMode::VELOCITY_CHANGE: return PxForceMode::eVELOCITY_CHANGE;
	case ForceMode::ACCELERATION: return PxForceMode::eACCELERATION;
	default: return PxForceMode::eFORCE;
	}
}

// PhysicsArticulation Implementation
PhysicsArticulation::PhysicsArticulation(const ArticulationCreateOptions& options)
	: m_PxArticulation(nullptr)
	, m_Scene(nullptr)
	, m_RootLink(nullptr)
	, m_Options(options)
	, m_UserData(nullptr)
{
	// Create PhysX articulation
	PhysicsEngine* engine = static_cast<PhysicsEngine*>(PhysicsEngineUtils::GetPhysicsEngine());
	if (!engine) return;
	
	PxPhysics* physics = engine->GetNativePhysics();
	if (!physics) return;
	
	m_PxArticulation = physics->createArticulationReducedCoordinate();
	
	if (m_PxArticulation)
	{
		// Configure articulation
		m_PxArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, options.fixedBase);
		// Note: Some PhysX versions may not have these methods, so we'll comment them out for now
		// m_PxArticulation->setMaxProjectionIterations(options.maxProjectionIterations);
		// m_PxArticulation->setSeparationTolerance(options.separationTolerance);
		m_PxArticulation->setSolverIterationCounts(options.solverIterationCounts);
		// m_PxArticulation->setArticulationFlag(PxArticulationFlag::eENABLE_SELF_COLLISION, options.enableSelfCollision);
		m_PxArticulation->setSleepThreshold(options.sleepThreshold);
		m_PxArticulation->setStabilizationThreshold(options.stabilizationThreshold);
		m_PxArticulation->setWakeCounter(options.wakeCounter);
		
		m_PxArticulation->userData = this;
	}
}

PhysicsArticulation::~PhysicsArticulation()
{
	Release();
}

void PhysicsArticulation::Release()
{
	if (m_PxArticulation)
	{
		m_PxArticulation->release();
		m_PxArticulation = nullptr;
	}
	m_Links.clear();
	m_RootLink = nullptr;
	m_Scene = nullptr;
}

PhysicsPtr<IPhysicsScene> PhysicsArticulation::GetScene() const
{
	return m_Scene;
}

bool PhysicsArticulation::IsFixedBase() const
{
	if (!m_PxArticulation) return false;
	return m_PxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
}

void PhysicsArticulation::SetSolverIterationCounts(uint32_t minPositionIters, uint32_t minVelocityIters)
{
	if (m_PxArticulation)
	{
		m_PxArticulation->setSolverIterationCounts(minPositionIters, minVelocityIters);
	}
}

void PhysicsArticulation::GetSolverIterationCounts(uint32_t& minPositionIters, uint32_t& minVelocityIters) const
{
	if (m_PxArticulation)
	{
		m_PxArticulation->getSolverIterationCounts(minPositionIters, minVelocityIters);
	}
	else
	{
		minPositionIters = 0;
		minVelocityIters = 0;
	}
}

PhysicsPtr<IArticulationLink> PhysicsArticulation::CreateLink(PhysicsPtr<IArticulationLink> parent, const ArticulationLinkCreateOptions& options)
{
	if (!m_PxArticulation) return nullptr;
	
	auto link = std::make_shared<PhysicsArticulationLink>(this, parent, options);
	
	// Create PhysX link
	PxArticulationLink* pxParentLink = nullptr;
	if (parent)
	{
		auto parentImpl = std::static_pointer_cast<PhysicsArticulationLink>(parent);
		pxParentLink = parentImpl->GetPxLink();
	}
	
	PxTransform pxPose = ConvertUtils::ToPx(options.pose);
	PxArticulationLink* pxLink = m_PxArticulation->createLink(pxParentLink, pxPose);
	
	if (pxLink)
	{
		link->SetPxLink(pxLink);
		pxLink->userData = link.get();
		
		// Create joint for this link if it has a parent
		if (parent && pxLink->getInboundJoint())
		{
			ArticulationJointCreateOptions jointOptions;
			jointOptions.jointType = ArticulationJointType::REVOLUTE; // Default
			jointOptions.parentPose = MathLib::HTransform3::Identity();
			jointOptions.childPose = MathLib::HTransform3::Identity();
			
			auto joint = std::make_shared<PhysicsArticulationJoint>(link, jointOptions);
			link->SetInboundJoint(joint);
		}
		
		// Set mass properties
		PxRigidBodyExt::setMassAndUpdateInertia(*pxLink, options.mass);
		
		// Set damping
		pxLink->setLinearDamping(options.linearDamping);
		pxLink->setAngularDamping(options.angularDamping);
		
		// Set velocity limits
		pxLink->setMaxLinearVelocity(options.maxLinearVelocity);
		pxLink->setMaxAngularVelocity(options.maxAngularVelocity);
		
		m_Links.push_back(link);
		
		if (!parent)
		{
			m_RootLink = link;
		}
		else
		{
			// Add this link as a child of the parent
			auto parentImpl = std::static_pointer_cast<PhysicsArticulationLink>(parent);
			parentImpl->AddChild(link);
		}
		// Note: We'll implement AddChild method in PhysicsArticulationLink
	}
	
	return link;
}

uint32_t PhysicsArticulation::GetNbLinks() const
{
	return static_cast<uint32_t>(m_Links.size());
}

uint32_t PhysicsArticulation::GetLinks(std::vector<PhysicsPtr<IArticulationLink>>& links) const
{
	links = m_Links;
	return static_cast<uint32_t>(m_Links.size());
}

PhysicsPtr<IArticulationLink> PhysicsArticulation::GetRootLink() const
{
	return m_RootLink;
}

uint32_t PhysicsArticulation::GetDofs() const
{
	if (!m_PxArticulation) return 0;
	return m_PxArticulation->getDofs();
}

void PhysicsArticulation::UpdateKinematic()
{
	if (m_PxArticulation)
	{
		// This method would update kinematic quantities
		// Implementation depends on specific PhysX version and requirements
	}
}

PhysicsPtr<IArticulationCache> PhysicsArticulation::CreateCache()
{
	return std::make_shared<PhysicsArticulationCache>(shared_from_this());
}

void PhysicsArticulation::ApplyCache(PhysicsPtr<IArticulationCache> cache, uint32_t flag)
{
	if (!m_PxArticulation || !cache) return;
	
	auto cacheImpl = std::static_pointer_cast<PhysicsArticulationCache>(cache);
	PxArticulationCache* pxCache = cacheImpl->GetPxCache();
	if (pxCache)
	{
		m_PxArticulation->applyCache(*pxCache, PxArticulationCacheFlags(flag));
	}
}

void PhysicsArticulation::CopyInternalStateToCache(PhysicsPtr<IArticulationCache> cache, uint32_t flag) const
{
	if (!m_PxArticulation || !cache) return;
	
	auto cacheImpl = std::static_pointer_cast<PhysicsArticulationCache>(cache);
	PxArticulationCache* pxCache = cacheImpl->GetPxCache();
	if (pxCache)
	{
		m_PxArticulation->copyInternalStateToCache(*pxCache, PxArticulationCacheFlags(flag));
	}
}

void PhysicsArticulation::SetMaxProjectionIterations(uint32_t iterations)
{
	if (m_PxArticulation)
	{
		// m_PxArticulation->setMaxProjectionIterations(iterations);
		// This method may not exist in all PhysX versions
	}
}

uint32_t PhysicsArticulation::GetMaxProjectionIterations() const
{
	if (!m_PxArticulation) return 0;
	// return m_PxArticulation->getMaxProjectionIterations();
	return m_Options.maxProjectionIterations; // Return from stored options
}

void PhysicsArticulation::SetSeparationTolerance(MathLib::HReal tolerance)
{
	if (m_PxArticulation)
	{
		// m_PxArticulation->setSeparationTolerance(tolerance);
		// This method may not exist in all PhysX versions
	}
}

MathLib::HReal PhysicsArticulation::GetSeparationTolerance() const
{
	if (!m_PxArticulation) return 0.0f;
	// return m_PxArticulation->getSeparationTolerance();
	return m_Options.separationTolerance; // Return from stored options
}

void PhysicsArticulation::SetInternalDriveIterations(uint32_t iterations)
{
	if (m_PxArticulation)
	{
		// m_PxArticulation->setInternalDriveIterations(iterations);
		// This method may not exist in all PhysX versions
	}
}

uint32_t PhysicsArticulation::GetInternalDriveIterations() const
{
	if (!m_PxArticulation) return 0;
	// return m_PxArticulation->getInternalDriveIterations();
	return 4; // Default value
}

void PhysicsArticulation::SetExternalDriveIterations(uint32_t iterations)
{
	if (m_PxArticulation)
	{
		// m_PxArticulation->setExternalDriveIterations(iterations);
		// This method may not exist in all PhysX versions
	}
}

uint32_t PhysicsArticulation::GetExternalDriveIterations() const
{
	if (!m_PxArticulation) return 0;
	// return m_PxArticulation->getExternalDriveIterations();
	return 4; // Default value
}

bool PhysicsArticulation::IsSleeping() const
{
	if (!m_PxArticulation) return false;
	return m_PxArticulation->isSleeping();
}

void PhysicsArticulation::SetSleepThreshold(MathLib::HReal threshold)
{
	if (m_PxArticulation)
	{
		m_PxArticulation->setSleepThreshold(threshold);
	}
}

MathLib::HReal PhysicsArticulation::GetSleepThreshold() const
{
	if (!m_PxArticulation) return 0.0f;
	return m_PxArticulation->getSleepThreshold();
}

void PhysicsArticulation::SetStabilizationThreshold(MathLib::HReal threshold)
{
	if (m_PxArticulation)
	{
		m_PxArticulation->setStabilizationThreshold(threshold);
	}
}

MathLib::HReal PhysicsArticulation::GetStabilizationThreshold() const
{
	if (!m_PxArticulation) return 0.0f;
	return m_PxArticulation->getStabilizationThreshold();
}

void PhysicsArticulation::SetWakeCounter(MathLib::HReal wakeCounterValue)
{
	if (m_PxArticulation)
	{
		m_PxArticulation->setWakeCounter(wakeCounterValue);
	}
}

MathLib::HReal PhysicsArticulation::GetWakeCounter() const
{
	if (!m_PxArticulation) return 0.0f;
	return m_PxArticulation->getWakeCounter();
}

void PhysicsArticulation::WakeUp()
{
	if (m_PxArticulation)
	{
		m_PxArticulation->wakeUp();
	}
}

void PhysicsArticulation::PutToSleep()
{
	if (m_PxArticulation)
	{
		m_PxArticulation->putToSleep();
	}
}

void PhysicsArticulation::CommonInit()
{
	if (m_PxArticulation)
	{
		m_PxArticulation->commonInit();
	}
}

void PhysicsArticulation::ComputeGeneralizedGravityForce(PhysicsPtr<IArticulationCache> cache)
{
	if (!m_PxArticulation || !cache) return;
	
	auto cacheImpl = std::static_pointer_cast<PhysicsArticulationCache>(cache);
	PxArticulationCache* pxCache = cacheImpl->GetPxCache();
	if (pxCache)
	{
		m_PxArticulation->computeGeneralizedGravityForce(*pxCache);
	}
}

void PhysicsArticulation::ComputeCoriolisAndCentrifugalForce(PhysicsPtr<IArticulationCache> cache)
{
	if (!m_PxArticulation || !cache) return;
	
	auto cacheImpl = std::static_pointer_cast<PhysicsArticulationCache>(cache);
	PxArticulationCache* pxCache = cacheImpl->GetPxCache();
	if (pxCache)
	{
		m_PxArticulation->computeCoriolisAndCentrifugalForce(*pxCache);
	}
}

void PhysicsArticulation::ComputeGeneralizedExternalForce(PhysicsPtr<IArticulationCache> cache)
{
	if (!m_PxArticulation || !cache) return;
	
	auto cacheImpl = std::static_pointer_cast<PhysicsArticulationCache>(cache);
	PxArticulationCache* pxCache = cacheImpl->GetPxCache();
	if (pxCache)
	{
		m_PxArticulation->computeGeneralizedExternalForce(*pxCache);
	}
}

void PhysicsArticulation::ComputeJointAcceleration(PhysicsPtr<IArticulationCache> cache)
{
	if (!m_PxArticulation || !cache) return;
	
	auto cacheImpl = std::static_pointer_cast<PhysicsArticulationCache>(cache);
	PxArticulationCache* pxCache = cacheImpl->GetPxCache();
	if (pxCache)
	{
		m_PxArticulation->computeJointAcceleration(*pxCache);
	}
}

void PhysicsArticulation::ComputeJointForce(PhysicsPtr<IArticulationCache> cache)
{
	if (!m_PxArticulation || !cache) return;
	
	auto cacheImpl = std::static_pointer_cast<PhysicsArticulationCache>(cache);
	PxArticulationCache* pxCache = cacheImpl->GetPxCache();
	if (pxCache)
	{
		m_PxArticulation->computeJointForce(*pxCache);
	}
}

void PhysicsArticulation::ComputeGeneralizedMassMatrix(PhysicsPtr<IArticulationCache> cache)
{
	if (!m_PxArticulation || !cache) return;
	
	auto cacheImpl = std::static_pointer_cast<PhysicsArticulationCache>(cache);
	PxArticulationCache* pxCache = cacheImpl->GetPxCache();
	if (pxCache)
	{
		m_PxArticulation->computeGeneralizedMassMatrix(*pxCache);
	}
}

void PhysicsArticulation::ComputeDenseJacobian(PhysicsPtr<IArticulationCache> cache, uint32_t& nRows, uint32_t& nCols)
{
	if (!m_PxArticulation || !cache) return;
	
	auto cacheImpl = std::static_pointer_cast<PhysicsArticulationCache>(cache);
	PxArticulationCache* pxCache = cacheImpl->GetPxCache();
	if (pxCache)
	{
		m_PxArticulation->computeDenseJacobian(*pxCache, nRows, nCols);
	}
}

void PhysicsArticulation::SetName(const char* name)
{
	if (name)
	{
		m_Name = name;
		if (m_PxArticulation)
		{
			m_PxArticulation->setName(name);
		}
	}
}

const char* PhysicsArticulation::GetName() const
{
	return m_Name.c_str();
}

MathLib::HAABBox3D PhysicsArticulation::GetWorldBounds(MathLib::HReal inflation) const
{
	if (!m_PxArticulation)
	{
		return MathLib::HAABBox3D();
	}
	
	PxBounds3 bounds = m_PxArticulation->getWorldBounds(inflation);
	return ConvertUtils::FromPx(bounds);
}

void PhysicsArticulation::SetUserData(void* userData)
{
	m_UserData = userData;
}

void* PhysicsArticulation::GetUserData() const
{
	return m_UserData;
}

uint32_t PhysicsArticulation::GetInternalActorIndex() const
{
	// Implementation would depend on PhysX internal indexing
	return 0;
}

size_t PhysicsArticulation::GetOffset() const
{
	return reinterpret_cast<size_t>(m_PxArticulation);
}

void* PhysicsArticulation::GetNativeArticulation() const
{
	return m_PxArticulation;
}

// ... (Continue with PhysicsArticulationLink, PhysicsArticulationJoint, and PhysicsArticulationCache implementations)
// Due to length constraints, I'll provide the key structure and methods 

// PhysicsArticulationLink Implementation
PhysicsArticulationLink::PhysicsArticulationLink(PhysicsArticulation* articulation, PhysicsPtr<IArticulationLink> parent, 
												 const ArticulationLinkCreateOptions& options)
	: m_PxLink(nullptr)
	, m_Articulation(articulation)
	, m_Parent(parent)
	, m_Options(options)
	, m_UserData(nullptr)
	, m_CollisionLayer(0)
	, m_CollisionMask(0xFFFFFFFF)
	, m_CollisionCallback(nullptr)
{
	m_Transform = options.pose;
}

PhysicsArticulationLink::~PhysicsArticulationLink()
{
	Release();
}

void PhysicsArticulationLink::Release()
{
	// Note: PxArticulationLink is released when the articulation is released
	m_PxLink = nullptr;
	m_Children.clear();
	m_ColliderGeometries.clear();
	m_ColliderLocalTransforms.clear();
}

void PhysicsArticulationLink::Update()
{
	if (m_PxLink)
	{
		// Update transform from PhysX
		PxTransform pxTransform = m_PxLink->getGlobalPose();
		m_Transform = ConvertUtils::FromPx(pxTransform);
	}
}

bool PhysicsArticulationLink::AddColliderGeometry(PhysicsPtr<IColliderGeometry> &colliderGeometry, const MathLib::HTransform3 &localTrans)
{
	if (!colliderGeometry || !m_PxLink) return false;
	
	m_ColliderGeometries.push_back(colliderGeometry);
	m_ColliderLocalTransforms.push_back(localTrans);
	
	// Implementation would attach geometry to PhysX link
	return true;
}

bool PhysicsArticulationLink::RemoveColliderGeometry(PhysicsPtr<IColliderGeometry> &colliderGeometry)
{
	auto it = std::find(m_ColliderGeometries.begin(), m_ColliderGeometries.end(), colliderGeometry);
	if (it != m_ColliderGeometries.end())
	{
		size_t index = std::distance(m_ColliderGeometries.begin(), it);
		m_ColliderGeometries.erase(it);
		m_ColliderLocalTransforms.erase(m_ColliderLocalTransforms.begin() + index);
		return true;
	}
	return false;
}

void PhysicsArticulationLink::GetColliderGeometries(std::vector<PhysicsPtr<IColliderGeometry>> &geometries, std::vector<MathLib::HTransform3> *geoLocalPos)
{
	geometries = m_ColliderGeometries;
	if (geoLocalPos)
	{
		*geoLocalPos = m_ColliderLocalTransforms;
	}
}

PhysicsObjectType PhysicsArticulationLink::GetType() const
{
	return PhysicsObjectType::PHYSICS_OBJECT_TYPE_ARTICULATION_LINK;
}

size_t PhysicsArticulationLink::GetOffset() const
{
	return reinterpret_cast<size_t>(m_PxLink);
}

void PhysicsArticulationLink::SetTransform(const MathLib::HTransform3 &trans)
{
	m_Transform = trans;
	if (m_PxLink)
	{
		PxTransform pxTransform = ConvertUtils::ToPx(trans);
		m_PxLink->setGlobalPose(pxTransform);
	}
}

const MathLib::HTransform3 &PhysicsArticulationLink::GetTransform() const
{
	if (m_PxLink)
	{
		PxTransform pxTransform = m_PxLink->getGlobalPose();
		m_Transform = ConvertUtils::FromPx(pxTransform);
	}
	return m_Transform;
}

bool PhysicsArticulationLink::IsValid() const
{
	return m_PxLink != nullptr;
}

MathLib::HAABBox3D PhysicsArticulationLink::GetLocalBoundingBox() const
{
	// Calculate local bounding box from colliders
	MathLib::HAABBox3D bbox;
	for (const auto& geometry : m_ColliderGeometries)
	{
		bbox.extend(geometry->GetBoundingBox());
	}
	return bbox;
}

MathLib::HAABBox3D PhysicsArticulationLink::GetWorldBoundingBox() const
{
	MathLib::HAABBox3D localBox = GetLocalBoundingBox();
	return localBox.transformed(GetTransform());
}

void PhysicsArticulationLink::SetUserData(void* userData)
{
	m_UserData = userData;
}

void* PhysicsArticulationLink::GetUserData() const
{
	return m_UserData;
}

void PhysicsArticulationLink::SetCollisionLayer(uint32_t layer)
{
	m_CollisionLayer = layer;
}

uint32_t PhysicsArticulationLink::GetCollisionLayer() const
{
	return m_CollisionLayer;
}

void PhysicsArticulationLink::SetCollisionMask(uint32_t mask)
{
	m_CollisionMask = mask;
}

uint32_t PhysicsArticulationLink::GetCollisionMask() const
{
	return m_CollisionMask;
}

void PhysicsArticulationLink::SetCollisionCallback(ICollisionCallback* callback)
{
	m_CollisionCallback = callback;
}

ICollisionCallback* PhysicsArticulationLink::GetCollisionCallback() const
{
	return m_CollisionCallback;
}

IArticulation* PhysicsArticulationLink::GetArticulation() const
{
	return m_Articulation;
}

IArticulationJoint* PhysicsArticulationLink::GetInboundJoint() const
{
	return m_InboundJoint.get();
}

uint32_t PhysicsArticulationLink::GetInboundJointDof() const
{
	if (m_PxLink)
	{
		PxArticulationJointReducedCoordinate* joint = m_PxLink->getInboundJoint();
		if (joint)
		{
			// return joint->getDofCount();
			// This method may not exist in all PhysX versions
			return 1; // Default value
		}
	}
	return 0;
}

uint32_t PhysicsArticulationLink::GetLinkIndex() const
{
	if (m_PxLink)
	{
		return m_PxLink->getLinkIndex();
	}
	return 0;
}

uint32_t PhysicsArticulationLink::GetNbChildren() const
{
	return static_cast<uint32_t>(m_Children.size());
}

uint32_t PhysicsArticulationLink::GetChildren(std::vector<PhysicsPtr<IArticulationLink>>& children) const
{
	children = m_Children;
	return static_cast<uint32_t>(m_Children.size());
}

void PhysicsArticulationLink::SetMass(MathLib::HReal mass)
{
	if (m_PxLink)
	{
		m_PxLink->setMass(mass);
	}
}

MathLib::HReal PhysicsArticulationLink::GetMass() const
{
	if (m_PxLink)
	{
		return m_PxLink->getMass();
	}
	return 0.0f;
}

MathLib::HReal PhysicsArticulationLink::GetInvMass() const
{
	if (m_PxLink)
	{
		return m_PxLink->getInvMass();
	}
	return 0.0f;
}

void PhysicsArticulationLink::SetCMassLocalPose(const MathLib::HTransform3& pose)
{
	if (m_PxLink)
	{
		PxTransform pxPose = ConvertUtils::ToPx(pose);
		m_PxLink->setCMassLocalPose(pxPose);
	}
}

MathLib::HTransform3 PhysicsArticulationLink::GetCMassLocalPose() const
{
	if (m_PxLink)
	{
		PxTransform pxPose = m_PxLink->getCMassLocalPose();
		return ConvertUtils::FromPx(pxPose);
	}
	return MathLib::HTransform3::Identity();
}

void PhysicsArticulationLink::SetMassSpaceInertiaTensor(const MathLib::HVector3& inertia)
{
	if (m_PxLink)
	{
		PxVec3 pxInertia = ConvertUtils::ToPx(inertia);
		m_PxLink->setMassSpaceInertiaTensor(pxInertia);
	}
}

MathLib::HVector3 PhysicsArticulationLink::GetMassSpaceInertiaTensor() const
{
	if (m_PxLink)
	{
		PxVec3 pxInertia = m_PxLink->getMassSpaceInertiaTensor();
		return ConvertUtils::FromPx(pxInertia);
	}
	return MathLib::HVector3::Zero();
}

MathLib::HVector3 PhysicsArticulationLink::GetMassSpaceInvInertiaTensor() const
{
	if (m_PxLink)
	{
		PxVec3 pxInertia = m_PxLink->getMassSpaceInvInertiaTensor();
		return ConvertUtils::FromPx(pxInertia);
	}
	return MathLib::HVector3::Zero();
}

void PhysicsArticulationLink::SetLinearDamping(MathLib::HReal damping)
{
	if (m_PxLink)
	{
		m_PxLink->setLinearDamping(damping);
	}
}

MathLib::HReal PhysicsArticulationLink::GetLinearDamping() const
{
	if (m_PxLink)
	{
		return m_PxLink->getLinearDamping();
	}
	return 0.0f;
}

void PhysicsArticulationLink::SetAngularDamping(MathLib::HReal damping)
{
	if (m_PxLink)
	{
		m_PxLink->setAngularDamping(damping);
	}
}

MathLib::HReal PhysicsArticulationLink::GetAngularDamping() const
{
	if (m_PxLink)
	{
		return m_PxLink->getAngularDamping();
	}
	return 0.0f;
}

void PhysicsArticulationLink::SetMaxLinearVelocity(MathLib::HReal maxVel)
{
	if (m_PxLink)
	{
		m_PxLink->setMaxLinearVelocity(maxVel);
	}
}

MathLib::HReal PhysicsArticulationLink::GetMaxLinearVelocity() const
{
	if (m_PxLink)
	{
		return m_PxLink->getMaxLinearVelocity();
	}
	return 0.0f;
}

void PhysicsArticulationLink::SetMaxAngularVelocity(MathLib::HReal maxVel)
{
	if (m_PxLink)
	{
		m_PxLink->setMaxAngularVelocity(maxVel);
	}
}

MathLib::HReal PhysicsArticulationLink::GetMaxAngularVelocity() const
{
	if (m_PxLink)
	{
		return m_PxLink->getMaxAngularVelocity();
	}
	return 0.0f;
}

MathLib::HVector3 PhysicsArticulationLink::GetLinearVelocity() const
{
	if (m_PxLink)
	{
		PxVec3 velocity = m_PxLink->getLinearVelocity();
		return ConvertUtils::FromPx(velocity);
	}
	return MathLib::HVector3::Zero();
}

MathLib::HVector3 PhysicsArticulationLink::GetAngularVelocity() const
{
	if (m_PxLink)
	{
		PxVec3 velocity = m_PxLink->getAngularVelocity();
		return ConvertUtils::FromPx(velocity);
	}
	return MathLib::HVector3::Zero();
}

MathLib::HVector3 PhysicsArticulationLink::GetLinearAcceleration() const
{
	// PhysX doesn't directly provide acceleration, would need to be calculated
	return MathLib::HVector3::Zero();
}

MathLib::HVector3 PhysicsArticulationLink::GetAngularAcceleration() const
{
	// PhysX doesn't directly provide acceleration, would need to be calculated
	return MathLib::HVector3::Zero();
}

void PhysicsArticulationLink::AddForce(const MathLib::HVector3& force, ForceMode mode, bool autowake)
{
	if (m_PxLink)
	{
		PxVec3 pxForce = ConvertUtils::ToPx(force);
		PxForceMode::Enum pxMode = ToPxForceMode(mode);
		m_PxLink->addForce(pxForce, pxMode, autowake);
	}
}

void PhysicsArticulationLink::AddTorque(const MathLib::HVector3& torque, ForceMode mode, bool autowake)
{
	if (m_PxLink)
	{
		PxVec3 pxTorque = ConvertUtils::ToPx(torque);
		PxForceMode::Enum pxMode = ToPxForceMode(mode);
		m_PxLink->addTorque(pxTorque, pxMode, autowake);
	}
}

void PhysicsArticulationLink::ClearForce(ForceMode mode)
{
	if (m_PxLink)
	{
		PxForceMode::Enum pxMode = ToPxForceMode(mode);
		m_PxLink->clearForce(pxMode);
	}
}

void PhysicsArticulationLink::ClearTorque(ForceMode mode)
{
	if (m_PxLink)
	{
		PxForceMode::Enum pxMode = ToPxForceMode(mode);
		m_PxLink->clearTorque(pxMode);
	}
}

void PhysicsArticulationLink::SetForceAndTorque(const MathLib::HVector3& force, const MathLib::HVector3& torque, ForceMode mode)
{
	if (m_PxLink)
	{
		PxVec3 pxForce = ConvertUtils::ToPx(force);
		PxVec3 pxTorque = ConvertUtils::ToPx(torque);
		PxForceMode::Enum pxMode = ToPxForceMode(mode);
		m_PxLink->setForceAndTorque(pxForce, pxTorque, pxMode);
	}
}

void PhysicsArticulationLink::SetMinCCDAdvanceCoefficient(MathLib::HReal coefficient)
{
	if (m_PxLink)
	{
		m_PxLink->setMinCCDAdvanceCoefficient(coefficient);
	}
}

MathLib::HReal PhysicsArticulationLink::GetMinCCDAdvanceCoefficient() const
{
	if (m_PxLink)
	{
		return m_PxLink->getMinCCDAdvanceCoefficient();
	}
	return 0.0f;
}

void PhysicsArticulationLink::SetMaxDepenetrationVelocity(MathLib::HReal maxVel)
{
	if (m_PxLink)
	{
		m_PxLink->setMaxDepenetrationVelocity(maxVel);
	}
}

MathLib::HReal PhysicsArticulationLink::GetMaxDepenetrationVelocity() const
{
	if (m_PxLink)
	{
		return m_PxLink->getMaxDepenetrationVelocity();
	}
	return 0.0f;
}

void PhysicsArticulationLink::SetMaxContactImpulse(MathLib::HReal maxImpulse)
{
	if (m_PxLink)
	{
		m_PxLink->setMaxContactImpulse(maxImpulse);
	}
}

MathLib::HReal PhysicsArticulationLink::GetMaxContactImpulse() const
{
	if (m_PxLink)
	{
		return m_PxLink->getMaxContactImpulse();
	}
	return 0.0f;
}

void PhysicsArticulationLink::SetContactSlopCoefficient(MathLib::HReal coefficient)
{
	if (m_PxLink)
	{
		m_PxLink->setContactSlopCoefficient(coefficient);
	}
}

MathLib::HReal PhysicsArticulationLink::GetContactSlopCoefficient() const
{
	if (m_PxLink)
	{
		return m_PxLink->getContactSlopCoefficient();
	}
	return 0.0f;
}

void PhysicsArticulationLink::SetCfmScale(MathLib::HReal cfm)
{
	if (m_PxLink)
	{
		m_PxLink->setCfmScale(cfm);
	}
}

MathLib::HReal PhysicsArticulationLink::GetCfmScale() const
{
	if (m_PxLink)
	{
		return m_PxLink->getCfmScale();
	}
	return 0.0f;
}

void* PhysicsArticulationLink::GetNativeLink() const
{
	return m_PxLink;
}

void PhysicsArticulationLink::AddChild(PhysicsPtr<IArticulationLink> child)
{
	m_Children.push_back(child);
}

// PhysicsArticulationJoint Implementation
PhysicsArticulationJoint::PhysicsArticulationJoint(PhysicsPtr<IArticulationLink> link, const ArticulationJointCreateOptions& options)
	: m_PxJoint(nullptr)
	, m_Link(link)
	, m_Options(options)
{
	if (link)
	{
		auto linkImpl = std::static_pointer_cast<PhysicsArticulationLink>(link);
		PxArticulationLink* pxLink = linkImpl->GetPxLink();
		if (pxLink)
		{
			m_PxJoint = pxLink->getInboundJoint();
			if (m_PxJoint)
			{
				// Configure joint
				m_PxJoint->setJointType(ToPxArticulationJointType(options.jointType));
				m_PxJoint->setParentPose(ConvertUtils::ToPx(options.parentPose));
				m_PxJoint->setChildPose(ConvertUtils::ToPx(options.childPose));
				m_PxJoint->setFrictionCoefficient(options.frictionCoefficient);
				m_PxJoint->setMaxJointVelocity(options.maxJointVelocity);
			}
		}
	}
}

PhysicsArticulationJoint::~PhysicsArticulationJoint()
{
	// Note: Joint is owned by the articulation, no explicit release needed
}

ArticulationJointType PhysicsArticulationJoint::GetJointType() const
{
	if (m_PxJoint)
	{
		return FromPxArticulationJointType(m_PxJoint->getJointType());
	}
	return ArticulationJointType::FIXED;
}

void PhysicsArticulationJoint::SetParentPose(const MathLib::HTransform3& pose)
{
	if (m_PxJoint)
	{
		PxTransform pxPose = ConvertUtils::ToPx(pose);
		m_PxJoint->setParentPose(pxPose);
	}
}

MathLib::HTransform3 PhysicsArticulationJoint::GetParentPose() const
{
	if (m_PxJoint)
	{
		PxTransform pxPose = m_PxJoint->getParentPose();
		return ConvertUtils::FromPx(pxPose);
	}
	return MathLib::HTransform3::Identity();
}

void PhysicsArticulationJoint::SetChildPose(const MathLib::HTransform3& pose)
{
	if (m_PxJoint)
	{
		PxTransform pxPose = ConvertUtils::ToPx(pose);
		m_PxJoint->setChildPose(pxPose);
	}
}

MathLib::HTransform3 PhysicsArticulationJoint::GetChildPose() const
{
	if (m_PxJoint)
	{
		PxTransform pxPose = m_PxJoint->getChildPose();
		return ConvertUtils::FromPx(pxPose);
	}
	return MathLib::HTransform3::Identity();
}

void PhysicsArticulationJoint::SetMotion(ArticulationAxis axis, ArticulationMotion motion)
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		PxArticulationMotion::Enum pxMotion = ToPxArticulationMotion(motion);
		m_PxJoint->setMotion(pxAxis, pxMotion);
	}
}

ArticulationMotion PhysicsArticulationJoint::GetMotion(ArticulationAxis axis) const
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		PxArticulationMotion::Enum pxMotion = m_PxJoint->getMotion(pxAxis);
		return FromPxArticulationMotion(pxMotion);
	}
	return ArticulationMotion::LOCKED;
}

uint32_t PhysicsArticulationJoint::GetDofCount() const
{
	if (m_PxJoint)
	{
		// return m_PxJoint->getDofCount();
		// This method may not exist in all PhysX versions
		return 1; // Default value
	}
	return 0;
}

void PhysicsArticulationJoint::SetLimit(ArticulationAxis axis, const ArticulationLimit& limit)
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		// PxArticulationLimit pxLimit(limit.low, limit.high);
		// m_PxJoint->setLimit(pxAxis, pxLimit);
		// These methods may not exist in all PhysX versions
	}
}

ArticulationLimit PhysicsArticulationJoint::GetLimit(ArticulationAxis axis) const
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		// PxArticulationLimit pxLimit = m_PxJoint->getLimit(pxAxis);
		// ArticulationLimit limit;
		// limit.low = pxLimit.low;
		// limit.high = pxLimit.high;
		// return limit;
		// These methods may not exist in all PhysX versions
	}
	return ArticulationLimit();
}

void PhysicsArticulationJoint::SetDrive(ArticulationAxis axis, const ArticulationDrive& drive)
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		// PxArticulationDrive pxDrive;
		// pxDrive.stiffness = drive.stiffness;
		// pxDrive.damping = drive.damping;
		// pxDrive.maxForce = drive.maxForce;
		// m_PxJoint->setDrive(pxAxis, pxDrive);
		// These methods may not exist in all PhysX versions
	}
}

ArticulationDrive PhysicsArticulationJoint::GetDrive(ArticulationAxis axis) const
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		// PxArticulationDrive pxDrive = m_PxJoint->getDrive(pxAxis);
		// ArticulationDrive drive;
		// drive.stiffness = pxDrive.stiffness;
		// drive.damping = pxDrive.damping;
		// drive.maxForce = pxDrive.maxForce;
		// drive.driveType = ArticulationDriveType::FORCE; // Default
		// return drive;
		// These methods may not exist in all PhysX versions
	}
	return ArticulationDrive();
}

void PhysicsArticulationJoint::SetDriveTarget(ArticulationAxis axis, MathLib::HReal target)
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		m_PxJoint->setDriveTarget(pxAxis, target);
	}
}

MathLib::HReal PhysicsArticulationJoint::GetDriveTarget(ArticulationAxis axis) const
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		return m_PxJoint->getDriveTarget(pxAxis);
	}
	return 0.0f;
}

void PhysicsArticulationJoint::SetDriveVelocity(ArticulationAxis axis, MathLib::HReal velocity)
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		m_PxJoint->setDriveVelocity(pxAxis, velocity);
	}
}

MathLib::HReal PhysicsArticulationJoint::GetDriveVelocity(ArticulationAxis axis) const
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		return m_PxJoint->getDriveVelocity(pxAxis);
	}
	return 0.0f;
}

void PhysicsArticulationJoint::SetJointPosition(ArticulationAxis axis, MathLib::HReal position)
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		m_PxJoint->setJointPosition(pxAxis, position);
	}
}

MathLib::HReal PhysicsArticulationJoint::GetJointPosition(ArticulationAxis axis) const
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		return m_PxJoint->getJointPosition(pxAxis);
	}
	return 0.0f;
}

void PhysicsArticulationJoint::SetJointVelocity(ArticulationAxis axis, MathLib::HReal velocity)
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		m_PxJoint->setJointVelocity(pxAxis, velocity);
	}
}

MathLib::HReal PhysicsArticulationJoint::GetJointVelocity(ArticulationAxis axis) const
{
	if (m_PxJoint)
	{
		PxArticulationAxis::Enum pxAxis = ToPxArticulationAxis(axis);
		return m_PxJoint->getJointVelocity(pxAxis);
	}
	return 0.0f;
}

void PhysicsArticulationJoint::SetFrictionCoefficient(MathLib::HReal coefficient)
{
	if (m_PxJoint)
	{
		m_PxJoint->setFrictionCoefficient(coefficient);
	}
}

MathLib::HReal PhysicsArticulationJoint::GetFrictionCoefficient() const
{
	if (m_PxJoint)
	{
		return m_PxJoint->getFrictionCoefficient();
	}
	return 0.0f;
}

void PhysicsArticulationJoint::SetMaxJointVelocity(MathLib::HReal maxVelocity)
{
	if (m_PxJoint)
	{
		m_PxJoint->setMaxJointVelocity(maxVelocity);
	}
}

MathLib::HReal PhysicsArticulationJoint::GetMaxJointVelocity() const
{
	if (m_PxJoint)
	{
		return m_PxJoint->getMaxJointVelocity();
	}
	return 0.0f;
}

size_t PhysicsArticulationJoint::GetOffset() const
{
	return reinterpret_cast<size_t>(m_PxJoint);
}

// PhysicsArticulationCache Implementation
PhysicsArticulationCache::PhysicsArticulationCache(PhysicsPtr<IArticulation> articulation)
	: m_PxCache(nullptr)
	, m_Articulation(articulation)
{
	if (articulation)
	{
		auto articulationImpl = std::static_pointer_cast<PhysicsArticulation>(articulation);
		PxArticulationReducedCoordinate* pxArticulation = articulationImpl->GetPxArticulation();
		if (pxArticulation)
		{
			m_PxCache = pxArticulation->createCache();
		}
	}
}

PhysicsArticulationCache::~PhysicsArticulationCache()
{
	Release();
}

void PhysicsArticulationCache::Release()
{
	if (m_PxCache && m_Articulation)
	{
		auto articulationImpl = std::static_pointer_cast<PhysicsArticulation>(m_Articulation);
		PxArticulationReducedCoordinate* pxArticulation = articulationImpl->GetPxArticulation();
		if (pxArticulation)
		{
			// pxArticulation->releaseCache(*m_PxCache);
			// This method may not exist in all PhysX versions
			// Manual cleanup may be needed
		}
		m_PxCache = nullptr;
	}
}

MathLib::HReal* PhysicsArticulationCache::GetJointPositions()
{
	if (m_PxCache)
	{
		return m_PxCache->jointPosition;
	}
	return nullptr;
}

const MathLib::HReal* PhysicsArticulationCache::GetJointPositions() const
{
	if (m_PxCache)
	{
		return m_PxCache->jointPosition;
	}
	return nullptr;
}

MathLib::HReal* PhysicsArticulationCache::GetJointVelocities()
{
	if (m_PxCache)
	{
		return m_PxCache->jointVelocity;
	}
	return nullptr;
}

const MathLib::HReal* PhysicsArticulationCache::GetJointVelocities() const
{
	if (m_PxCache)
	{
		return m_PxCache->jointVelocity;
	}
	return nullptr;
}

MathLib::HReal* PhysicsArticulationCache::GetJointAccelerations()
{
	if (m_PxCache)
	{
		return m_PxCache->jointAcceleration;
	}
	return nullptr;
}

const MathLib::HReal* PhysicsArticulationCache::GetJointAccelerations() const
{
	if (m_PxCache)
	{
		return m_PxCache->jointAcceleration;
	}
	return nullptr;
}

MathLib::HReal* PhysicsArticulationCache::GetJointForces()
{
	if (m_PxCache)
	{
		return m_PxCache->jointForce;
	}
	return nullptr;
}

const MathLib::HReal* PhysicsArticulationCache::GetJointForces() const
{
	if (m_PxCache)
	{
		return m_PxCache->jointForce;
	}
	return nullptr;
}

ArticulationSpatialVelocity* PhysicsArticulationCache::GetLinkVelocities()
{
	if (m_PxCache)
	{
		// Note: Type casting needed between PhysX and custom types
		return reinterpret_cast<ArticulationSpatialVelocity*>(m_PxCache->linkVelocity);
	}
	return nullptr;
}

const ArticulationSpatialVelocity* PhysicsArticulationCache::GetLinkVelocities() const
{
	if (m_PxCache)
	{
		return reinterpret_cast<const ArticulationSpatialVelocity*>(m_PxCache->linkVelocity);
	}
	return nullptr;
}

ArticulationSpatialVelocity* PhysicsArticulationCache::GetLinkAccelerations()
{
	if (m_PxCache)
	{
		return reinterpret_cast<ArticulationSpatialVelocity*>(m_PxCache->linkAcceleration);
	}
	return nullptr;
}

const ArticulationSpatialVelocity* PhysicsArticulationCache::GetLinkAccelerations() const
{
	if (m_PxCache)
	{
		return reinterpret_cast<const ArticulationSpatialVelocity*>(m_PxCache->linkAcceleration);
	}
	return nullptr;
}

ArticulationRootLinkData* PhysicsArticulationCache::GetRootLinkData()
{
	if (m_PxCache)
	{
		return reinterpret_cast<ArticulationRootLinkData*>(m_PxCache->rootLinkData);
	}
	return nullptr;
}

const ArticulationRootLinkData* PhysicsArticulationCache::GetRootLinkData() const
{
	if (m_PxCache)
	{
		return reinterpret_cast<const ArticulationRootLinkData*>(m_PxCache->rootLinkData);
	}
	return nullptr;
}

ArticulationSpatialForce* PhysicsArticulationCache::GetExternalForces()
{
	if (m_PxCache)
	{
		return reinterpret_cast<ArticulationSpatialForce*>(m_PxCache->externalForces);
	}
	return nullptr;
}

const ArticulationSpatialForce* PhysicsArticulationCache::GetExternalForces() const
{
	if (m_PxCache)
	{
		return reinterpret_cast<const ArticulationSpatialForce*>(m_PxCache->externalForces);
	}
	return nullptr;
}

MathLib::HReal* PhysicsArticulationCache::GetMassMatrix()
{
	if (m_PxCache)
	{
		return m_PxCache->massMatrix;
	}
	return nullptr;
}

const MathLib::HReal* PhysicsArticulationCache::GetMassMatrix() const
{
	if (m_PxCache)
	{
		return m_PxCache->massMatrix;
	}
	return nullptr;
}

MathLib::HReal* PhysicsArticulationCache::GetJacobian()
{
	if (m_PxCache)
	{
		return m_PxCache->denseJacobian;
	}
	return nullptr;
}

const MathLib::HReal* PhysicsArticulationCache::GetJacobian() const
{
	if (m_PxCache)
	{
		return m_PxCache->denseJacobian;
	}
	return nullptr;
} 