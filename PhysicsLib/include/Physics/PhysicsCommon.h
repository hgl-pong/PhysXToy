#pragma once
#include "PhysicsTypes.h"
#include "PhysicsMacros.h"
#include <memory>
#include <unordered_set>
#include <unordered_map>
#include <functional>

class IPhysicsEngine;
class IPhysicsScene;
class IColliderGeometry;
class IPhysicsObject;
class IRigidBody;
class IRigidStatic;
class IRigidDynamic;
class IPhysicsMaterial;
class IPhysicsJoint;
class ISoftBody;
class ICloth;
class IPhysicsDebugRenderer;
class IPhysicsProfiler;
class IPhysicsDebugRender;
class IArticulation;
class IArticulationLink;
class IArticulationJoint;
class IArticulationCache;

enum class ForceMode
{
	FORCE,          
	IMPULSE,        
	VELOCITY_CHANGE,
	ACCELERATION    
};

enum class ArticulationJointType
{
	FIXED,						// Fixed joint, 0 DOF
	PRISMATIC,					// Prismatic joint, 1 DOF
	REVOLUTE,					// Revolute joint, 1 DOF
	REVOLUTE_UNWRAPPED,			// Revolute joint without wrapping, 1 DOF
	SPHERICAL					// Spherical joint, 2-3 DOF
};

enum class ArticulationAxis
{
	TWIST,						// X-axis rotation
	SWING1,						// Y-axis rotation
	SWING2,						// Z-axis rotation
	X,							// X-axis translation
	Y,							// Y-axis translation
	Z							// Z-axis translation
};

enum class ArticulationMotion
{
	LOCKED,						// Locked motion
	LIMITED,					// Limited motion
	FREE						// Free motion
};

enum class ArticulationDriveType
{
	NONE,
	FORCE,						// Force drive
	ACCELERATION				// Acceleration drive
};

enum class ArticulationCacheFlag
{
	VELOCITY = (1 << 0),			// Joint velocities
	ACCELERATION = (1 << 1),		// Joint accelerations
	POSITION = (1 << 2),			// Joint positions
	FORCE = (1 << 3),				// Joint forces
	LINK_VELOCITY = (1 << 4),		// Link velocities
	LINK_ACCELERATION = (1 << 5),	// Link accelerations
	ROOT_TRANSFORM = (1 << 6),		// Root link transform
	ROOT_VELOCITIES = (1 << 7),		// Root link velocities
	SENSOR_FORCES = (1 << 8),		// Sensor forces
	JOINT_SOLVER_FORCES = (1 << 9),	// Joint solver forces
	ALL = 0x3FF
};

struct ArticulationLimit
{
	MathLib::HReal low = 0.0f;				// Lower limit
	MathLib::HReal high = 0.0f;				// Upper limit
	MathLib::HReal stiffness = 0.0f;		// Stiffness
	MathLib::HReal damping = 0.0f;			// Damping
	bool isValid = false;					// Is valid
};

struct ArticulationDrive
{
	ArticulationDriveType driveType = ArticulationDriveType::NONE;
	MathLib::HReal stiffness = 0.0f;		// Stiffness
	MathLib::HReal damping = 0.0f;			// Damping
	MathLib::HReal maxForce = 3.402823466e+38f;  // Force limit
	MathLib::HReal targetPosition = 0.0f;	// Target position
	MathLib::HReal targetVelocity = 0.0f;	// Target velocity
	bool isAcceleration = false;			// Is acceleration drive
};

struct ArticulationRootLinkData
{
	MathLib::HTransform3 transform;			// Root link transform
	MathLib::HVector3 worldLinVel;			// World linear velocity
	MathLib::HVector3 worldAngVel;			// World angular velocity
	MathLib::HVector3 worldLinAccel;		// World linear acceleration
	MathLib::HVector3 worldAngAccel;		// World angular acceleration
};

struct ArticulationSpatialForce
{
	MathLib::HVector3 force;				// Force
	MathLib::HVector3 torque;				// Torque
};

struct ArticulationSpatialVelocity
{
	MathLib::HVector3 linear;				// Linear velocity
	MathLib::HVector3 angular;				// Angular velocity
};

struct ArticulationCreateOptions
{
	bool fixedBase = false;					// Whether to fix the base
	uint32_t maxProjectionIterations = 4;	// Maximum projection iterations
	MathLib::HReal separationTolerance = 0.1f; // Separation tolerance
	uint32_t solverIterationCounts = 4;		// Solver iteration counts
	bool enableSelfCollision = false;		// Enable self collision
	MathLib::HReal sleepThreshold = 5e-5f;	// Sleep threshold
	MathLib::HReal stabilizationThreshold = 1e-5f; // Stabilization threshold
	MathLib::HReal wakeCounter = 0.4f;		// Wake counter
};

struct ArticulationLinkCreateOptions
{
	MathLib::HTransform3 pose;				// Link pose
	MathLib::HReal mass = 1.0f;				// Mass
	MathLib::HVector3 massLocalPose = MathLib::HVector3::Zero(); // Mass center local pose
	MathLib::HVector3 inertia = MathLib::HVector3(1.0f, 1.0f, 1.0f); // Inertia tensor
	MathLib::HReal linearDamping = 0.05f;	// Linear damping
	MathLib::HReal angularDamping = 0.05f;	// Angular damping
	MathLib::HReal maxLinearVelocity = 100.0f; // Maximum linear velocity
	MathLib::HReal maxAngularVelocity = 50.0f; // Maximum angular velocity
	void* userData = nullptr;				// User data
};

struct ArticulationJointCreateOptions
{
	ArticulationJointType jointType = ArticulationJointType::FIXED;
	MathLib::HTransform3 parentPose;		// Parent link pose
	MathLib::HTransform3 childPose;			// Child link pose
	MathLib::HReal frictionCoefficient = 0.05f; // Friction coefficient
	MathLib::HReal maxJointVelocity = 100.0f; // Maximum joint velocity
};

struct CollisionEventData
{
	PhysicsPtr<IPhysicsObject> objectA;
	PhysicsPtr<IPhysicsObject> objectB;
	MathLib::HVector3 contactPoint;
	MathLib::HVector3 contactNormal;
	MathLib::HReal penetrationDepth;
};

class PHYSICSLIB_API ICollisionCallback
{
public:

	virtual ~ICollisionCallback() = default;
	virtual	bool	BufferContacts()		const												= 0;
	virtual	size_t	GetContactFlags()		const												= 0;
	virtual	float	GetContactThreshold()	const												= 0;
	virtual void OnCollisionEnter(const CollisionEventData& eventData) = 0;
	virtual void OnCollisionStay(const CollisionEventData& eventData) = 0;
	virtual void OnCollisionExit(const CollisionEventData& eventData) = 0;
};

struct JointCreateOptions
{
	JointType type;
	PhysicsPtr<IPhysicsObject> objectA;
	PhysicsPtr<IPhysicsObject> objectB;
	MathLib::HTransform3 localFrameA;
	MathLib::HTransform3 localFrameB;
	bool collisionEnabled = false;
};

class PHYSICSLIB_API IPhysicsProfiler
{
public:
    virtual ~IPhysicsProfiler() = default;
    
    // Basic profiler control
    virtual void* ZoneStart(const char* eventName, bool detached, uint64_t contextId) = 0;
    virtual void ZoneEnd(void* profilerData, const char* eventName, bool detached, uint64_t contextId) = 0;
    virtual void RecordData(const char* name, float value, uint64_t contextId) = 0;
    virtual void RecordData(const char* name, int32_t value, uint64_t contextId) = 0;
    virtual void RecordFrame(const char* name, uint64_t contextId) = 0;
	virtual void EndFrame() = 0;
    
    // Statistics management
    virtual const PhysicsStatisticsData::FrameStats& GetLatestFrameStats() const = 0;
    virtual const std::vector<PhysicsStatisticsData::FrameStats>& GetFrameHistory() const = 0;
    virtual uint64_t GetAverageFrameTime() const = 0;
    virtual uint64_t GetAveragePhysicsStepTime() const = 0;
    virtual uint64_t GetPeakFrameTime() const = 0;
    virtual float GetPhysicsTimePercentage() const = 0;
    
    // Detailed event stats access
    virtual const std::unordered_map<std::string, ProfileEventStats>& GetEventStats() const = 0;
    
    // Control methods
    virtual void ResetStatistics() = 0;
    virtual void SetVerboseOutput(bool enable) = 0;
    
    // Export functionality
    virtual bool ExportStatisticsToCSV(const std::string& filename) = 0;
    virtual bool ExportStatisticsToJSON(const std::string& filename) = 0;
    virtual bool ExportStatisticsToHTML(const std::string& filename) = 0;
    
    virtual void SetMemoryUsage(uint64_t bytes) = 0;
    virtual void AddMemoryUsage(uint64_t bytes) = 0;
    virtual void SubtractMemoryUsage(uint64_t bytes) = 0;
    virtual uint64_t GetMemoryUsage() const = 0;
}; 

class PHYSICSLIB_API IPhysicsEngine
{
public:
	virtual PhysicsPtr<IPhysicsObject> CreateObject(const PhysicsObjectCreateOptions &options) = 0;
	virtual PhysicsPtr<IPhysicsMaterial> CreateMaterial(const PhysicsMaterialCreateOptions &options) = 0;
	virtual PhysicsPtr<IPhysicsScene> CreateScene(const PhysicsSceneCreateOptions &options) = 0;
	virtual PhysicsPtr<IColliderGeometry> CreateColliderGeometry(const CollisionGeometryCreateOptions &options) = 0;
	virtual PhysicsPtr<IPhysicsJoint> CreateJoint(const JointCreateOptions &options) = 0;
	virtual PhysicsPtr<ISoftBody> CreateSoftBody(const SoftBodyCreateOptions &options) = 0;
	virtual PhysicsPtr<ICloth> CreateCloth(const ClothCreateOptions &options) = 0;
	virtual PhysicsPtr<IArticulation> CreateArticulation(const ArticulationCreateOptions &options) = 0;
	virtual void Release() = 0;
	
	virtual void SetSolverIterationCount(uint32_t count) = 0;
	virtual uint32_t GetSolverIterationCount() const = 0;
	virtual void SetDebugRenderer(PhysicsPtr<IPhysicsDebugRenderer> renderer) = 0;
	virtual PhysicsPtr<IPhysicsDebugRenderer> GetDebugRenderer() const = 0;
	virtual void SetDebugRender(PhysicsPtr<IPhysicsDebugRender> render) = 0;
	virtual PhysicsPtr<IPhysicsDebugRender> GetDebugRender() const = 0;
	
	virtual IPhysicsProfiler* GetProfiler() = 0;
	virtual void RegisterCollisionCallback(ICollisionCallback* callback) = 0;
	virtual void UnregisterCollisionCallback(ICollisionCallback* callback) = 0;
	virtual void SetActiveScene(PhysicsPtr<IPhysicsScene> scene) = 0;
	virtual PhysicsPtr<IPhysicsScene> GetActiveScene() const = 0;
};

class PHYSICSLIB_API IPhysicsScene
{
public:
	virtual void Release() = 0;
	virtual void Tick(MathLib::HReal deltaTime) = 0;
	virtual bool AddPhysicsObject(PhysicsPtr<IPhysicsObject> &physicsObject) = 0;
	virtual void RemovePhysicsObject(PhysicsPtr<IPhysicsObject> &physicsObject) = 0;
	virtual bool AddJoint(PhysicsPtr<IPhysicsJoint> &joint) = 0;
	virtual void RemoveJoint(PhysicsPtr<IPhysicsJoint> &joint) = 0;
	virtual uint32_t GetPhysicsObjectCount() const = 0;
	virtual uint32_t GetPhysicsRigidDynamicCount() const = 0;
	virtual uint32_t GetPhysicsRigidStaticCount() const = 0;
	virtual uint32_t GetJointCount() const = 0;
	virtual void RaycastSingle(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, PhysicsRaycastHit& hit) = 0;
	virtual void RaycastAll(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, std::vector<PhysicsRaycastHit>& hits) = 0;
	virtual void SetGravity(const MathLib::HVector3& gravity) = 0;
	virtual MathLib::HVector3 GetGravity() const = 0;
	virtual void DebugDraw() = 0;
	virtual size_t GetOffset() const = 0;
	virtual void* GetNativeScene() const = 0;
	virtual void Clear() = 0;
};

class PHYSICSLIB_API IColliderGeometry
{
public:
	virtual void Release() = 0;
	virtual CollierGeometryType GetType() const = 0;
	virtual void SetScale(const MathLib::HVector3 &scale) = 0;
	virtual void GetParams(CollisionGeometryCreateOptions &options) = 0;
	virtual MathLib::HAABBox3D GetBoundingBox()const = 0;
	virtual void SetMaterial(PhysicsPtr<IPhysicsMaterial>& material) = 0;
	virtual PhysicsPtr<IPhysicsMaterial> GetMaterial() const = 0;
	virtual bool RaycastTest(const MathLib::HRay3D& ray, const MathLib::HTransform3& worldTransform, MathLib::HReal& distance, MathLib::HVector3& normal) = 0;
};

class PHYSICSLIB_API IPhysicsObject
{
public:
	virtual void Release() = 0;
	virtual void Update() = 0;
	virtual bool AddColliderGeometry(PhysicsPtr<IColliderGeometry> &colliderGeometry, const MathLib::HTransform3 &localTrans) = 0;
	virtual bool RemoveColliderGeometry(PhysicsPtr<IColliderGeometry> &colliderGeometry) = 0;
	virtual void GetColliderGeometries(std::vector<PhysicsPtr<IColliderGeometry>> &geomeries, std::vector<MathLib::HTransform3> *geoLocalPos = nullptr) = 0;
	virtual PhysicsObjectType GetType() const = 0;
	virtual size_t GetOffset() const = 0;
	virtual void SetTransform(const MathLib::HTransform3 &trans) = 0;
	virtual const MathLib::HTransform3 &GetTransform() const = 0;
	virtual bool IsValid() const = 0;
	virtual MathLib::HAABBox3D GetLocalBoundingBox() const = 0;
	virtual MathLib::HAABBox3D GetWorldBoundingBox() const = 0;
	virtual void SetUserData(void* userData) = 0;
	virtual void* GetUserData() const = 0;
	virtual void SetCollisionLayer(uint32_t layer) = 0;
	virtual uint32_t GetCollisionLayer() const = 0;
	virtual void SetCollisionMask(uint32_t mask) = 0;
	virtual uint32_t GetCollisionMask() const = 0;
	virtual void SetCollisionCallback(ICollisionCallback* callback) = 0;
	virtual ICollisionCallback* GetCollisionCallback() const = 0;
};

class PHYSICSLIB_API IRigidBody : public IPhysicsObject
{
public:
	virtual ~IRigidBody() = default;
	
	virtual PhysicsObjectType GetRigidBodyType() const = 0;
	
	virtual void SetMassProperties(const MathLib::HReal& mass, const MathLib::HVector3& centerOfMass, const MathLib::HMatrix3& inertiaTensor) = 0;
	virtual MathLib::HReal GetMass() const = 0;
	virtual MathLib::HVector3 GetCenterOfMass() const = 0;
	virtual MathLib::HMatrix3 GetInertiaTensor() const = 0;
	
	virtual void SetGravityEnabled(bool enabled) = 0;
	virtual bool IsGravityEnabled() const = 0;
	
	virtual void SetSleepThreshold(const MathLib::HReal& threshold) = 0;
	virtual MathLib::HReal GetSleepThreshold() const = 0;
	virtual bool IsSleeping() const = 0;
	virtual void WakeUp() = 0;
	virtual void PutToSleep() = 0;
	
	virtual void SetFriction(const MathLib::HReal& staticFriction, const MathLib::HReal& dynamicFriction) = 0;
	virtual MathLib::HReal GetStaticFriction() const = 0;
	virtual MathLib::HReal GetDynamicFriction() const = 0;
	virtual void SetRestitution(const MathLib::HReal& restitution) = 0;
	virtual MathLib::HReal GetRestitution() const = 0;
	
	virtual void SetLinearDamping(const MathLib::HReal& damping) = 0;
	virtual MathLib::HReal GetLinearDamping() const = 0;
	virtual void SetAngularDamping(const MathLib::HReal& damping) = 0;
	virtual MathLib::HReal GetAngularDamping() const = 0;
	
	virtual bool IsEmpty() const = 0;

	virtual void* GetNativeActor() const = 0;
};

class PHYSICSLIB_API IRigidDynamic : public IRigidBody
{
public:
	virtual void SetAngularDamping(const MathLib::HReal &damping) = 0;
	virtual void SetLinearDamping(const MathLib::HReal &damping) = 0;
	virtual void SetLinearVelocity(const MathLib::HVector3 &velocity) = 0;
	virtual void SetAngularVelocity(const MathLib::HVector3 &velocity) = 0;
	virtual void SetKinematic(bool bKinematic) = 0;
	virtual void SetMass(const MathLib::HReal &mass) = 0;
	virtual void AddForce(const MathLib::HVector3 &force, ForceMode mode = ForceMode::FORCE) = 0;
	virtual void AddTorque(const MathLib::HVector3 &torque, ForceMode mode = ForceMode::FORCE) = 0;
	virtual void AddForceAtLocalPosition(const MathLib::HVector3 &force, const MathLib::HVector3 &pos, ForceMode mode = ForceMode::FORCE) = 0;
	virtual void AddForceAtPosition(const MathLib::HVector3 &force, const MathLib::HVector3 &pos, ForceMode mode = ForceMode::FORCE) = 0;
	virtual void ClearForce(bool clearVelocity = false) = 0;
	virtual bool IsKinematic() const = 0;
	virtual MathLib::HReal GetMass() const = 0;
	virtual MathLib::HVector3 GetLinearVelocity() const = 0;
	virtual MathLib::HReal GetAngularDamping() const = 0;
	virtual MathLib::HReal GetLinearDamping() const = 0;
	virtual MathLib::HVector3 GetAngularVelocity() const = 0;
	virtual MathLib::HMatrix3 GetInertiaTensor() const = 0;
	virtual bool IsSleeping() const = 0;
	virtual void SetCenterOfMass(const MathLib::HVector3& centerOfMass) = 0;
	virtual MathLib::HVector3 GetCenterOfMass() const = 0;
	virtual void SetGravityEnabled(bool enabled) = 0;
	virtual bool IsGravityEnabled() const = 0;
	virtual void SetSleepThreshold(const MathLib::HReal& threshold) = 0;
	virtual MathLib::HReal GetSleepThreshold() const = 0;
	virtual void EnableGyroscopicForces(bool enable) = 0;
	virtual bool IsGyroscopicForcesEnabled() const = 0;
};

class PHYSICSLIB_API IRigidStatic : public IRigidBody
{
public:
};

class PHYSICSLIB_API ISoftBody : public IPhysicsObject
{
public:
	virtual ~ISoftBody() = default;
	virtual void SetParameter(const SoftBodyParams& params) = 0;
	virtual SoftBodyParams GetParameter() const = 0;
	virtual uint32_t GetVertexCount() const = 0;
	virtual uint32_t GetTetrahedronCount() const = 0;
	virtual void GetDeformedVertexPositions(std::vector<MathLib::HVector3>& positions) const = 0;
	virtual void GetTetrahedronIndices(std::vector<uint32_t>& indices) const = 0;
	virtual void SetVertexFixed(uint32_t vertexIndex, bool fixed) = 0;
	virtual bool IsVertexFixed(uint32_t vertexIndex) const = 0;
	virtual void ApplyForceToVertex(uint32_t vertexIndex, const MathLib::HVector3& force) = 0;
	virtual void SetMass(MathLib::HReal mass) = 0;
	virtual MathLib::HReal GetMass() const = 0;
	virtual void UpdateMass(MathLib::HReal density) = 0;
	virtual void Transform(const MathLib::HTransform3& transform, const MathLib::HVector3& scale = MathLib::HVector3(1.0f, 1.f, 1.f)) = 0;
	virtual void GetOriginalVertexPositions(std::vector<MathLib::HVector3>& positions) const = 0;
	virtual void* GetTetrahedronMesh() const = 0;
	virtual void* GetSoftBodyData() const = 0;
	virtual void Commit() = 0;
	virtual void GetRenderMesh(std::vector<MathLib::HVector3>& vertices, std::vector<uint32_t>& indices) const = 0;
};

class PHYSICSLIB_API ICloth : public IPhysicsObject
{
public:
	virtual ~ICloth() = default;
	virtual void SetParameter(const ClothParams& params) = 0;
	virtual ClothParams GetParameter() const = 0;
	virtual uint32_t GetVertexCount() const = 0;
	virtual uint32_t GetTriangleCount() const = 0;
	virtual void GetDeformedVertexPositions(std::vector<MathLib::HVector3>& positions) const = 0;
	virtual void GetTriangleIndices(std::vector<uint32_t>& indices) const = 0;
	virtual void SetVertexFixed(uint32_t vertexIndex, bool fixed) = 0;
	virtual bool IsVertexFixed(uint32_t vertexIndex) const = 0;
	virtual void ApplyForceToVertex(uint32_t vertexIndex, const MathLib::HVector3& force) = 0;
	virtual void ApplyWindForce(const MathLib::HVector3& windDirection, MathLib::HReal strength) = 0;
	virtual void SetMass(MathLib::HReal mass) = 0;
	virtual MathLib::HReal GetMass() const = 0;
	virtual void GetOriginalVertexPositions(std::vector<MathLib::HVector3>& positions) const = 0;
	virtual void* GetClothData() const = 0;
	virtual void Commit() = 0;
	virtual void GetRenderMesh(std::vector<MathLib::HVector3>& vertices, std::vector<uint32_t>& indices) const = 0;
};

class PHYSICSLIB_API IPhysicsMaterial
{
public:
	virtual void Release() = 0;
	virtual MathLib::HReal GetStaticFriction() const = 0;
	virtual MathLib::HReal GetDynamicFriction() const = 0;
	virtual MathLib::HReal GetRestitution() const = 0;
	virtual MathLib::HReal SetStaticFriction(const MathLib::HReal &value) = 0;
	virtual MathLib::HReal SetDynamicFriction(const MathLib::HReal &value) = 0;
	virtual MathLib::HReal SetRestitution(const MathLib::HReal &value) = 0;
	virtual MathLib::HReal GetDensity() const = 0;
	virtual MathLib::HReal SetDensity(const MathLib::HReal &value) = 0;
	virtual size_t GetOffset() const = 0;
};

class PHYSICSLIB_API IPhysicsJoint
{
public:
	virtual void Release() = 0;
	virtual JointType GetType() const = 0;
	virtual void SetBreakForce(const MathLib::HReal& force) = 0;
	virtual MathLib::HReal GetBreakForce() const = 0;
	virtual void SetBreakTorque(const MathLib::HReal& torque) = 0;
	virtual MathLib::HReal GetBreakTorque() const = 0;
	virtual PhysicsPtr<IPhysicsObject> GetObjectA() const = 0;
	virtual PhysicsPtr<IPhysicsObject> GetObjectB() const = 0;
	virtual void SetLocalPose(bool isObjectA, const MathLib::HTransform3& pose) = 0;
	virtual MathLib::HTransform3 GetLocalPose(bool isObjectA) const = 0;
	virtual bool IsBroken() const = 0;
	virtual size_t GetOffset() const = 0;
	virtual void SetJointLimits(const JointLimitOptions& limitOptions) = 0;
	virtual JointLimitOptions GetJointLimits() const = 0;
	
	virtual void SetDrive(JointAxis axis, const JointDriveSettings& driveSettings) = 0;
	virtual JointDriveSettings GetDrive(JointAxis axis) const = 0;
	virtual void SetDriveConfig(const JointDriveConfig& driveConfig) = 0;
	virtual JointDriveConfig GetDriveConfig() const = 0;
	
	virtual void SetDriveVelocity(JointAxis axis, MathLib::HReal velocity) = 0;
	virtual void SetDrivePosition(JointAxis axis, MathLib::HReal position) = 0;
	virtual void SetDriveForceLimit(JointAxis axis, MathLib::HReal forceLimit) = 0;
	virtual void SetDriveEnabled(JointAxis axis, bool enabled) = 0;
};

class PHYSICSLIB_API IPhysicsDebugRenderer
{
public:
	virtual void Release() = 0;
	virtual void DrawLine(const MathLib::HVector3& start, const MathLib::HVector3& end, const MathLib::HVector3& color) = 0;
	virtual void DrawSphere(const MathLib::HVector3& center, float radius, const MathLib::HVector3& color) = 0;
	virtual void DrawBox(const MathLib::HVector3& center, const MathLib::HVector3& halfExtents, const MathLib::HVector3& color) = 0;
	virtual void DrawCapsule(const MathLib::HVector3& center, float radius, float halfHeight, const MathLib::HVector3& color) = 0;
	virtual void DrawTriangleMesh(const std::vector<MathLib::HVector3>& vertices, const std::vector<uint32_t>& indices, const MathLib::HVector3& color) = 0;
	virtual void DrawConvexMesh(const std::vector<MathLib::HVector3>& vertices, const std::vector<uint32_t>& indices, const MathLib::HVector3& color) = 0;
	virtual void DrawTransform(const MathLib::HTransform3& transform, float size) = 0;
	virtual void Flush() = 0;
};

class PHYSICSLIB_API IPhysicsDebugRender
{
public:
	virtual void Release() = 0;
	virtual void SetVisibleGroup(PhysicsDebugRenderGroupType group) = 0;
	virtual void DrawAABB(const MathLib::HVector3& min, const MathLib::HVector3& max, const MathLib::HVector3& color) = 0;
	virtual void DrawOBB(const MathLib::HVector3& center, const MathLib::HVector3& halfExtents, const MathLib::HTransform3& transform, const MathLib::HVector3& color) = 0;
	virtual void DrawArrow(const MathLib::HVector3& start, const MathLib::HVector3& end, float headSize, const MathLib::HVector3& color) = 0;
	virtual void DrawCircle(const MathLib::HVector3& center, float radius, const MathLib::HVector3& normal, const MathLib::HVector3& color) = 0;
	virtual void DrawCone(const MathLib::HVector3& apex, const MathLib::HVector3& direction, float height, float radius, const MathLib::HVector3& color) = 0;
	virtual void DrawCylinder(const MathLib::HVector3& start, const MathLib::HVector3& end, float radius, const MathLib::HVector3& color) = 0;
	virtual void DrawGrid(const MathLib::HVector3& center, const MathLib::HVector3& normal, float size, uint32_t divisions, const MathLib::HVector3& color) = 0;
	virtual void DrawAxis(const MathLib::HTransform3& transform, float size, bool text = true) = 0;
	virtual void DrawText(const MathLib::HVector3& position, const std::string& text, const MathLib::HVector3& color, float size = 1.0f) = 0;
	virtual void DrawContact(const MathLib::HVector3& position, const MathLib::HVector3& normal, float impulse, const MathLib::HVector3& color) = 0;
	virtual void DrawBoundingSphere(const MathLib::HVector3& center, float radius, const MathLib::HVector3& color) = 0;
	virtual void DrawPoint(const MathLib::HVector3& position, float size, const MathLib::HVector3& color) = 0;
	virtual void SetLineWidth(float width) = 0;
	virtual float GetLineWidth() const = 0;
	virtual void EnableDepthTest(bool enable) = 0;
	virtual bool IsDepthTestEnabled() const = 0;
	virtual void Clear() = 0;
	virtual void Flush() = 0;
};

class IPhysicsContactCallback
{
	public:
	// True to buffer contacts and send them after simulation is completed,
	// false to send contacts as soon as they're available.
	virtual	bool	BufferContacts()		const												= 0;

	enum _
	{
		CONTACT_FOUND	= (1<<0),
		CONTACT_PERSIST	= (1<<1),
		CONTACT_LOST	= (1<<2),
		CONTACT_ALL		= CONTACT_FOUND|CONTACT_PERSIST|CONTACT_LOST
	};
	virtual	uint32_t	GetContactFlags()		const												= 0;

	virtual	float	GetContactThreshold()	const												= 0;
	virtual	void	OnContact(uint32_t nb_contacts, const PhysicsContactData* contacts)	= 0;
};

class PHYSICSLIB_API PhysicsEngineUtils
{
public:
	static IPhysicsEngine *CreatePhysicsEngine(const PhysicsEngineOptions &options, const bool createConvexDecomposer = true);
	static void DestroyPhysicsEngine();
	static IPhysicsEngine *GetPhysicsEngine();
	static PhysicsPtr<IPhysicsObject> CreateObject(const PhysicsObjectCreateOptions &options);
	static PhysicsPtr<IPhysicsMaterial> CreateMaterial(const PhysicsMaterialCreateOptions &options);
	static PhysicsPtr<IPhysicsScene> CreateScene(const PhysicsSceneCreateOptions &options);
	static PhysicsPtr<IColliderGeometry> CreateColliderGeometry(const CollisionGeometryCreateOptions &options);
	static PhysicsPtr<IPhysicsJoint> CreateJoint(const JointCreateOptions &options);
	static PhysicsPtr<ISoftBody> CreateSoftBody(const SoftBodyCreateOptions &options);
	
	static PhysicsPtr<IRigidStatic> CreateRigidStatic(const PhysicsObjectCreateOptions &options);
	static PhysicsPtr<IRigidDynamic> CreateRigidDynamic(const PhysicsObjectCreateOptions &options);

	static IPhysicsProfiler* GetProfiler();
	
	static void BuildConvexMesh(const std::vector<MathLib::HVector3> &vertices, const std::vector<uint32_t> &indices, PhysicsMeshData &meshdata);
	static bool ConvexDecomposition(const PhysicsMeshData &meshData, const ConvexDecomposeOptions &params, std::vector<PhysicsMeshData> &convexMeshesData);
	
	static bool RaycastSingle(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, PhysicsRaycastHit& hit);
	static bool RaycastAll(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, std::vector<PhysicsRaycastHit>& hits);
	static bool SweepTest(PhysicsPtr<IColliderGeometry> geometry, const MathLib::HTransform3& startTransform, 
						 const MathLib::HVector3& direction, MathLib::HReal maxDistance, PhysicsRaycastHit& hit);
	static bool BoxOverlap(const MathLib::HVector3& center, const MathLib::HVector3& halfExtents, 
						  std::vector<PhysicsPtr<IPhysicsObject>>& overlappingObjects);
	static bool SphereOverlap(const MathLib::HVector3& center, MathLib::HReal radius, 
							 std::vector<PhysicsPtr<IPhysicsObject>>& overlappingObjects);
	
	static void EnableDebugDrawing(bool enable);
	static bool IsDebugDrawingEnabled();
	static void SetDebugRenderer(PhysicsPtr<IPhysicsDebugRenderer> renderer);
	static PhysicsPtr<IPhysicsDebugRenderer> GetDebugRenderer();
	
	static void SetDebugRender(PhysicsPtr<IPhysicsDebugRender> render);
	static PhysicsPtr<IPhysicsDebugRender> GetDebugRender();
	
	static void SetCollisionFilterCallback(std::function<bool(uint32_t, uint32_t)> callback);
	static bool DefaultCollisionFilter(uint32_t layerA, uint32_t layerB);

	// Articulation utility functions
	static PhysicsPtr<IArticulation> CreateArticulation(const ArticulationCreateOptions &options);
	static PhysicsPtr<IArticulationLink> CreateArticulationLink(PhysicsPtr<IArticulation> articulation, PhysicsPtr<IArticulationLink> parent, const ArticulationLinkCreateOptions &options);
	static PhysicsPtr<IArticulationJoint> CreateArticulationJoint(PhysicsPtr<IArticulationLink> link, const ArticulationJointCreateOptions &options);
	static PhysicsPtr<IArticulationCache> CreateArticulationCache(PhysicsPtr<IArticulation> articulation);
};

class PHYSICSLIB_API IArticulationCache
{
public:
	virtual ~IArticulationCache() = default;
	virtual void Release() = 0;
	
	// Data access interface
	virtual MathLib::HReal* GetJointPositions() = 0;
	virtual const MathLib::HReal* GetJointPositions() const = 0;
	virtual MathLib::HReal* GetJointVelocities() = 0;
	virtual const MathLib::HReal* GetJointVelocities() const = 0;
	virtual MathLib::HReal* GetJointAccelerations() = 0;
	virtual const MathLib::HReal* GetJointAccelerations() const = 0;
	virtual MathLib::HReal* GetJointForces() = 0;
	virtual const MathLib::HReal* GetJointForces() const = 0;
	
	virtual ArticulationSpatialVelocity* GetLinkVelocities() = 0;
	virtual const ArticulationSpatialVelocity* GetLinkVelocities() const = 0;
	virtual ArticulationSpatialVelocity* GetLinkAccelerations() = 0;
	virtual const ArticulationSpatialVelocity* GetLinkAccelerations() const = 0;
	
	virtual ArticulationRootLinkData* GetRootLinkData() = 0;
	virtual const ArticulationRootLinkData* GetRootLinkData() const = 0;
	
	virtual ArticulationSpatialForce* GetExternalForces() = 0;
	virtual const ArticulationSpatialForce* GetExternalForces() const = 0;
	
	// Advanced computation data
	virtual MathLib::HReal* GetMassMatrix() = 0;
	virtual const MathLib::HReal* GetMassMatrix() const = 0;
	virtual MathLib::HReal* GetJacobian() = 0;
	virtual const MathLib::HReal* GetJacobian() const = 0;
};

class PHYSICSLIB_API IArticulationJoint
{
public:
	virtual ~IArticulationJoint() = default;
	
	// Basic properties
	virtual ArticulationJointType GetJointType() const = 0;
	virtual void SetParentPose(const MathLib::HTransform3& pose) = 0;
	virtual MathLib::HTransform3 GetParentPose() const = 0;
	virtual void SetChildPose(const MathLib::HTransform3& pose) = 0;
	virtual MathLib::HTransform3 GetChildPose() const = 0;
	
	// Motion configuration
	virtual void SetMotion(ArticulationAxis axis, ArticulationMotion motion) = 0;
	virtual ArticulationMotion GetMotion(ArticulationAxis axis) const = 0;
	virtual uint32_t GetDofCount() const = 0;
	
	// Limit settings
	virtual void SetLimit(ArticulationAxis axis, const ArticulationLimit& limit) = 0;
	virtual ArticulationLimit GetLimit(ArticulationAxis axis) const = 0;
	
	// Drive settings
	virtual void SetDrive(ArticulationAxis axis, const ArticulationDrive& drive) = 0;
	virtual ArticulationDrive GetDrive(ArticulationAxis axis) const = 0;
	virtual void SetDriveTarget(ArticulationAxis axis, MathLib::HReal target) = 0;
	virtual MathLib::HReal GetDriveTarget(ArticulationAxis axis) const = 0;
	virtual void SetDriveVelocity(ArticulationAxis axis, MathLib::HReal velocity) = 0;
	virtual MathLib::HReal GetDriveVelocity(ArticulationAxis axis) const = 0;
	
	// Joint position and velocity
	virtual void SetJointPosition(ArticulationAxis axis, MathLib::HReal position) = 0;
	virtual MathLib::HReal GetJointPosition(ArticulationAxis axis) const = 0;
	virtual void SetJointVelocity(ArticulationAxis axis, MathLib::HReal velocity) = 0;
	virtual MathLib::HReal GetJointVelocity(ArticulationAxis axis) const = 0;
	
	// Physical properties
	virtual void SetFrictionCoefficient(MathLib::HReal coefficient) = 0;
	virtual MathLib::HReal GetFrictionCoefficient() const = 0;
	virtual void SetMaxJointVelocity(MathLib::HReal maxVelocity) = 0;
	virtual MathLib::HReal GetMaxJointVelocity() const = 0;
	
	virtual size_t GetOffset() const = 0;
};

class PHYSICSLIB_API IArticulationLink : public IPhysicsObject
{
public:
	virtual ~IArticulationLink() = default;
	
	// Link hierarchy
	virtual IArticulation* GetArticulation() const = 0;
	virtual IArticulationJoint* GetInboundJoint() const = 0;
	virtual uint32_t GetInboundJointDof() const = 0;
	virtual uint32_t GetLinkIndex() const = 0;
	
	// Child link management
	virtual uint32_t GetNbChildren() const = 0;
	virtual uint32_t GetChildren(std::vector<PhysicsPtr<IArticulationLink>>& children) const = 0;
	
	// Mass properties
	virtual void SetMass(MathLib::HReal mass) = 0;
	virtual MathLib::HReal GetMass() const = 0;
	virtual MathLib::HReal GetInvMass() const = 0;
	virtual void SetCMassLocalPose(const MathLib::HTransform3& pose) = 0;
	virtual MathLib::HTransform3 GetCMassLocalPose() const = 0;
	virtual void SetMassSpaceInertiaTensor(const MathLib::HVector3& inertia) = 0;
	virtual MathLib::HVector3 GetMassSpaceInertiaTensor() const = 0;
	virtual MathLib::HVector3 GetMassSpaceInvInertiaTensor() const = 0;
	
	// Damping
	virtual void SetLinearDamping(MathLib::HReal damping) = 0;
	virtual MathLib::HReal GetLinearDamping() const = 0;
	virtual void SetAngularDamping(MathLib::HReal damping) = 0;
	virtual MathLib::HReal GetAngularDamping() const = 0;
	
	// Velocity limits
	virtual void SetMaxLinearVelocity(MathLib::HReal maxVel) = 0;
	virtual MathLib::HReal GetMaxLinearVelocity() const = 0;
	virtual void SetMaxAngularVelocity(MathLib::HReal maxVel) = 0;
	virtual MathLib::HReal GetMaxAngularVelocity() const = 0;
	
	// Velocity and acceleration
	virtual MathLib::HVector3 GetLinearVelocity() const = 0;
	virtual MathLib::HVector3 GetAngularVelocity() const = 0;
	virtual MathLib::HVector3 GetLinearAcceleration() const = 0;
	virtual MathLib::HVector3 GetAngularAcceleration() const = 0;
	
	// Force application
	virtual void AddForce(const MathLib::HVector3& force, ForceMode mode = ForceMode::FORCE, bool autowake = true) = 0;
	virtual void AddTorque(const MathLib::HVector3& torque, ForceMode mode = ForceMode::FORCE, bool autowake = true) = 0;
	virtual void ClearForce(ForceMode mode = ForceMode::FORCE) = 0;
	virtual void ClearTorque(ForceMode mode = ForceMode::FORCE) = 0;
	virtual void SetForceAndTorque(const MathLib::HVector3& force, const MathLib::HVector3& torque, ForceMode mode = ForceMode::FORCE) = 0;
	
	// CCD and physical properties
	virtual void SetMinCCDAdvanceCoefficient(MathLib::HReal coefficient) = 0;
	virtual MathLib::HReal GetMinCCDAdvanceCoefficient() const = 0;
	virtual void SetMaxDepenetrationVelocity(MathLib::HReal maxVel) = 0;
	virtual MathLib::HReal GetMaxDepenetrationVelocity() const = 0;
	virtual void SetMaxContactImpulse(MathLib::HReal maxImpulse) = 0;
	virtual MathLib::HReal GetMaxContactImpulse() const = 0;
	virtual void SetContactSlopCoefficient(MathLib::HReal coefficient) = 0;
	virtual MathLib::HReal GetContactSlopCoefficient() const = 0;
	
	// CFM (Constraint Force Mixing)
	virtual void SetCfmScale(MathLib::HReal cfm) = 0;
	virtual MathLib::HReal GetCfmScale() const = 0;
	
	virtual void* GetNativeLink() const = 0;
};

class PHYSICSLIB_API IArticulation
{
public:
	virtual ~IArticulation() = default;
	virtual void Release() = 0;
	
	// Basic properties
	virtual PhysicsPtr<IPhysicsScene> GetScene() const = 0;
	virtual bool IsFixedBase() const = 0;
	virtual void SetSolverIterationCounts(uint32_t minPositionIters, uint32_t minVelocityIters = 1) = 0;
	virtual void GetSolverIterationCounts(uint32_t& minPositionIters, uint32_t& minVelocityIters) const = 0;
	
	// Link management
	virtual PhysicsPtr<IArticulationLink> CreateLink(PhysicsPtr<IArticulationLink> parent, const ArticulationLinkCreateOptions& options) = 0;
	virtual uint32_t GetNbLinks() const = 0;
	virtual uint32_t GetLinks(std::vector<PhysicsPtr<IArticulationLink>>& links) const = 0;
	virtual PhysicsPtr<IArticulationLink> GetRootLink() const = 0;
	
	// Joint management
	virtual uint32_t GetDofs() const = 0;
	virtual void UpdateKinematic() = 0;
	
	// Cache management
	virtual PhysicsPtr<IArticulationCache> CreateCache() = 0;
	virtual void ApplyCache(PhysicsPtr<IArticulationCache> cache, uint32_t flag) = 0;
	virtual void CopyInternalStateToCache(PhysicsPtr<IArticulationCache> cache, uint32_t flag) const = 0;
	
	// Projection and separation settings
	virtual void SetMaxProjectionIterations(uint32_t iterations) = 0;
	virtual uint32_t GetMaxProjectionIterations() const = 0;
	virtual void SetSeparationTolerance(MathLib::HReal tolerance) = 0;
	virtual MathLib::HReal GetSeparationTolerance() const = 0;
	
	// Drive iteration settings
	virtual void SetInternalDriveIterations(uint32_t iterations) = 0;
	virtual uint32_t GetInternalDriveIterations() const = 0;
	virtual void SetExternalDriveIterations(uint32_t iterations) = 0;
	virtual uint32_t GetExternalDriveIterations() const = 0;
	
	// Sleep and wake
	virtual bool IsSleeping() const = 0;
	virtual void SetSleepThreshold(MathLib::HReal threshold) = 0;
	virtual MathLib::HReal GetSleepThreshold() const = 0;
	virtual void SetStabilizationThreshold(MathLib::HReal threshold) = 0;
	virtual MathLib::HReal GetStabilizationThreshold() const = 0;
	virtual void SetWakeCounter(MathLib::HReal wakeCounterValue) = 0;
	virtual MathLib::HReal GetWakeCounter() const = 0;
	virtual void WakeUp() = 0;
	virtual void PutToSleep() = 0;
	
	// Advanced computation functions
	virtual void CommonInit() = 0;
	virtual void ComputeGeneralizedGravityForce(PhysicsPtr<IArticulationCache> cache) = 0;
	virtual void ComputeCoriolisAndCentrifugalForce(PhysicsPtr<IArticulationCache> cache) = 0;
	virtual void ComputeGeneralizedExternalForce(PhysicsPtr<IArticulationCache> cache) = 0;
	virtual void ComputeJointAcceleration(PhysicsPtr<IArticulationCache> cache) = 0;
	virtual void ComputeJointForce(PhysicsPtr<IArticulationCache> cache) = 0;
	virtual void ComputeGeneralizedMassMatrix(PhysicsPtr<IArticulationCache> cache) = 0;
	virtual void ComputeDenseJacobian(PhysicsPtr<IArticulationCache> cache, uint32_t& nRows, uint32_t& nCols) = 0;
	
	// Name and bounds
	virtual void SetName(const char* name) = 0;
	virtual const char* GetName() const = 0;
	virtual MathLib::HAABBox3D GetWorldBounds(MathLib::HReal inflation = 1.01f) const = 0;
	
	// User data and internal index
	virtual void SetUserData(void* userData) = 0;
	virtual void* GetUserData() const = 0;
	virtual uint32_t GetInternalActorIndex() const = 0;
	
	virtual size_t GetOffset() const = 0;
	virtual void* GetNativeArticulation() const = 0;
};
