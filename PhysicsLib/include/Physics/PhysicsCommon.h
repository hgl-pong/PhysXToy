#pragma once
#include "Physics/PhysicsTypes.h"
#include "PhysicsMacro.h"
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

enum class ForceMode
{
	FORCE,          
	IMPULSE,        
	VELOCITY_CHANGE,
	ACCELERATION    
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

struct RaycastHit
{
	PhysicsPtr<IPhysicsObject> object;
	MathLib::HVector3 position;
	MathLib::HVector3 normal;
	MathLib::HReal distance;
	PhysicsPtr<IColliderGeometry> collider;
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
	virtual IPhysicsProfiler* GetProfiler() = 0;

	virtual void SetSolverIterationCount(uint32_t count) = 0;
	virtual uint32_t GetSolverIterationCount() const = 0;
	virtual void SetDebugRenderer(PhysicsPtr<IPhysicsDebugRenderer> renderer) = 0;
	virtual PhysicsPtr<IPhysicsDebugRenderer> GetDebugRenderer() const = 0;
	virtual void RegisterCollisionCallback(ICollisionCallback* callback) = 0;
	virtual void UnregisterCollisionCallback(ICollisionCallback* callback) = 0;
	virtual PhysicsPtr<IPhysicsScene> GetActiveScene() const = 0;
	virtual void SetActiveScene(PhysicsPtr<IPhysicsScene> scene) = 0;
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
	virtual void RaycastSingle(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, RaycastHit& hit) = 0;
	virtual void RaycastAll(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, std::vector<RaycastHit>& hits) = 0;
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
	
	static bool RaycastSingle(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, RaycastHit& hit);
	static bool RaycastAll(const MathLib::HRay3D& ray, MathLib::HReal maxDistance, std::vector<RaycastHit>& hits);
	static bool SweepTest(PhysicsPtr<IColliderGeometry> geometry, const MathLib::HTransform3& startTransform, 
						 const MathLib::HVector3& direction, MathLib::HReal maxDistance, RaycastHit& hit);
	static bool BoxOverlap(const MathLib::HVector3& center, const MathLib::HVector3& halfExtents, 
						  std::vector<PhysicsPtr<IPhysicsObject>>& overlappingObjects);
	static bool SphereOverlap(const MathLib::HVector3& center, MathLib::HReal radius, 
							 std::vector<PhysicsPtr<IPhysicsObject>>& overlappingObjects);
	
	static void EnableDebugDrawing(bool enable);
	static bool IsDebugDrawingEnabled();
	static void SetDebugRenderer(PhysicsPtr<IPhysicsDebugRenderer> renderer);
	static PhysicsPtr<IPhysicsDebugRenderer> GetDebugRenderer();
	
	static void SetCollisionFilterCallback(std::function<bool(uint32_t, uint32_t)> callback);
	static bool DefaultCollisionFilter(uint32_t layerA, uint32_t layerB);
};
