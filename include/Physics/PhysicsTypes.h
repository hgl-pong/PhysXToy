#pragma once
#include <Math/MathUtils.h>
#include <Math/GraphicUtils/MeshData.h>
#define DEFAULT_CPU_DISPATCHER_NUM_THREADS 2
#define DEFAULT_SOLVER_ITERATION_COUNT 6

#define PHYSICS_INLINE inline
#define PHYSICS_FORCE_INLINE __forceinline

template <typename T>
struct PhysicsDeleter
{
	void operator()(T *ptr) const
	{
		if (ptr)
		{
			ptr->Release();
			ptr = nullptr;
		}
	}
};

template <typename T>
using PhysicsPtr = std::shared_ptr<T>;

template <typename T>
PhysicsPtr<T> make_physics_ptr(T *ptr)
{
	return PhysicsPtr<T>(ptr, PhysicsDeleter<T>());
}

template <typename T>
struct PhysXDeleter
{
	void operator()(T *ptr) const
	{
		if (ptr)
		{
			ptr->release();
			ptr = nullptr;
		}
	}
};

template <typename T>
using PhysXPtr = std::unique_ptr<T, PhysXDeleter<T>>;

template <typename T>
PhysXPtr<T> make_physx_ptr(T *ptr)
{
	return PhysXPtr<T>(ptr);
}

namespace MathLib {
    class HRay3D
    {
    public:
        HRay3D() : m_Origin(0, 0, 0), m_Direction(0, 1, 0) {}
        HRay3D(const HVector3& origin, const HVector3& direction) 
            : m_Origin(origin), m_Direction(direction.normalized()) {}
        
        const HVector3& GetOrigin() const { return m_Origin; }
        const HVector3& GetDirection() const { return m_Direction; }
        
        HVector3 GetPoint(HReal t) const { return m_Origin + m_Direction * t; }
        
    private:
        HVector3 m_Origin;
        HVector3 m_Direction;
    };
}

struct PhysicsEngineOptions
{
	uint32_t m_NumThreads = DEFAULT_CPU_DISPATCHER_NUM_THREADS;
	bool m_bEnablePVD = true;
	uint32_t m_SolverIterationCount = DEFAULT_SOLVER_ITERATION_COUNT;
    bool m_EnableCCD = false;               
    bool m_EnableDebugVisualization = false;
    bool m_EnableProfiler = false;
};

enum class PhysicsSceneFilterShaderType
{
	eDEFAULT
};

struct PhysicsSceneCreateOptions
{
	MathLib::HVector3 m_Gravity;
	PhysicsSceneFilterShaderType m_FilterShaderType;
    uint32_t m_MaxSubSteps = 3;                      
    MathLib::HReal m_FixedTimeStep = 1.0f / 60.0f;   
};

struct PhysicsMaterialCreateOptions
{
	MathLib::HReal m_StaticFriction = 0.5;
	MathLib::HReal m_DynamicFriction = 0.5;
	MathLib::HReal m_Restitution = 0.6;
	MathLib::HReal m_Density = 10;
};

enum class CollierGeometryType
{
	COLLIER_GEOMETRY_TYPE_SPHERE,
	COLLIER_GEOMETRY_TYPE_BOX,
	COLLIER_GEOMETRY_TYPE_CAPSULE,
	COLLIER_GEOMETRY_TYPE_PLANE,
	COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH,
	COLLIER_GEOMETRY_TYPE_CONVEX_MESH,
	COLLIER_GEOMETRY_TYPE_HEIGHT_FIELD,
	COLLIER_GEOMETRY_TYPE_COUNT
};

struct CollisionGeometryCreateOptions
{
	CollierGeometryType m_GeometryType;
	MathLib::HTransform3 m_LocalTransform;
	struct SphereParams
	{
		MathLib::HReal m_Radius = 1;
	} m_SphereParams;
	struct BoxParams
	{
		MathLib::HVector3 m_HalfExtents = {1, 1, 1};
	} m_BoxParams;
	struct CapsuleParams
	{
		MathLib::HReal m_Radius = 1;
		MathLib::HReal m_HalfHeight = 1;
	} m_CapsuleParams;
	struct PlaneParams
	{
		MathLib::HVector3 m_Normal = {0, 1, 0};
		MathLib::HReal m_Distance = 0;
	} m_PlaneParams;
	struct TriangleMeshParams
	{
		std::vector<MathLib::HVector3> m_Vertices;
		std::vector<uint32_t> m_Indices;
	} m_TriangleMeshParams;
	struct ConvexMeshParams
	{
		std::vector<MathLib::HVector3> m_Vertices;
		std::vector<uint32_t> m_Indices;
	} m_ConvexMeshParams;
	struct HeightFieldParams
	{
		std::vector<MathLib::HReal> m_HeightData;
		int m_Rows = 0;
		int m_Columns = 0;
		MathLib::HReal m_RowScale = 1.0f;
		MathLib::HReal m_ColumnScale = 1.0f;
		MathLib::HReal m_HeightScale = 1.0f;
	} m_HeightFieldParams;
	MathLib::HVector3 m_Scale = {1, 1, 1};
};

enum class PhysicsObjectType
{
	PHYSICS_OBJECT_TYPE_RIGID_STATIC,
	PHYSICS_OBJECT_TYPE_RIGID_DYNAMIC,
	PHYSICS_OBJECT_TYPE_SOFT_BODY,
	PHYSICS_OBJECT_TYPE_CLOTH,
	PHYSICS_OBJECT_TYPE_COUNT
};

struct PhysicsObjectCreateOptions
{
	PhysicsObjectType m_ObjectType;
	MathLib::HTransform3 m_Transform;
	PhysicsMaterialCreateOptions m_MaterialOptions;
    uint32_t m_CollisionLayer = 1;      
    uint32_t m_CollisionMask = 0xFFFFFFFF; 
    bool m_EnableCCD = false;           
    MathLib::HReal m_LinearDamping = 0.1f;  
    MathLib::HReal m_AngularDamping = 0.1f; 
    MathLib::HReal m_Mass = 1.0f;           
    bool m_IsKinematic = false;        
    bool m_EnableGravity = true;       
    void* m_UserData = nullptr;        
};

typedef MathLib::GraphicUtils::MeshData32 PhysicsMeshData;

struct ConvexDecomposeOptions
{
	uint32_t m_MaximumNumberOfHulls = 8;			// Maximum number of convex hull generated
	uint32_t m_MaximumNumberOfVerticesPerHull = 64; // (default=64, range=4-1024)
	uint32_t m_VoxelGridResolution = 1000000;		//(default=1,000,000, range=10,000-16,000,000).
	MathLib::HReal m_Concavity = 0.0025f;			// Value between 0 and 1
};

enum class QueryFilterFlag
{
    STATIC = (1 << 0),      
    DYNAMIC = (1 << 1),     
    KINEMATIC = (1 << 2),   
    DEBRIS = (1 << 3),      
    TRIGGER = (1 << 4),     
    CHARACTER = (1 << 5),   
    ALL = 0xFFFFFFFF        
};

struct RaycastOptions
{
    uint32_t m_FilterMask = static_cast<uint32_t>(QueryFilterFlag::ALL); 
    bool m_HitBackFaces = false;                      
    bool m_HitTriggers = false;                       
    MathLib::HReal m_MaxDistance = FLT_MAX;           
};

struct JointLimitOptions
{
    struct LinearLimit
    {
        MathLib::HReal m_LowerLimit = 0;              
        MathLib::HReal m_UpperLimit = 0;              
        MathLib::HReal m_Stiffness = 0;               
        MathLib::HReal m_Damping = 0;                 
        bool m_IsLimited = false;                     
    };
    
    struct AngularLimit
    {
        MathLib::HReal m_LowerLimit = 0;              
        MathLib::HReal m_UpperLimit = 0;              
        MathLib::HReal m_Stiffness = 0;               
        MathLib::HReal m_Damping = 0;                 
        bool m_IsLimited = false;                     
    };
    
    LinearLimit m_XAxis;                             
    LinearLimit m_YAxis;                             
    LinearLimit m_ZAxis;                             
    AngularLimit m_Twist;                            
    AngularLimit m_Swing1;                           
    AngularLimit m_Swing2;                           
};

struct SoftBodyParams
{
    MathLib::HReal m_YoungModulus = 1e+9f;      
    MathLib::HReal m_PoissonRatio = 0.45f;      
    MathLib::HReal m_Damping = 0.5f;            
    MathLib::HReal m_MaxInvMassRatio = 10.0f;    
    MathLib::HReal m_Mass = 1.0f;                
    MathLib::HReal m_Density = 10.0f;            
    uint32_t m_SolverIterations = 10;            
    bool m_EnableCCD = false;                   
};

struct ClothParams
{
    MathLib::HReal m_Stiffness = 1.0f;           
    MathLib::HReal m_BendingStiffness = 0.5f;    
    MathLib::HReal m_Damping = 0.1f;             
    MathLib::HReal m_Mass = 1.0f;                
    MathLib::HReal m_WindStrength = 0.0f;        
    MathLib::HVector3 m_WindDirection = {0.0f, 0.0f, 1.0f};
    uint32_t m_SolverIterations = 5;             
    MathLib::HReal m_Friction = 0.5f;            
    bool m_EnableSelfCollision = false;          
    bool m_EnableCCD = false;                    
};

struct SoftBodyMeshDesc
{
    std::vector<MathLib::HVector3> m_Vertices;           
    std::vector<uint32_t> m_Indices;                     
    std::vector<uint32_t> m_FixedVertices;               
    bool m_IsTetrahedronMesh = true;                     
};

struct SoftBodyCreateOptions
{
    SoftBodyMeshDesc m_MeshDesc;                   
    MathLib::HTransform3 m_Transform;              
    SoftBodyParams m_Params;                       
    bool m_CreateCollisionMesh = true;             
    bool m_CreateSimulationMesh = true;            
    uint32_t m_VoxelResolution = 20;               
    PhysicsMaterialCreateOptions m_MaterialOptions;
    uint32_t m_CollisionLayer = 1;                 
    uint32_t m_CollisionMask = 0xFFFFFFFF;         
    void* m_UserData = nullptr;                    
};

struct ClothMeshDesc
{
    std::vector<MathLib::HVector3> m_Vertices;     
    std::vector<uint32_t> m_Indices;              
    std::vector<uint32_t> m_FixedVertices;         
    int m_Width = 0;                               
    int m_Height = 0;                              
    MathLib::HReal m_ParticleSpacing = 1.0f;       
};

struct ClothCreateOptions
{
    ClothMeshDesc m_MeshDesc;                     
    MathLib::HTransform3 m_Transform;             
    ClothParams m_Params;                         
    PhysicsMaterialCreateOptions m_MaterialOptions;
    uint32_t m_CollisionLayer = 1;                
    uint32_t m_CollisionMask = 0xFFFFFFFF;        
    void* m_UserData = nullptr;                   
};

enum class JointType
{
	FIXED,
	DISTANCE,
	SPHERICAL,
	REVOLUTE,
	PRISMATIC,
	D6,
    HINGE,
    GEAR,
    RACK_AND_PINION,
    CHAIN,
    PORTAL,
    COUNT
};

class IPhysicsObject;

struct PhysicsContactData
{
    IPhysicsObject* m_ObjectA;
    IPhysicsObject* m_ObjectB;
    MathLib::HVector3 m_ContactPoint;
    MathLib::HVector3 m_ContactNormal;
    MathLib::HVector3 m_Impulse;
    size_t m_InternalFaceIndexA;
    size_t m_InternalFaceIndexB;
    MathLib::HReal m_Separation;
};

enum PhysicsContactFlags
{
    CONTACT_FOUND	= (1<<0),
    CONTACT_PERSIST	= (1<<1),
    CONTACT_LOST	= (1<<2),
    CONTACT_ALL		= CONTACT_FOUND|CONTACT_PERSIST|CONTACT_LOST
};

struct PhysicsLimits
{
    PhysicsLimits(MathLib::HReal m=0.0f, MathLib::HReal M=0.0f) : m_MinValue(m), m_MaxValue(M)								{}
    PhysicsLimits(const PhysicsLimits& limits) : m_MinValue(limits.m_MinValue), m_MaxValue(limits.m_MaxValue)	{}

    void Set(MathLib::HReal m, MathLib::HReal M)
    {
        m_MinValue = m;
        m_MaxValue = M;
    }

    MathLib::HReal	m_MinValue;
    MathLib::HReal	m_MaxValue;
};

struct PhysicsSpring
{
	PhysicsSpring(MathLib::HReal s=0.0f, MathLib::HReal d=0.0f) : m_Stiffness(s), m_Damping(d)	{}

	MathLib::HReal	m_Stiffness;
	MathLib::HReal	m_Damping;
};

struct PhysicsHingeDynamicData
{
	MathLib::HReal	m_TwistAngle;
};

struct PhysicsD6DynamicData
{
	MathLib::HReal	m_TwistAngle;
	MathLib::HReal	m_SwingYAngle;
	MathLib::HReal	m_SwingZAngle;
};

enum class PhysicsVehicleDifferential
{
    DIFFERENTIAL_LS_4WD,		
    DIFFERENTIAL_LS_FRONTWD,	
    DIFFERENTIAL_LS_REARWD,	
    DIFFERENTIAL_OPEN_4WD,		
    DIFFERENTIAL_OPEN_FRONTWD,	
    DIFFERENTIAL_OPEN_REARWD,	
    DIFFERENTIAL_UNDEFINED
};

