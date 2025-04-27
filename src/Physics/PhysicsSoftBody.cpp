#include "PhysicsSoftBody.h"
#include "PhysicsEngine.h"
#include "physx/extensions/PxSoftBodyExt.h"
#include "physx/PxSoftBody.h"
#include "physx/PxShape.h"
#include "physx/cooking/PxCooking.h"
#include "physx/foundation/PxArray.h"
#include "PhysicsMaterial.h"
#include "PxPhysicsAPI.h"
#include "Utility/PhysXUtils.h"

PhysicsSoftBody::PhysicsSoftBody(PhysicsPtr<IPhysicsMaterial>& material)
    : m_Material(material)
{
    m_Type = PhysicsObjectType::PHYSICS_OBJECT_TYPE_SOFT_BODY;
}

PhysicsSoftBody::~PhysicsSoftBody()
{
    Release();
}

void PhysicsSoftBody::Release()
{
    m_SoftBody = nullptr;
    m_SimulationMesh = nullptr;
    m_CollisionMesh = nullptr;
    m_SoftBodyMesh = nullptr;
    m_FEMMaterial = nullptr;
    m_ColliderGeometries.clear();
    m_ColliderLocalPos.clear();
    m_FixedVertices.clear();
    m_OriginalVertices.clear();
    m_TetrahedronIndices.clear();
}

void PhysicsSoftBody::Update()
{
}

bool PhysicsSoftBody::AddColliderGeometry(PhysicsPtr<IColliderGeometry>& colliderGeometry, const MathLib::HTransform3& localTrans)
{
    return false;
}

bool PhysicsSoftBody::RemoveColliderGeometry(PhysicsPtr<IColliderGeometry>& colliderGeometry)
{
    return false;
}

void PhysicsSoftBody::GetColliderGeometries(std::vector<PhysicsPtr<IColliderGeometry>>& geomeries, std::vector<MathLib::HTransform3>* geoLocalPos)
{
    geomeries = m_ColliderGeometries;
    if (geoLocalPos)
        *geoLocalPos = m_ColliderLocalPos;
}

size_t PhysicsSoftBody::GetOffset() const
{
    return reinterpret_cast<size_t>(m_SoftBody.get());
}

void PhysicsSoftBody::SetTransform(const MathLib::HTransform3& trans)
{
    m_Transform = trans;
    if (m_SoftBody)
    {
        Transform(trans);
    }
}

MathLib::HAABBox3D PhysicsSoftBody::GetWorldBoundingBox() const
{
    MathLib::HAABBox3D worldBBox = m_BoundingBox;
    MathLib::HVector3 min = worldBBox.min();
    MathLib::HVector3 max = worldBBox.max();
    min = m_Transform * min;
    max = m_Transform * max;
    
    MathLib::HAABBox3D transformedBox;
    transformedBox.extend(min);
    transformedBox.extend(max);
    return transformedBox;
}

void PhysicsSoftBody::SetParameter(const SoftBodyParams& params)
{
    m_Params = params;
    
    if (m_SoftBody)
    {
        physx::PxRigidDynamic* rigidBody = reinterpret_cast<physx::PxRigidDynamic*>(m_SoftBody.get());
        if (rigidBody)
        {
            rigidBody->setSolverIterationCounts(params.m_SolverIterations, 1);
        }
        
        SetMass(params.m_Mass);
    }
}

uint32_t PhysicsSoftBody::GetVertexCount() const
{
    if (m_SimulationMesh)
    {
        return static_cast<uint32_t>(m_SimulationMesh->getNbVertices());
    }
    return 0;
}

uint32_t PhysicsSoftBody::GetTetrahedronCount() const
{
    if (m_SimulationMesh)
    {
        return static_cast<uint32_t>(m_SimulationMesh->getNbTetrahedrons());
    }
    return 0;
}

void PhysicsSoftBody::GetDeformedVertexPositions(std::vector<MathLib::HVector3>& positions) const
{
    if (!m_SoftBody || !m_SimulationMesh)
    {
        positions.clear();
        return;
    }
    
    positions = m_OriginalVertices;
}

void PhysicsSoftBody::GetTetrahedronIndices(std::vector<uint32_t>& indices) const
{
    indices = m_TetrahedronIndices;
}

void PhysicsSoftBody::SetVertexFixed(uint32_t vertexIndex, bool fixed)
{
    if (vertexIndex >= m_FixedVertices.size())
    {
        return;
    }
    
    m_FixedVertices[vertexIndex] = fixed;
}

bool PhysicsSoftBody::IsVertexFixed(uint32_t vertexIndex) const
{
    if (vertexIndex < m_FixedVertices.size())
    {
        return m_FixedVertices[vertexIndex];
    }
    return false;
}

void PhysicsSoftBody::ApplyForceToVertex(uint32_t vertexIndex, const MathLib::HVector3& force)
{
    if (!m_SoftBody || vertexIndex >= GetVertexCount())
    {
        return;
    }
}

void PhysicsSoftBody::SetMass(MathLib::HReal mass)
{
    if (m_SoftBody)
    {
        m_Params.m_Mass = mass;
        
        // 设置刚体的质量
        physx::PxRigidDynamic* rigidBody = reinterpret_cast<physx::PxRigidDynamic*>(m_SoftBody.get());
        if (rigidBody)
        {
            rigidBody->setMass(mass);
        }
    }
}

void PhysicsSoftBody::UpdateMass(MathLib::HReal density)
{
    if (m_SoftBody)
    {
        m_Params.m_Density = density;
        
        // 计算并设置刚体的质量
        physx::PxRigidDynamic* rigidBody = reinterpret_cast<physx::PxRigidDynamic*>(m_SoftBody.get());
        if (rigidBody)
        {
            // 简单计算质量
            MathLib::HReal volume = 1.0f; // 简化
            rigidBody->setMass(density * volume);
        }
    }
}

void PhysicsSoftBody::Transform(const MathLib::HTransform3& transform, const MathLib::HVector3& scale)
{
    if (m_SoftBody)
    {
        // 更新刚体的位置
        physx::PxRigidDynamic* rigidBody = reinterpret_cast<physx::PxRigidDynamic*>(m_SoftBody.get());
        if (rigidBody)
        {
            physx::PxTransform pxTransform = ConvertUtils::ToPx(transform);
            rigidBody->setGlobalPose(pxTransform);
        }
        
        m_Transform = transform;
        
        UpdateBoundingBox();
    }
}

void PhysicsSoftBody::GetOriginalVertexPositions(std::vector<MathLib::HVector3>& positions) const
{
    positions = m_OriginalVertices;
}

void* PhysicsSoftBody::GetTetrahedronMesh() const
{
    return m_SimulationMesh.get();
}

void* PhysicsSoftBody::GetSoftBodyData() const
{
    return m_SoftBody.get();
}

void PhysicsSoftBody::Commit()
{
    // 在实际PhysX版本中，这里应该调用适当的API提交更改
    if (m_SoftBody)
    {
        physx::PxRigidDynamic* rigidBody = reinterpret_cast<physx::PxRigidDynamic*>(m_SoftBody.get());
        if (rigidBody)
        {
            rigidBody->wakeUp();
        }
    }
}

void PhysicsSoftBody::GetRenderMesh(std::vector<MathLib::HVector3>& vertices, std::vector<uint32_t>& indices) const
{
    GetDeformedVertexPositions(vertices);
    
    if (m_TetrahedronIndices.empty() || !m_SimulationMesh)
    {
        indices.clear();
        return;
    }
    
    indices.clear();
    const int tetFaces[4][3] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
    
    uint32_t tetCount = GetTetrahedronCount();
    const bool has16BitIndices = m_SimulationMesh->getTetrahedronMeshFlags() & physx::PxTetrahedronMeshFlag::e16_BIT_INDICES;
    const void* indexBuffer = m_SimulationMesh->getTetrahedrons();
    
    if (has16BitIndices)
    {
        const uint16_t* shortIndices = reinterpret_cast<const uint16_t*>(indexBuffer);
        for (uint32_t i = 0; i < tetCount; ++i)
        {
            uint16_t vref[4];
            vref[0] = *shortIndices++;
            vref[1] = *shortIndices++;
            vref[2] = *shortIndices++;
            vref[3] = *shortIndices++;
            
            for (uint32_t j = 0; j < 4; ++j)
            {
                indices.push_back(vref[tetFaces[j][0]]);
                indices.push_back(vref[tetFaces[j][1]]);
                indices.push_back(vref[tetFaces[j][2]]);
            }
        }
    }
    else
    {
        const uint32_t* intIndices = reinterpret_cast<const uint32_t*>(indexBuffer);
        for (uint32_t i = 0; i < tetCount; ++i)
        {
            uint32_t vref[4];
            vref[0] = *intIndices++;
            vref[1] = *intIndices++;
            vref[2] = *intIndices++;
            vref[3] = *intIndices++;
            
            for (uint32_t j = 0; j < 4; ++j)
            {
                indices.push_back(vref[tetFaces[j][0]]);
                indices.push_back(vref[tetFaces[j][1]]);
                indices.push_back(vref[tetFaces[j][2]]);
            }
        }
    }
}

bool PhysicsSoftBody::CreateFromMesh(const SoftBodyCreateOptions& options)
{
    Release();
    
    m_OriginalVertices = options.m_MeshDesc.m_Vertices;
    
    m_FixedVertices.resize(options.m_MeshDesc.m_Vertices.size(), false);
    for (uint32_t fixedIndex : options.m_MeshDesc.m_FixedVertices)
    {
        if (fixedIndex < m_FixedVertices.size())
        {
            m_FixedVertices[fixedIndex] = true;
        }
    }
    
    m_Params = options.m_Params;
    m_Transform = options.m_Transform;
    m_CollisionLayer = options.m_CollisionLayer;
    m_CollisionMask = options.m_CollisionMask;
    m_UserData = options.m_UserData;
    
    return InitializeSoftBody(options);
}

bool PhysicsSoftBody::InitializeSoftBody(const SoftBodyCreateOptions& options)
{
    PhysicsEngine* engine = static_cast<PhysicsEngine*>(PhysicsEngineUtils::GetPhysicsEngine());
    if (!engine || !engine->m_Physics)
    {
        return false;
    }
    
    if (!CreateTetrahedronMesh(options.m_MeshDesc, true) || !CreateTetrahedronMesh(options.m_MeshDesc, false))
    {
        return false;
    }
    
    physx::PxRigidDynamic* actorPtr = PxGetPhysics().createRigidDynamic(physx::PxTransform(physx::PxIdentity));
    if (!actorPtr) 
    {
        return false;
    }
    
    m_SoftBody = make_physx_ptr(reinterpret_cast<physx::PxSoftBody*>(actorPtr));
    
    physx::PxTetrahedronMeshGeometry geometry(m_CollisionMesh.get());
    
    physx::PxMaterial* material = static_cast<PhysicsMaterial*>(m_Material.get())->GetMaterial();
    if (!material)
    {
        actorPtr->release();
        return false;
    }
    
    physx::PxShape* shape = PxGetPhysics().createShape(geometry, *material, true);
    if (!shape)
    {
        actorPtr->release();
        return false;
    }
    
    actorPtr->attachShape(*shape);
    shape->release();
    
    actorPtr->setSolverIterationCounts(options.m_Params.m_SolverIterations, 1);
    actorPtr->setMass(options.m_Params.m_Mass);
    
    physx::PxTransform pxTransform = ConvertUtils::ToPx(options.m_Transform);
    actorPtr->setGlobalPose(pxTransform);
    
    UpdateBoundingBox();
    
    return true;
}

bool PhysicsSoftBody::CreateTetrahedronMesh(const SoftBodyMeshDesc& meshDesc, bool isCollisionMesh)
{
    PhysicsEngine* engine = static_cast<PhysicsEngine*>(PhysicsEngineUtils::GetPhysicsEngine());
    if (!engine || !engine->m_Physics)
    {
        return false;
    }
    
    physx::PxPhysics& physics = PxGetPhysics();
    
    physx::PxArray<physx::PxVec3> pxVertices;
    pxVertices.resize(meshDesc.m_Vertices.size());
    for (size_t i = 0; i < meshDesc.m_Vertices.size(); i++)
    {
        const MathLib::HVector3& v = meshDesc.m_Vertices[i];
        pxVertices[i] = physx::PxVec3(v.x(), v.y(), v.z());
    }
    
    physx::PxArray<uint32_t> pxIndices;
    pxIndices.resize(meshDesc.m_Indices.size());
    for (size_t i = 0; i < meshDesc.m_Indices.size(); i++)
    {
        pxIndices[i] = meshDesc.m_Indices[i];
    }
    
    physx::PxTetrahedronMeshDesc meshDescriptor;
    meshDescriptor.points.count = static_cast<physx::PxU32>(pxVertices.size());
    meshDescriptor.points.stride = sizeof(physx::PxVec3);
    meshDescriptor.points.data = pxVertices.begin();
    
    meshDescriptor.tetrahedrons.count = static_cast<physx::PxU32>(pxIndices.size()) / 4;
    meshDescriptor.tetrahedrons.stride = 4 * sizeof(uint32_t);
    meshDescriptor.tetrahedrons.data = pxIndices.begin();
    
    if (isCollisionMesh)
    {
        physx::PxCookingParams cookingParams(physics.getTolerancesScale());
        cookingParams.midphaseDesc.setToDefault(physx::PxMeshMidPhase::eBVH34);
        cookingParams.meshPreprocessParams = physx::PxMeshPreprocessingFlag::eWELD_VERTICES;
        
        physx::PxDefaultMemoryOutputStream buf;
        if (PxCookTetrahedronMesh(cookingParams, meshDescriptor, buf))
        {
            physx::PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
            m_CollisionMesh = make_physx_ptr(physics.createTetrahedronMesh(input));
        }
        
        return m_CollisionMesh != nullptr;
    }
    else
    {
        if (!m_CollisionMesh)
        {
            return false;
        }
        
        physx::PxCookingParams cookingParams(physics.getTolerancesScale());
        cookingParams.midphaseDesc.setToDefault(physx::PxMeshMidPhase::eBVH34);
        cookingParams.meshPreprocessParams = physx::PxMeshPreprocessingFlag::eWELD_VERTICES;
        
        physx::PxDefaultMemoryOutputStream buf;
        if (PxCookTetrahedronMesh(cookingParams, meshDescriptor, buf))
        {
            physx::PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
            m_SimulationMesh = make_physx_ptr(physics.createTetrahedronMesh(input));
            
            if (m_SimulationMesh)
            {
                m_TetrahedronIndices = std::vector<uint32_t>(pxIndices.begin(), pxIndices.end());
            }
        }
        
        return m_SimulationMesh != nullptr;
    }
}

void PhysicsSoftBody::UpdateBoundingBox()
{
    if (!m_SimulationMesh)
    {
        m_BoundingBox = MathLib::HAABBox3D();
        return;
    }
    
    physx::PxBounds3 bounds = m_SimulationMesh->getLocalBounds();
    
    MathLib::HVector3 min(bounds.minimum.x, bounds.minimum.y, bounds.minimum.z);
    MathLib::HVector3 max(bounds.maximum.x, bounds.maximum.y, bounds.maximum.z);
    
    m_BoundingBox = MathLib::HAABBox3D();
    m_BoundingBox.extend(min);
    m_BoundingBox.extend(max);
} 