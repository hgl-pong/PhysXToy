#pragma once
#include "PhysicsTypes.h"
#include "PhysicsMacro.h"
#include <vector>
#include <string>
#include <cstring>
#include <stdint.h>

/**
 * @class PhysicsStream
 * @brief Provides serialization and deserialization capability for physics data
 * 
 * PhysicsStream is a utility class that allows physics objects, their properties,
 * and simulation states to be serialized to a binary buffer and later deserialized.
 * This enables saving/loading physics states and network transmission.
 */
class PHYSICSLIB_API PhysicsStream
{
public:
    /**
     * @brief Stream modes for read or write operations
     */
    enum class Mode
    {
        READ,   ///< Stream is in read mode (deserialization)
        WRITE   ///< Stream is in write mode (serialization)
    };

    /**
     * @brief Constructs a stream for reading or writing
     * @param mode The stream mode (READ or WRITE)
     * @param initialCapacity Initial buffer capacity in bytes for WRITE mode
     */
    PhysicsStream(Mode mode, size_t initialCapacity = 1024)
        : m_Mode(mode)
        , m_Position(0)
    {
        if (mode == Mode::WRITE)
        {
            m_Buffer.reserve(initialCapacity);
        }
    }

    /**
     * @brief Constructs a read stream from existing data
     * @param data Pointer to data buffer
     * @param size Size of data buffer in bytes
     */
    PhysicsStream(const void* data, size_t size)
        : m_Mode(Mode::READ)
        , m_Position(0)
    {
        m_Buffer.resize(size);
        memcpy(m_Buffer.data(), data, size);
    }

    /**
     * @brief Resets the stream to its initial state
     */
    void Reset()
    {
        m_Position = 0;
        if (m_Mode == Mode::WRITE)
        {
            m_Buffer.clear();
        }
    }

    /**
     * @brief Gets the current position in the stream
     * @return The current byte position
     */
    size_t GetPosition() const { return m_Position; }

    /**
     * @brief Sets the current position in the stream
     * @param position The new position in bytes
     */
    void SetPosition(size_t position) { m_Position = position; }

    /**
     * @brief Gets the total size of the stream data
     * @return The size in bytes
     */
    size_t GetSize() const { return m_Buffer.size(); }

    /**
     * @brief Gets the raw buffer data
     * @return Pointer to the buffer data
     */
    const uint8_t* GetData() const { return m_Buffer.data(); }

    /**
     * @brief Gets a copy of the buffer data
     * @return Vector containing the buffer data
     */
    const std::vector<uint8_t>& GetBuffer() const { return m_Buffer; }

    /**
     * @brief Checks if the stream has reached the end
     * @return True if at end of stream
     */
    bool IsEOF() const { return m_Position >= m_Buffer.size(); }

    /**
     * @brief Writes raw data to the stream
     * @param data Pointer to the data
     * @param size Size of the data in bytes
     * @return True if write was successful
     */
    bool Write(const void* data, size_t size)
    {
        if (m_Mode != Mode::WRITE)
            return false;

        const uint8_t* byteData = static_cast<const uint8_t*>(data);
        size_t requiredSize = m_Position + size;
        
        if (m_Buffer.size() < requiredSize)
            m_Buffer.resize(requiredSize);
        
        memcpy(m_Buffer.data() + m_Position, byteData, size);
        m_Position += size;
        return true;
    }

    /**
     * @brief Reads raw data from the stream
     * @param data Pointer to destination buffer
     * @param size Size to read in bytes
     * @return True if read was successful
     */
    bool Read(void* data, size_t size)
    {
        if (m_Mode != Mode::READ || m_Position + size > m_Buffer.size())
            return false;

        memcpy(data, m_Buffer.data() + m_Position, size);
        m_Position += size;
        return true;
    }

    /**
     * @brief Stream write operator for fundamental types
     * @tparam T Type to write
     * @param value Value to write
     * @return Reference to this stream
     */
    template<typename T>
    typename std::enable_if<std::is_fundamental<T>::value, PhysicsStream&>::type
    operator<<(const T& value)
    {
        Write(&value, sizeof(T));
        return *this;
    }

    /**
     * @brief Stream read operator for fundamental types
     * @tparam T Type to read
     * @param value Reference to store read value
     * @return Reference to this stream
     */
    template<typename T>
    typename std::enable_if<std::is_fundamental<T>::value, PhysicsStream&>::type
    operator>>(T& value)
    {
        Read(&value, sizeof(T));
        return *this;
    }

    /**
     * @brief Stream write operator for std::string
     * @param str String to write
     * @return Reference to this stream
     */
    PhysicsStream& operator<<(const std::string& str)
    {
        uint32_t length = static_cast<uint32_t>(str.length());
        *this << length;
        if (length > 0)
            Write(str.data(), length);
        return *this;
    }

    /**
     * @brief Stream read operator for std::string
     * @param str Reference to store read string
     * @return Reference to this stream
     */
    PhysicsStream& operator>>(std::string& str)
    {
        uint32_t length = 0;
        *this >> length;
        
        if (length > 0)
        {
            str.resize(length);
            Read(&str[0], length);
        }
        else
        {
            str.clear();
        }
        return *this;
    }

    /**
     * @brief Stream write operator for vector of fundamental types
     * @tparam T Vector element type
     * @param vec Vector to write
     * @return Reference to this stream
     */
    template<typename T>
    typename std::enable_if<std::is_fundamental<T>::value, PhysicsStream&>::type
    operator<<(const std::vector<T>& vec)
    {
        uint32_t size = static_cast<uint32_t>(vec.size());
        *this << size;
        if (size > 0)
            Write(vec.data(), size * sizeof(T));
        return *this;
    }

    /**
     * @brief Stream read operator for vector of fundamental types
     * @tparam T Vector element type
     * @param vec Reference to store read vector
     * @return Reference to this stream
     */
    template<typename T>
    typename std::enable_if<std::is_fundamental<T>::value, PhysicsStream&>::type
    operator>>(std::vector<T>& vec)
    {
        uint32_t size = 0;
        *this >> size;
        
        if (size > 0)
        {
            vec.resize(size);
            Read(vec.data(), size * sizeof(T));
        }
        else
        {
            vec.clear();
        }
        return *this;
    }

    /**
     * @brief Stream write operator for MathLib::HVector3
     * @param vec Vector to write
     * @return Reference to this stream
     */
    PhysicsStream& operator<<(const MathLib::HVector3& vec)
    {
        *this << vec.x << vec.y << vec.z;
        return *this;
    }

    /**
     * @brief Stream read operator for MathLib::HVector3
     * @param vec Reference to store read vector
     * @return Reference to this stream
     */
    PhysicsStream& operator>>(MathLib::HVector3& vec)
    {
        *this >> vec.x >> vec.y >> vec.z;
        return *this;
    }

    /**
     * @brief Stream write operator for MathLib::HMatrix3
     * @param mat Matrix to write
     * @return Reference to this stream
     */
    PhysicsStream& operator<<(const MathLib::HMatrix3& mat)
    {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                *this << mat(i, j);
        return *this;
    }

    /**
     * @brief Stream read operator for MathLib::HMatrix3
     * @param mat Reference to store read matrix
     * @return Reference to this stream
     */
    PhysicsStream& operator>>(MathLib::HMatrix3& mat)
    {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                *this >> mat(i, j);
        return *this;
    }

    /**
     * @brief Stream write operator for MathLib::HTransform3
     * @param transform Transform to write
     * @return Reference to this stream
     */
    PhysicsStream& operator<<(const MathLib::HTransform3& transform)
    {
        *this << transform.GetRotation() << transform.GetTranslation();
        return *this;
    }

    /**
     * @brief Stream read operator for MathLib::HTransform3
     * @param transform Reference to store read transform
     * @return Reference to this stream
     */
    PhysicsStream& operator>>(MathLib::HTransform3& transform)
    {
        MathLib::HMatrix3 rotation;
        MathLib::HVector3 translation;
        *this >> rotation >> translation;
        transform.SetRotation(rotation);
        transform.SetTranslation(translation);
        return *this;
    }

    /**
     * @brief Stream write operator for MathLib::HRay3D
     * @param ray Ray to write
     * @return Reference to this stream
     */
    PhysicsStream& operator<<(const MathLib::HRay3D& ray)
    {
        *this << ray.GetOrigin() << ray.GetDirection();
        return *this;
    }

    /**
     * @brief Stream read operator for MathLib::HRay3D (creates a new ray)
     * @param ray Reference to store read ray
     * @return Reference to this stream
     */
    PhysicsStream& operator>>(MathLib::HRay3D& ray)
    {
        MathLib::HVector3 origin, direction;
        *this >> origin >> direction;
        ray = MathLib::HRay3D(origin, direction);
        return *this;
    }

    /**
     * @brief Stream write operator for MathLib::HAABBox3D
     * @param box Bounding box to write
     * @return Reference to this stream
     */
    PhysicsStream& operator<<(const MathLib::HAABBox3D& box)
    {
        *this << box.GetMin() << box.GetMax();
        return *this;
    }

    /**
     * @brief Stream read operator for MathLib::HAABBox3D
     * @param box Reference to store read bounding box
     * @return Reference to this stream
     */
    PhysicsStream& operator>>(MathLib::HAABBox3D& box)
    {
        MathLib::HVector3 minPoint, maxPoint;
        *this >> minPoint >> maxPoint;
        box.Set(minPoint, maxPoint);
        return *this;
    }

    /**
     * @brief Stream write operator for PhysicsMaterialCreateOptions
     * @param options Material creation options to write
     * @return Reference to this stream
     */
    PhysicsStream& operator<<(const PhysicsMaterialCreateOptions& options)
    {
        *this << options.m_StaticFriction
              << options.m_DynamicFriction
              << options.m_Restitution
              << options.m_Density;
        return *this;
    }

    /**
     * @brief Stream read operator for PhysicsMaterialCreateOptions
     * @param options Reference to store read material creation options
     * @return Reference to this stream
     */
    PhysicsStream& operator>>(PhysicsMaterialCreateOptions& options)
    {
        *this >> options.m_StaticFriction
              >> options.m_DynamicFriction
              >> options.m_Restitution
              >> options.m_Density;
        return *this;
    }

    /**
     * @brief Stream write operator for CollisionGeometryCreateOptions
     * @param options Collision geometry creation options to write
     * @return Reference to this stream
     */
    PhysicsStream& operator<<(const CollisionGeometryCreateOptions& options)
    {
        *this << static_cast<uint32_t>(options.m_GeometryType)
              << options.m_LocalTransform
              << options.m_Scale;

        // Serialize geometry-specific parameters based on type
        switch (options.m_GeometryType)
        {
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
            *this << options.m_SphereParams.m_Radius;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
            *this << options.m_BoxParams.m_HalfExtents;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
            *this << options.m_CapsuleParams.m_Radius
                  << options.m_CapsuleParams.m_HalfHeight;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
            *this << options.m_PlaneParams.m_Normal
                  << options.m_PlaneParams.m_Distance;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
            *this << options.m_TriangleMeshParams.m_Vertices
                  << options.m_TriangleMeshParams.m_Indices;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
            *this << options.m_ConvexMeshParams.m_Vertices
                  << options.m_ConvexMeshParams.m_Indices;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_HEIGHT_FIELD:
            *this << options.m_HeightFieldParams.m_HeightData
                  << options.m_HeightFieldParams.m_Rows
                  << options.m_HeightFieldParams.m_Columns
                  << options.m_HeightFieldParams.m_RowScale
                  << options.m_HeightFieldParams.m_ColumnScale
                  << options.m_HeightFieldParams.m_HeightScale;
            break;
        default:
            break;
        }
        
        return *this;
    }

    /**
     * @brief Stream read operator for CollisionGeometryCreateOptions
     * @param options Reference to store read collision geometry creation options
     * @return Reference to this stream
     */
    PhysicsStream& operator>>(CollisionGeometryCreateOptions& options)
    {
        uint32_t geometryType;
        *this >> geometryType
              >> options.m_LocalTransform
              >> options.m_Scale;
              
        options.m_GeometryType = static_cast<CollierGeometryType>(geometryType);

        // Deserialize geometry-specific parameters based on type
        switch (options.m_GeometryType)
        {
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE:
            *this >> options.m_SphereParams.m_Radius;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX:
            *this >> options.m_BoxParams.m_HalfExtents;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CAPSULE:
            *this >> options.m_CapsuleParams.m_Radius
                  >> options.m_CapsuleParams.m_HalfHeight;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE:
            *this >> options.m_PlaneParams.m_Normal
                  >> options.m_PlaneParams.m_Distance;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_TRIANGLE_MESH:
            *this >> options.m_TriangleMeshParams.m_Vertices
                  >> options.m_TriangleMeshParams.m_Indices;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_CONVEX_MESH:
            *this >> options.m_ConvexMeshParams.m_Vertices
                  >> options.m_ConvexMeshParams.m_Indices;
            break;
        case CollierGeometryType::COLLIER_GEOMETRY_TYPE_HEIGHT_FIELD:
            *this >> options.m_HeightFieldParams.m_HeightData
                  >> options.m_HeightFieldParams.m_Rows
                  >> options.m_HeightFieldParams.m_Columns
                  >> options.m_HeightFieldParams.m_RowScale
                  >> options.m_HeightFieldParams.m_ColumnScale
                  >> options.m_HeightFieldParams.m_HeightScale;
            break;
        default:
            break;
        }
        
        return *this;
    }

    /**
     * @brief Stream write operator for PhysicsObjectCreateOptions
     * @param options Physics object creation options to write
     * @return Reference to this stream
     */
    PhysicsStream& operator<<(const PhysicsObjectCreateOptions& options)
    {
        *this << static_cast<uint32_t>(options.m_ObjectType)
              << options.m_Transform
              << options.m_MaterialOptions
              << options.m_CollisionLayer
              << options.m_CollisionMask
              << options.m_EnableCCD
              << options.m_LinearDamping
              << options.m_AngularDamping
              << options.m_Mass
              << options.m_IsKinematic
              << options.m_EnableGravity;
        
        // User data pointer is not serialized as it's context-dependent
        return *this;
    }

    /**
     * @brief Stream read operator for PhysicsObjectCreateOptions
     * @param options Reference to store read physics object creation options
     * @return Reference to this stream
     */
    PhysicsStream& operator>>(PhysicsObjectCreateOptions& options)
    {
        uint32_t objectType;
        *this >> objectType
              >> options.m_Transform
              >> options.m_MaterialOptions
              >> options.m_CollisionLayer
              >> options.m_CollisionMask
              >> options.m_EnableCCD
              >> options.m_LinearDamping
              >> options.m_AngularDamping
              >> options.m_Mass
              >> options.m_IsKinematic
              >> options.m_EnableGravity;
              
        options.m_ObjectType = static_cast<PhysicsObjectType>(objectType);
        options.m_UserData = nullptr; // User data is not deserialized
        
        return *this;
    }

    /**
     * @brief Stream write operator for PhysicsRaycastHit
     * @param hit Raycast hit information to write
     * @return Reference to this stream
     */
    PhysicsStream& operator<<(const PhysicsRaycastHit& hit)
    {
        // Store hit data, but not the actual object pointers
        // Instead, store object IDs or other identifiers if available
        *this << hit.m_Position
              << hit.m_Normal
              << hit.m_Distance;
        return *this;
    }

    /**
     * @brief Stream read operator for PhysicsRaycastHit
     * @param hit Reference to store read raycast hit information
     * @return Reference to this stream
     */
    PhysicsStream& operator>>(PhysicsRaycastHit& hit)
    {
        *this >> hit.m_Position
              >> hit.m_Normal
              >> hit.m_Distance;
        
        // Object pointers are not deserialized, need to be resolved by the application
        hit.m_Object = nullptr;
        hit.m_Collider = nullptr;
        
        return *this;
    }

private:
    Mode m_Mode;                   ///< Stream mode (read or write)
    std::vector<uint8_t> m_Buffer; ///< Data buffer
    size_t m_Position;             ///< Current position in buffer
};
