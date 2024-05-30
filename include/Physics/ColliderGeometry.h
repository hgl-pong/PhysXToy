#pragma once
#include "Physics/PhysicsCommon.h"
namespace physx
{
	class PxShape;
}

class BoxColliderGeometry : public IColliderGeometry
{
public:
	BoxColliderGeometry(const MathLib::HVector3 &halfExtents) : m_HalfExtents(halfExtents) {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_BOX; }
	void SetScale(const MathLib::HVector3 &scale) override { m_Scale = scale; }
	MathLib::HVector3 GetHalfSize() const { return m_HalfExtents; }
	MathLib::HVector3 GetScale() const { return m_Scale; }

private:
	MathLib::HVector3 m_HalfExtents;
	MathLib::HVector3 m_Scale;
};

class SphereColliderGeometry : public IColliderGeometry
{
public:
	SphereColliderGeometry(MathLib::HReal radius) : m_Radius(radius) {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_SPHERE; }
	void SetScale(const MathLib::HVector3 &scale) override { m_Scale = scale; }
	MathLib::HReal GetRadius() const { return m_Radius; }
	MathLib::HVector3 GetScale() const { return m_Scale; }

private:
	MathLib::HReal m_Radius;
	MathLib::HVector3 m_Scale;
};

class PlaneColliderGeometry : public IColliderGeometry
{
public:
	PlaneColliderGeometry(const MathLib::HVector3 &normal, MathLib::HReal distance) : m_Normal(normal), m_Distance(distance) {}
	CollierGeometryType GetType() const override { return CollierGeometryType::COLLIER_GEOMETRY_TYPE_PLANE; }
	void SetScale(const MathLib::HVector3 &scale) override { m_Scale = scale; }
	MathLib::HVector3 GetNormal() const { return m_Normal; }
	MathLib::HReal GetDistance() const { return m_Distance; }
	MathLib::HVector3 GetScale() const { return m_Scale; }
private:
	MathLib::HVector3 m_Normal;
	MathLib::HReal m_Distance;
	MathLib::HVector3 m_Scale;
};