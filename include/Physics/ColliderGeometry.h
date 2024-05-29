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
	CollierGeometryType GetType() const override { return CollierGeometryType::Box; }
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
	CollierGeometryType GetType() const override { return CollierGeometryType::Sphere; }
	void SetScale(const MathLib::HVector3 &scale) override { m_Scale = scale; }
	MathLib::HReal GetRadius() const { return m_Radius; }
	MathLib::HVector3 GetScale() const { return m_Scale; }

private:
	MathLib::HReal m_Radius;
	MathLib::HVector3 m_Scale;
};