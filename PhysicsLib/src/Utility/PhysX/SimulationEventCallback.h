#pragma once
#include "Physics/PhysicsCommon.h"
#include "../PhysXUtils.h"
#include "PxPhysicsAPI.h"

class PhysXSimulationEventCallback : public physx::PxSimulationEventCallback
{
public:
	PhysXSimulationEventCallback() : m_ContactCallback(nullptr) {}

	void Init(IPhysicsContactCallback* cb)
	{
		m_ContactCallback = cb;
	}
	void Release()
	{
		m_ContactData.clear();
	}

	virtual void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count)
	{

	}
	virtual void onWake(physx::PxActor** actors, physx::PxU32 count)
	{

	}
	virtual void onSleep(physx::PxActor** actors, physx::PxU32 count)
	{

	}
	virtual void onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs)
	{
		if (!m_ContactCallback)
			return;

		/*		if(pairHeader.actors[0]->getConcreteType() == physx::PxConcreteType::eRIGID_DYNAMIC
			   && pairHeader.actors[1]->getConcreteType() == physx::PxConcreteType::eRIGID_DYNAMIC
			   )
			   return;*/


		const bool bufferContacts = m_ContactCallback->BufferContacts();

		IPhysicsObject* handle0 = static_cast<IPhysicsObject*>(pairHeader.actors[0]->userData);
		IPhysicsObject* handle1 = static_cast<IPhysicsObject*>(pairHeader.actors[1]->userData);

		const physx::PxU32 bufferSize = 1024;	//###
		physx::PxContactPairPoint contacts[bufferSize];
		for (physx::PxU32 i = 0; i < nbPairs; i++)
		{
			const physx::PxContactPair& cp = pairs[i];
			//		if (!(cp.events & (physx::PxPairFlag::eNOTIFY_TOUCH_LOST | physx::PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST)))
			{
				const physx::PxU32 nbContacts = pairs[i].extractContacts(contacts, bufferSize);
				if (nbContacts)
				{
					for (physx::PxU32 j = 0; j < nbContacts; j++)
					{
#ifdef USE_FORCE_THRESHOLD
						auto src = contacts[j];
						PhysicsContactData dst;
						dst.m_ObjectA = handle0;
						dst.m_ObjectB = handle1;
						dst.m_ContactPoint = ConvertUtils::FromPx(src.position);
						dst.m_Separation = src.separation;
						dst.m_ContactNormal = ConvertUtils::FromPx(src.normal);
						dst.m_InternalFaceIndexA = src.internalFaceIndex0;
						dst.m_Impulse = ConvertUtils::FromPx(src.impulse);
						dst.m_InternalFaceIndexB = src.internalFaceIndex1;
						m_ContactData.push_back(dst);
#else
						const float Force = contacts[j].impulse.magnitude() * 60.0f;

						if (!bufferContacts || Force > 1000.0f)	// ### disabled for contact notify test
						{
							auto src = contacts[j];
							PhysicsContactData dst;
							dst.m_ObjectA = handle0;
							dst.m_ObjectB = handle1;
							dst.m_ContactPoint = ConvertUtils::FromPx(src.position);
							dst.m_Separation = src.separation;
							dst.m_ContactNormal = ConvertUtils::FromPx(src.normal);
							dst.m_InternalFaceIndexA = src.internalFaceIndex0;
							dst.m_Impulse = ConvertUtils::FromPx(src.impulse);
							dst.m_InternalFaceIndexB = src.internalFaceIndex1;
							m_ContactData.push_back(dst);
						}
#endif
					}
				}
			}
		}

		if (!bufferContacts)
			SendContacts();
	}
	virtual void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count)
	{

	}
	virtual void onAdvance(const physx::PxRigidBody* const* bodyBuffer, const physx::PxTransform* poseBuffer, const physx::PxU32 count)
	{

	}

	void SendContacts()
	{
		if (m_ContactData.size())
		{
			if (m_ContactCallback)
			{
				const size_t NbContacts = m_ContactData.size();
				m_ContactCallback->OnContact(NbContacts, m_ContactData.data());
			}
			m_ContactData.clear();
		}
	}
private:
	IPhysicsContactCallback* m_ContactCallback = nullptr;
	std::vector<PhysicsContactData>	m_ContactData;
};