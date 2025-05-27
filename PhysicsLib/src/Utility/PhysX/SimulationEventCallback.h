#pragma once
#include "Physics/PhysicsCommon.h"
#include "../PhysXUtils.h"
#include "PxPhysicsAPI.h"
#include <unordered_map>
#include <unordered_set>

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
		IPhysicsObject* handle0 = static_cast<IPhysicsObject*>(pairHeader.actors[0]->userData);
		IPhysicsObject* handle1 = static_cast<IPhysicsObject*>(pairHeader.actors[1]->userData);

		ProcessCollisionCallbacks(pairHeader, pairs, nbPairs, handle0, handle1);

		if (m_ContactCallback)
		{
			const bool bufferContacts = m_ContactCallback->BufferContacts();

			const physx::PxU32 bufferSize = 1024;
			physx::PxContactPairPoint contacts[bufferSize];
			for (physx::PxU32 i = 0; i < nbPairs; i++)
			{
				const physx::PxContactPair& cp = pairs[i];
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

							if (!bufferContacts || Force > 1000.0f)
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
	void ProcessCollisionCallbacks(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs, IPhysicsObject* obj0, IPhysicsObject* obj1)
	{
		if (!obj0 || !obj1) return;

		ICollisionCallback* callback0 = obj0->GetCollisionCallback();
		ICollisionCallback* callback1 = obj1->GetCollisionCallback();

		if (!callback0 && !callback1) return;

		uint64_t pairId = CreatePairId(obj0, obj1);

		bool hasContact = false;
		CollisionEventData eventData;
		
		for (physx::PxU32 i = 0; i < nbPairs; i++)
		{
			const physx::PxContactPair& cp = pairs[i];
			
			if (cp.events & physx::PxPairFlag::eNOTIFY_TOUCH_FOUND)
			{
				if (m_ActiveCollisions.find(pairId) == m_ActiveCollisions.end())
				{
					m_ActiveCollisions.insert(pairId);
					hasContact = true;
					
					if (ExtractContactInfo(cp, eventData, obj0, obj1))
					{
						if (callback0) callback0->OnCollisionEnter(eventData);
						if (callback1) 
						{
							CollisionEventData swappedData = eventData;
							swappedData.objectA = eventData.objectB;
							swappedData.objectB = eventData.objectA;
							swappedData.contactNormal = -eventData.contactNormal;
							callback1->OnCollisionEnter(swappedData);
						}
					}
				}
			}
			else if (cp.events & physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS)
			{
				if (callback0) callback0->OnCollisionStay(eventData);
				if (callback1)
				{
					CollisionEventData swappedData = eventData;
					swappedData.objectA = eventData.objectB;
					swappedData.objectB = eventData.objectA;
					swappedData.contactNormal = -eventData.contactNormal;
					callback1->OnCollisionStay(swappedData);
				}
			}
			else if (cp.events & physx::PxPairFlag::eNOTIFY_TOUCH_LOST)
			{
				auto it = m_ActiveCollisions.find(pairId);
				if (it != m_ActiveCollisions.end())
				{
					m_ActiveCollisions.erase(it);
					
					if (ExtractContactInfo(cp, eventData, obj0, obj1))
					{
						if (callback0) callback0->OnCollisionExit(eventData);
						if (callback1)
						{
							CollisionEventData swappedData = eventData;
							swappedData.objectA = eventData.objectB;
							swappedData.objectB = eventData.objectA;
							swappedData.contactNormal = -eventData.contactNormal;
							callback1->OnCollisionExit(swappedData);
						}
					}
				}
			}
		}
	}

	uint64_t CreatePairId(IPhysicsObject* obj0, IPhysicsObject* obj1)
	{
		uintptr_t ptr0 = reinterpret_cast<uintptr_t>(obj0);
		uintptr_t ptr1 = reinterpret_cast<uintptr_t>(obj1);
		if (ptr0 > ptr1) std::swap(ptr0, ptr1);
		
		return (static_cast<uint64_t>(ptr0) << 32) | static_cast<uint64_t>(ptr1);
	}

	bool ExtractContactInfo(const physx::PxContactPair& cp, CollisionEventData& eventData, IPhysicsObject* obj0, IPhysicsObject* obj1)
	{
		const physx::PxU32 bufferSize = 64;
		physx::PxContactPairPoint contacts[bufferSize];
		const physx::PxU32 nbContacts = cp.extractContacts(contacts, bufferSize);
		
		if (nbContacts > 0)
		{
			const auto& contact = contacts[0];
			
			eventData.objectA = PhysicsPtr<IPhysicsObject>(obj0, [](IPhysicsObject*){});
			eventData.objectB = PhysicsPtr<IPhysicsObject>(obj1, [](IPhysicsObject*){});
			eventData.contactPoint = ConvertUtils::FromPx(contact.position);
			eventData.contactNormal = ConvertUtils::FromPx(contact.normal);
			eventData.penetrationDepth = -contact.separation;
			
			return true;
		}
		
		return false;
	}

private:
	IPhysicsContactCallback* m_ContactCallback = nullptr;
	std::vector<PhysicsContactData>	m_ContactData;
	std::unordered_set<uint64_t> m_ActiveCollisions;
};