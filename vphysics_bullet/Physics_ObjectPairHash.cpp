#include "StdAfx.h"
#include "Physics_ObjectPairHash.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/***********************************
* CLASS CPhysicsObjectPairHash
***********************************/

CPhysicsObjectPairHash::CPhysicsObjectPairHash() {
	for (int i = 0; i < 256; i++)
		m_pHashList[i] = NULL;
}

void CPhysicsObjectPairHash::AddObjectPair(void *pObject0, void *pObject1) {
	int entry = GetEntry(pObject0, pObject1);

	// This particular entry may have more than one hash, so find the last one.
	pair_hash_list *last = NULL;
	for (pair_hash_list *hash = m_pHashList[entry]; hash; hash = hash->next)
		last = hash;

	// Setup our hash
	pair_hash_list *hash = new pair_hash_list;
	hash->object0 = pObject0;
	hash->object1 = pObject1;
	//hash->previous = last;
	hash->next = NULL;

	// Now link ourselves up to the list.
	if (last)
		last->next = hash;
	else
		m_pHashList[entry] = hash;
}

void CPhysicsObjectPairHash::RemoveObjectPair(void *pObject0, void *pObject1) {
	int entry = GetEntry(pObject0, pObject1);

	pair_hash_list *next = NULL;
	pair_hash_list *last = NULL;

	for (pair_hash_list *hash = m_pHashList[entry]; hash; hash = next) {
		if ((hash->object0 == pObject0 || hash->object0 == pObject1) && (hash->object1 == pObject0 || hash->object1 == pObject1)) {
			if (last)
				last->next = hash->next;
			else
				m_pHashList[entry] = hash->next;

			//if (hash->next)
			//	hash->next->previous = hash->previous;
			next = hash->next;	// Fix for access violation

			delete hash;
		} else {
			next = hash->next;
			last = hash;
		}
	}
}

void CPhysicsObjectPairHash::RemoveAllPairsForObject(void *pObject0) {
	// Loop through all entries
	for (int i = 0; i < 256; i++) {
		pair_hash_list *next = NULL;
		pair_hash_list *last = NULL;

		for (pair_hash_list *hash = m_pHashList[i]; hash; hash = next) {
			if (hash->object0 == pObject0 || hash->object1 == pObject0) {
				if (last)
					last->next = hash->next;
				else
					m_pHashList[i] = hash->next;

				//if (hash->next)
				//	hash->next->previous = hash->previous;
				next = hash->next;	// Fix for access violation

				delete hash;
			} else {
				next = hash->next;
				last = hash;
			}
		}
	}
}

bool CPhysicsObjectPairHash::IsObjectPairInHash(void *pObject0, void *pObject1) {
	int entry = GetEntry(pObject0, pObject1);
	for (pair_hash_list *hash = m_pHashList[entry]; hash; hash = hash->next) {
		if ((hash->object0 == pObject0 || hash->object0 == pObject1) && (hash->object1 == pObject0 || hash->object1 == pObject1))
			return true;
	}

	return false;
}

bool CPhysicsObjectPairHash::IsObjectInHash(void *pObject0) {
	for (int i = 0; i < 256; i++) {
		for (pair_hash_list *hash = m_pHashList[i]; hash; hash = hash->next) {
			if (hash->object0 == pObject0 || hash->object1 == pObject0)
				return true;
		}
	}

	return false;
}

int CPhysicsObjectPairHash::GetPairCountForObject(void *pObject0) {
	int c = 0;
	for (int i = 0; i < 256; i++) {
		for (pair_hash_list *hash = m_pHashList[i]; hash; hash = hash->next) {
			if (hash->object0 == pObject0 || hash->object1 == pObject0)
				c++;
		}
	}

	return c;
}

int CPhysicsObjectPairHash::GetPairListForObject(void *pObject0, int nMaxCount, void **ppObjectList) {
	int c = 0;
	for (int i = 0; i < 256; i++) {
		if (c >= nMaxCount)
			break;

		for (pair_hash_list *hash = m_pHashList[i]; hash; hash = hash->next) {
			if (c >= nMaxCount)
				break;

			// Get the opposite object in the pair
			ppObjectList[c++] = hash->object0 == pObject0 ? hash->object1 : hash->object0;
		}
	}

	return c;
}

// Purpose: Generate a number in [0,255] given 2 pointers. Must be the same for the same 2 pointers.
int CPhysicsObjectPairHash::GetEntry(void *pObject0, void *pObject1) {
	return (((intptr_t)pObject0 ^ (intptr_t)pObject1) >> (sizeof(void*))) & 0xFF;
}