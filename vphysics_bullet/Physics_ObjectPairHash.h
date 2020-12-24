#ifndef PHYSICS_OBJECTPAIRHASH_H
#define PHYSICS_OBJECTPAIRHASH_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

#include <vphysics/object_hash.h>

struct pair_hash_list {
	void *object0;
	void *object1;

	pair_hash_list *next;
};

class CPhysicsObjectPairHash : public IPhysicsObjectPairHash {
	public:
		CPhysicsObjectPairHash();

		void	AddObjectPair(void *pObject0, void *pObject1);
		void	RemoveObjectPair(void *pObject0, void *pObject1);
		bool	IsObjectPairInHash(void *pObject0, void *pObject1);
		void	RemoveAllPairsForObject(void *pObject0);
		bool	IsObjectInHash(void *pObject0);

		int		GetPairCountForObject(void *pObject0);
		int		GetPairListForObject(void *pObject0, int nMaxCount, void **ppObjectList);

		int		GetEntry(void *pObject0, void *pObject1);

	private:
		pair_hash_list *m_pHashList[256];
};

#endif // PHYSICS_OBJECTPAIRHASH_H
