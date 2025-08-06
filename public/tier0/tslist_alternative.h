#pragma once

#include "tier0/dbg.h"
#include "tier0/threadtools.h"
#include "tier0/memalloc.h"
#include "tier0/memdbgoff.h"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>

//-----------------------------------------------------------------------------

#if defined( PLATFORM_64BITS )
constexpr std::size_t TSLIST_HEAD_ALIGNMENT = 16;
constexpr std::size_t TSLIST_NODE_ALIGNMENT = 16;
#else
constexpr std::size_t TSLIST_HEAD_ALIGNMENT = 8;
constexpr std::size_t TSLIST_NODE_ALIGNMENT = 8;
#endif

#define TSLIST_HEAD_ALIGN alignas(TSLIST_HEAD_ALIGNMENT)
#define TSLIST_NODE_ALIGN alignas(TSLIST_NODE_ALIGNMENT)
#define TSLIST_HEAD_ALIGN_POST
#define TSLIST_NODE_ALIGN_POST


//-----------------------------------------------------------------------------
// Lock free list.
//-----------------------------------------------------------------------------

struct TSLIST_NODE_ALIGN TSLNodeBase_t
{
    TSLNodeBase_t *Next; // name to match Windows
} TSLIST_NODE_ALIGN_POST;

struct TSLIST_HEAD_ALIGN TSLHead_t
{
    TSLHead_t() = default;
    TSLHead_t(TSLNodeBase_t *next, int16 depth, int16 sequence)
        : Next{next}, Depth{depth}, Sequence{sequence}
    {
    }

    TSLNodeBase_t *Next;
    int16   Depth;
    int16   Sequence;
} TSLIST_HEAD_ALIGN_POST;


//-------------------------------------
class CTSListBase
{
        std::shared_ptr<TSLHead_t> m_Head;
public:

        // override new/delete so we can guarantee 8-byte aligned allocs
        static void * operator new(size_t size)
        {
                CTSListBase *pNode = (CTSListBase *)MemAlloc_AllocAlignedFileLine( size, TSLIST_HEAD_ALIGNMENT, __FILE__, __LINE__ );
                return pNode;
        }

        static void * operator new(size_t size, int nBlockUse, const char *pFileName, int nLine)
        {
                CTSListBase *pNode = (CTSListBase *)MemAlloc_AllocAlignedFileLine( size, TSLIST_HEAD_ALIGNMENT, pFileName, nLine );
                return pNode;
        }

        static void operator delete(void *p)
        {
                MemAlloc_FreeAligned( p );
        }

        static void operator delete(void *p, int nBlockUse, const char *pFileName, int nLine)
        {
                MemAlloc_FreeAligned( p );
        }

private:
        // These ain't gonna work
        static void * operator new[]( size_t size );
        static void operator delete[]( void *p );

public:

        CTSListBase()
        {
                m_Head = std::make_shared<TSLHead_t>(nullptr, 0, 0);

                if ( reinterpret_cast<std::uintptr_t>(m_Head.get()) % TSLIST_HEAD_ALIGNMENT != 0 )
                {
                        Error( "CTSListBase: Misaligned list\n" );
                        DebuggerBreak();
                }
        }

        ~CTSListBase()
        {
            Detach();
        }

        TSLNodeBase_t *Push( TSLNodeBase_t *pNode )
        {
#ifdef _DEBUG
                if ( reinterpret_cast<std::uintptr_t>(pNode) % TSLIST_NODE_ALIGNMENT != 0 )
                {
                        Error( "CTSListBase: Misaligned node\n" );
                        DebuggerBreak();
                }
#endif

                std::shared_ptr<TSLHead_t> oldHead = std::atomic_load(&m_Head);
                std::shared_ptr<TSLHead_t> newHead = std::make_shared<TSLHead_t>();


                for ( ;; )
                {
                    pNode->Next = oldHead->Next;
                    newHead->Next = pNode;

                    newHead->Depth = oldHead->Depth + 1;
                    newHead->Sequence = oldHead->Sequence + 1;


                    if (std::atomic_compare_exchange_weak(&m_Head, &oldHead, newHead)) {
                        break;
                    }
                    ThreadPause();
                };

                return oldHead->Next;
        }

        TSLNodeBase_t *Pop()
        {
            std::shared_ptr<TSLHead_t> oldHead = std::atomic_load(&m_Head);
            std::shared_ptr<TSLHead_t> newHead = std::make_shared<TSLHead_t>();

            for ( ;; )
            {
                if ( !oldHead->Next )
                        return nullptr;

                newHead->Next = oldHead->Next->Next;
                newHead->Depth = oldHead->Depth - 1;
                newHead->Sequence = oldHead->Sequence;

                if (std::atomic_compare_exchange_weak(&m_Head, &oldHead, newHead)) {
                    break;
                }
                ThreadPause();
            };

            return oldHead->Next;
        }

        TSLNodeBase_t *Detach()
        {
            std::shared_ptr<TSLHead_t> oldHead = std::atomic_load(&m_Head);
            std::shared_ptr<TSLHead_t> newHead = std::make_shared<TSLHead_t>();

            for ( ;; )
            {
                ThreadPause();

                if ( !oldHead->Next )
                    return nullptr;

                newHead->Next = nullptr;
                newHead->Depth = 0;
                newHead->Sequence = oldHead->Sequence;

                if (std::atomic_compare_exchange_weak(&m_Head, &oldHead, newHead)) {
                    break;
                }
                ThreadPause();
            }

            return oldHead->Next;
        }

        TSLHead_t *AccessUnprotected()
        {
            return m_Head.get();
        }

        int Count() const
        {
            return std::atomic_load(&m_Head)->Depth;
        }


} TSLIST_HEAD_ALIGN_POST;

//-------------------------------------

template <typename T>
class TSLIST_HEAD_ALIGN CTSSimpleList : public CTSListBase    
{
        static_assert(sizeof(T) >= sizeof(TSLNodeBase_t), "T must be a subclass of TSLNodeBase_t");
public:
        void Push( T *pNode )
        {
                CTSListBase::Push( reinterpret_cast<TSLNodeBase_t*>(pNode) );
        }

        T *Pop()
        {
                return reinterpret_cast<T*>( CTSListBase::Pop() );
        }
} TSLIST_HEAD_ALIGN_POST;

//-------------------------------------
// this is a replacement for CTSList<> and CObjectPool<> that does not
// have a per-item, per-alloc new/delete overhead
// similar to CTSSimpleList except that it allocates it's own pool objects
// and frees them on destruct.  Also it does not overlay the TSNodeBase_t memory
// on T's memory
template< class T >
class TSLIST_HEAD_ALIGN CTSPool : public CTSListBase
{
        // packs the node and the item (T) into a single struct and pools those
        struct TSLIST_NODE_ALIGN simpleTSPoolStruct_t : public TSLNodeBase_t
        {
                T elem;
        } TSLIST_NODE_ALIGN_POST;

public:

        ~CTSPool()
        {
                Purge();
        }

        void Purge()
        {
                simpleTSPoolStruct_t *pNode = NULL;
                while ( 1 )
                {
                        pNode = (simpleTSPoolStruct_t *)CTSListBase::Pop();
                        if ( !pNode )
                                break;
                        delete pNode;
                }
        }

        void PutObject( T *pInfo )
        {
                char *pElem = (char *)pInfo;
                pElem -= offsetof( simpleTSPoolStruct_t, elem );
                simpleTSPoolStruct_t *pNode = (simpleTSPoolStruct_t *)pElem;

                CTSListBase::Push( pNode );
        }

        T *GetObject()
        {
                simpleTSPoolStruct_t *pNode = (simpleTSPoolStruct_t *)CTSListBase::Pop();
                if ( !pNode )
                {
                        pNode = new simpleTSPoolStruct_t;
                }
                return &pNode->elem;
        }

        // omg windows sdk - why do you #define GetObject()?
        FORCEINLINE T *Get()
        {
                return GetObject();
        }
} TSLIST_HEAD_ALIGN_POST;
//-------------------------------------

template <typename T>
class TSLIST_HEAD_ALIGN CTSList : public CTSListBase
{
public:
        struct TSLIST_NODE_ALIGN Node_t : public TSLNodeBase_t
        {
                Node_t() {}
                Node_t( const T &init ) : elem( init ) {}
                T elem;

                // override new/delete so we can guarantee 8-byte aligned allocs
                static void * operator new(size_t size)
                {
                        Node_t *pNode = (Node_t *)MemAlloc_AllocAlignedFileLine( size, TSLIST_NODE_ALIGNMENT, __FILE__, __LINE__ );
                        return pNode;
                }

                // override new/delete so we can guarantee 8-byte aligned allocs
                static void * operator new(size_t size, int nBlockUse, const char *pFileName, int nLine)
                {
                        Node_t *pNode = (Node_t *)MemAlloc_AllocAlignedFileLine( size, TSLIST_NODE_ALIGNMENT, pFileName, nLine );
                        return pNode;
                }

                static void operator delete(void *p)
                {
                        MemAlloc_FreeAligned( p );
                }
                static void operator delete(void *p, int nBlockUse, const char *pFileName, int nLine)
                {
                        MemAlloc_FreeAligned( p );
                }

        } TSLIST_NODE_ALIGN_POST;

        ~CTSList()
        {
                Purge();
        }

        void Purge()
        {
                Node_t *pCurrent = Detach();
                Node_t *pNext;
                while ( pCurrent )
                {
                        pNext = (Node_t *)pCurrent->Next;
                        delete pCurrent;
                        pCurrent = pNext;
                }
        }

        void RemoveAll()
        {
                Purge();
        }

        Node_t *Push( Node_t *pNode )
        {
                return (Node_t *)CTSListBase::Push( pNode );
        }

        Node_t *Pop()
        {
                return (Node_t *)CTSListBase::Pop();
        }

        void PushItem( const T &init )
        {
                Push( new Node_t( init ) );
        }

        bool PopItem( T *pResult )
        {
                Node_t *pNode = Pop();
                if ( !pNode )
                        return false;
                *pResult = pNode->elem;
                delete pNode;
                return true;
        }

        Node_t *Detach()
        {
                return (Node_t *)CTSListBase::Detach();
        }

} TSLIST_HEAD_ALIGN_POST;

//-------------------------------------

template <typename T>
class TSLIST_HEAD_ALIGN CTSListWithFreeList : public CTSListBase
{
public:
        struct TSLIST_NODE_ALIGN Node_t : public TSLNodeBase_t
        {
                Node_t() {}
                Node_t( const T &init ) : elem( init ) {}

                T elem;
        } TSLIST_NODE_ALIGN_POST;

        ~CTSListWithFreeList()
        {
                Purge();
        }

        void Purge()
        {
                Node_t *pCurrent = Detach();
                Node_t *pNext;
                while ( pCurrent )
                {
                        pNext = (Node_t *)pCurrent->Next;
                        delete pCurrent;
                        pCurrent = pNext;
                }
                pCurrent = (Node_t *)m_FreeList.Detach();
                while ( pCurrent )
                {
                        pNext = (Node_t *)pCurrent->Next;
                        delete pCurrent;
                        pCurrent = pNext;
                }
        }

        void RemoveAll()
        {
                Node_t *pCurrent = Detach();
                Node_t *pNext;
                while ( pCurrent )
                {
                        pNext = (Node_t *)pCurrent->Next;
                        m_FreeList.Push( pCurrent );
                        pCurrent = pNext;
                }
        }

        Node_t *Push( Node_t *pNode )
        {
                return (Node_t *)CTSListBase::Push( pNode );
        }

        Node_t *Pop()
        {
                return (Node_t *)CTSListBase::Pop();
        }

        void PushItem( const T &init )
        {
                Node_t *pNode = (Node_t *)m_FreeList.Pop();
                if ( !pNode )
                {
                        pNode = new Node_t;
                }
                pNode->elem = init;
                Push( pNode );
        }

        bool PopItem( T *pResult )
        {
                Node_t *pNode = Pop();
                if ( !pNode )
                        return false;
                *pResult = pNode->elem;
                m_FreeList.Push( pNode );
                return true;
        }

        Node_t *Detach()
        {
                return (Node_t *)CTSListBase::Detach();
        }

        void FreeNode( Node_t *pNode )
        {
                m_FreeList.Push( pNode );
        }

private:
        CTSListBase m_FreeList;
} TSLIST_HEAD_ALIGN_POST;

//-----------------------------------------------------------------------------
// Lock free queue
//
// A special consideration: the element type should be simple. This code
// actually dereferences freed nodes as part of pop, but later detects
// that. If the item in the queue is a complex type, only bad things can
// come of that. Also, therefore, if you're using Push/Pop instead of
// push item, be aware that the node memory cannot be freed until
// all threads that might have been popping have completed the pop.
// The PushItem()/PopItem() for handles this by keeping a persistent
// free list. Dont mix Push/PushItem. Note also nodes will be freed at the end,
// and are expected to have been allocated with operator new.
//-----------------------------------------------------------------------------

template <typename T, bool bTestOptimizerUnused = false>
class TSLIST_HEAD_ALIGN CTSQueue
{
public:

        // override new/delete so we can guarantee 8-byte aligned allocs
        static void * operator new(size_t size)
        {
                CTSQueue *pNode = (CTSQueue *)MemAlloc_AllocAlignedFileLine( size, TSLIST_HEAD_ALIGNMENT, __FILE__, __LINE__ );
                return pNode;
        }

        // override new/delete so we can guarantee 8-byte aligned allocs
        static void * operator new(size_t size, int nBlockUse, const char *pFileName, int nLine)
        {
                CTSQueue *pNode = (CTSQueue *)MemAlloc_AllocAlignedFileLine( size, TSLIST_HEAD_ALIGNMENT, pFileName, nLine );
                return pNode;
        }

        static void operator delete(void *p)
        {
                MemAlloc_FreeAligned( p );
        }

        static void operator delete(void *p, int nBlockUse, const char *pFileName, int nLine)
        {
                MemAlloc_FreeAligned( p );
        }

private:
        // These ain't gonna work
        static void * operator new[]( size_t size ) = delete;

        static void operator delete []( void *p ) = delete;

public:

        struct TSLIST_NODE_ALIGN Node_t
        {
                // override new/delete so we can guarantee 8-byte aligned allocs
                static void * operator new(size_t size)
                {
                        Node_t *pNode = (Node_t *)MemAlloc_AllocAlignedFileLine( size, TSLIST_HEAD_ALIGNMENT, __FILE__, __LINE__ );
                        return pNode;
                }

                static void * operator new(size_t size, int nBlockUse, const char *pFileName, int nLine)
                {
                        Node_t *pNode = (Node_t *)MemAlloc_AllocAlignedFileLine( size, TSLIST_HEAD_ALIGNMENT, pFileName, nLine );
                        return pNode;
                }

                static void operator delete(void *p)
                {
                        MemAlloc_FreeAligned( p );
                }

                static void operator delete(void *p, int nBlockUse, const char *pFileName, int nLine)
                {
                        MemAlloc_FreeAligned( p );
                }

                Node_t() {}
                Node_t( const T &init ) : elem( init ) {}

                Node_t *pNext;
                T elem;
        } TSLIST_NODE_ALIGN_POST;

    static_assert( sizeof(Node_t) >= sizeof(TSLNodeBase_t) );

        struct TSLIST_HEAD_ALIGN NodeLink_t
        {
                // override new/delete so we can guarantee 8-byte aligned allocs
                static void * operator new(size_t size)
                {
                        NodeLink_t *pNode = (NodeLink_t *)MemAlloc_AllocAlignedFileLine( size, TSLIST_HEAD_ALIGNMENT, __FILE__, __LINE__ );
                        return pNode;
                }

                static void operator delete(void *p)
                {
                        MemAlloc_FreeAligned( p );
                }

                Node_t *pNode;
                intp	sequence;
        } TSLIST_HEAD_ALIGN_POST;

        CTSQueue()
        {
                m_Head = std::make_shared<NodeLink_t>();
                m_Tail = std::make_shared<NodeLink_t>();
                if ( reinterpret_cast<std::uintptr_t>(m_Head.get()) % TSLIST_HEAD_ALIGNMENT != 0 )
                {
                        Error( "CTSQueue: Misaligned queue\n" );
                        DebuggerBreak();
                }
                if ( reinterpret_cast<std::uintptr_t>(m_Tail.get()) % TSLIST_HEAD_ALIGNMENT != 0 )
                {
                        Error( "CTSQueue: Misaligned queue\n" );
                        DebuggerBreak();
                }
                m_Count = 0;
                m_Head->sequence = m_Tail->sequence = 0;
                m_Head->pNode = m_Tail->pNode = new Node_t; // list always contains a dummy node
                m_Head->pNode->pNext = End();
                purgedOrDeleted = false;
        }

        ~CTSQueue()
        {
                Purge();
                Assert( m_Count == 0 );
                Assert( m_Head->pNode == m_Tail->pNode );
                Assert( m_Head->pNode->pNext == End() );
                delete m_Head->pNode;
        }

        // Note: Purge, RemoveAll, and Validate are *not* threadsafe
        void Purge()
        {
            purgedOrDeleted = true;
                if ( IsDebug() )
                {
                        ValidateQueue();
                }

                Node_t *pNode;
                while ( (pNode = Pop()) != NULL )
                {
                        delete pNode;
                }

                while ( (pNode = (Node_t *)m_FreeNodes.Pop()) != NULL )
                {
                        delete pNode;
                }

                Assert( m_Count == 0 );
                Assert( m_Head->pNode == m_Tail->pNode );
                Assert( m_Head->pNode->pNext == End() );

                m_Count = 0;
                m_Head->sequence = m_Tail->sequence = 0;
                m_Head->pNode = m_Tail->pNode = new Node_t; // list always contains a dummy node
                m_Head->pNode->pNext = End();
        }

        void RemoveAll()
        {
                if ( IsDebug() )
                {
                        ValidateQueue();
                }

                Node_t *pNode;
                while ( (pNode = Pop()) != NULL )
                {
                        m_FreeNodes.Push( (TSLNodeBase_t *)pNode );
                }
        }

        bool ValidateQueue()
        {
                if ( IsDebug() )
                {
                        bool bResult = true;
                        int nNodes = 0;
                        if ( m_Tail->pNode->pNext != End() )
                        {
                                DebuggerBreakIfDebugging();
                                bResult = false;
                        }

                        if ( m_Count == 0 )
                        {
                                if ( m_Head->pNode != m_Tail->pNode )
                                {
                                        DebuggerBreakIfDebugging();
                                        bResult = false;
                                }
                        }

                        Node_t *pNode = m_Head->pNode;
                        while ( pNode != End() )
                        {
                                nNodes++;
                                pNode = pNode->pNext;
                        }

                        nNodes--;// skip dummy node

                        if ( nNodes != m_Count )
                        {
                                DebuggerBreakIfDebugging();
                                bResult = false;
                        }

                        if ( !bResult )
                        {
                                Msg( "Corrupt CTSQueueDetected" );
                        }

                        return bResult;
                }
                else
                {
                        return true;
                }
        }



        Node_t *Push( Node_t *pNode )
        {
#ifdef _DEBUG
                if ( reinterpret_cast<std::uintptr_t>(pNode) % TSLIST_NODE_ALIGNMENT != 0 )
                {
                        Error( "CTSListBase: Misaligned node\n" );
                        DebuggerBreak();
                }
#endif

                std::shared_ptr<NodeLink_t> oldTail;

                pNode->pNext = End();

                for ( ;; )
                {
                        oldTail = std::atomic_load(&m_Tail);

                        if ( InterlockedCompareExchangeNode( &(oldTail->pNode->pNext), pNode, End() ) == End() )
                        {
                                break;
                        }
                        else
                        {
                                // Another thread is trying to push, help it along
                                FinishPush( oldTail->pNode->pNext, oldTail );
                        }
                }

                FinishPush( pNode, oldTail ); // This can fail if another thread pushed between the sequence and node grabs above. Later pushes or pops corrects

                ++m_Count;

                return oldTail->pNode;
        }

        Node_t *Pop()
        {
                std::shared_ptr<NodeLink_t> oldHead = std::atomic_load(&m_Head);
                std::shared_ptr<NodeLink_t> newHead = std::make_shared<NodeLink_t>();

                T elem;

                for ( ;; )
                {
                        std::shared_ptr<NodeLink_t> tail = std::atomic_load(&m_Tail);

                        const auto pNext = oldHead->pNode->pNext;

                        // Checking pNext only to force optimizer to not reorder the assignment
                        // to pNext and the compare of the sequence
                        // may be removed here ?
                        if ( !pNext || oldHead->sequence != std::atomic_load(&m_Head)->sequence ) {
                                oldHead = std::atomic_load(&m_Head);
                                continue;
                        }

                        if ( oldHead->pNode == tail->pNode )
                        {
                                if ( pNext == End() )
                                        return nullptr;

                                // Another thread is trying to push, help it along
                                FinishPush( pNext, tail );
                                continue;
                        }

                        if ( pNext != End() )
                        {
                                elem = pNext->elem; // NOTE: next could be a freed node here, by design
                                newHead->pNode = pNext;
                                newHead->sequence = oldHead->sequence + 1;
                                if (std::atomic_compare_exchange_weak(&m_Head, &oldHead, newHead))
                                {
                                        ThreadMemoryBarrier();

                                        break;
                                }
                        }
                }

                --m_Count;
                oldHead->pNode->elem = elem;
                return oldHead->pNode;
        }

        void FreeNode( Node_t *pNode )
        {
                m_FreeNodes.Push( (TSLNodeBase_t *)pNode );
        }

        void PushItem( const T &init )
        {
                Node_t *pNode = (Node_t *)m_FreeNodes.Pop();
                if ( pNode )
                {
                        pNode->elem = init;
                }
                else
                {
                        pNode = new Node_t( init );
                }
                Push( pNode );
        }

        bool PopItem( T *pResult )
        {
                Node_t *pNode = Pop();
                if ( !pNode )
                        return false;

                *pResult = pNode->elem;
                m_FreeNodes.Push( (TSLNodeBase_t *)pNode );
                return true;
        }

        int Count() const
        {
            return m_Count;
        }

private:
        void FinishPush( Node_t *pNode, std::shared_ptr<NodeLink_t> oldTail )
        {
            std::shared_ptr<NodeLink_t> newTail = std::make_shared<NodeLink_t>();

            newTail->pNode = pNode;
            newTail->sequence = oldTail->sequence + 1;

            ThreadMemoryBarrier();

            std::atomic_compare_exchange_strong(&m_Tail, &oldTail, newTail);
        }

        Node_t *End() { return reinterpret_cast<Node_t *>(this); } // just need a unique signifier

        Node_t *InterlockedCompareExchangeNode( Node_t * volatile *ppNode, Node_t *value, Node_t *comperand )
        {
            return (Node_t *)::ThreadInterlockedCompareExchangePointer( (void **)ppNode, value, comperand );
        }


        std::shared_ptr<NodeLink_t> m_Head;
        std::shared_ptr<NodeLink_t> m_Tail;

        std::atomic<int> m_Count;

        CTSListBase m_FreeNodes;

        std::atomic_bool purgedOrDeleted;
} TSLIST_NODE_ALIGN_POST;

