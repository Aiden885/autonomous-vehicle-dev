/**
 * @file apaSearchTypes.h
 * @brief APA search-planning foundation types extracted from MATLAB search assets.
 *
 * Source:
 *   - HybridNode.m
 *   - hybrid_node.m
 *   - HashNodesList.m
 *   - PriorityQueueNodesList.m
 *   - getTotalCost.m
 *   - Enum*.m via apaEnums.h
 *
 * Date: 2026-04-14
 */
#ifndef APA_SEARCH_TYPES_H
#define APA_SEARCH_TYPES_H

#include "apaEnums.h"
#include "apaMathUtils.h"
#include "baseTypes.h"

/* Expanded parking graph is compile-time opt-in pending frozen-scene validation. */
#ifndef APA_ENABLE_HT_SEARCH_MODEL
#define APA_ENABLE_HT_SEARCH_MODEL 0
#endif
#if APA_ENABLE_HT_SEARCH_MODEL
#define APA_HASH_CAPACITY 16384
#define APA_PQ_CAPACITY 16384
#else
#define APA_HASH_CAPACITY 1000
#define APA_PQ_CAPACITY 1000
#define APA_HASH_MAX_NODES_PER_BUCKET 10
#endif
/* Colliding hash keys share a chain; only the node pool limits capacity. */
#define APA_HASH_TABLE_SIZE 25013
#define APA_RS_MAX_SEGMENTS 5
#define APA_RS_TYPE_COUNT 18
#define APA_GRID_HASH_STRIDE 480

typedef struct {
  UInteger16 Xidx;
  UInteger16 Yidx;
  UInteger16 Yawidx;
  /* Previous gear and steer state; zero when unavailable. */
  UInteger16 MotionClass;
  Real D;
  Real Delta;
  Real X;
  Real Y;
  Real Theta;
  Integer16 ParentNodeIndex;
  UInteger8 SegCnt;
  Real TrajCost;
  Real HeuristicCost;
  Real TotalCost;
} ApaHybridNode_t;

static inline void apaNodeInit(ApaHybridNode_t *node) {
  node->Xidx = 0U;
  node->Yidx = 0U;
  node->Yawidx = 0U;
  node->MotionClass = 0U;
  node->D = 0.0f;
  node->Delta = 0.0f;
  node->X = 0.0f;
  node->Y = 0.0f;
  node->Theta = 0.0f;
  node->ParentNodeIndex = -1;
  node->SegCnt = 0U;
  node->TrajCost = 0.0f;
  node->HeuristicCost = 0.0f;
  node->TotalCost = 0.0f;
}

static inline void apaNodeSetTrajCost(ApaHybridNode_t *node, Real cost) {
  node->TrajCost = cost;
}

static inline void apaNodeSetHeuristicCost(ApaHybridNode_t *node, Real cost) {
  node->HeuristicCost = cost;
}

typedef enum {
  APA_RS_SEG_NONE = 0,
  APA_RS_SEG_LEFT = 1,
  APA_RS_SEG_RIGHT = 2,
  APA_RS_SEG_STRAIGHT = 3
} ApaRsSegType_e;

/* Mirrors RSPath.Types row order, converted from MATLAB 1-based to C 0-based. */
typedef enum {
  APA_RS_PATTERN_LRL = 0,
  APA_RS_PATTERN_RLR = 1,
  APA_RS_PATTERN_LRLR = 2,
  APA_RS_PATTERN_RLRL = 3,
  APA_RS_PATTERN_LRSL = 4,
  APA_RS_PATTERN_RLSR = 5,
  APA_RS_PATTERN_LSRL = 6,
  APA_RS_PATTERN_RSLR = 7,
  APA_RS_PATTERN_LRSR = 8,
  APA_RS_PATTERN_RLSL = 9,
  APA_RS_PATTERN_RSRL = 10,
  APA_RS_PATTERN_LSLR = 11,
  APA_RS_PATTERN_LSR = 12,
  APA_RS_PATTERN_RSL = 13,
  APA_RS_PATTERN_LSL = 14,
  APA_RS_PATTERN_RSR = 15,
  APA_RS_PATTERN_LRSLR = 16,
  APA_RS_PATTERN_RLSRL = 17
} ApaRsPattern_e;

typedef struct {
  Integer Type[APA_RS_MAX_SEGMENTS];
  Real Lengths[APA_RS_MAX_SEGMENTS];
  Real TotalLength;
} ApaRsPath_t;

typedef struct {
  Integer SegType[APA_RS_MAX_SEGMENTS];
} ApaRsPathPattern_t;

static const ApaRsPathPattern_t APA_RS_PATH_TYPE_TABLE[APA_RS_TYPE_COUNT] = {
    {{APA_RS_SEG_LEFT, APA_RS_SEG_RIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_NONE, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_RIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_RIGHT, APA_RS_SEG_NONE, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_LEFT, APA_RS_SEG_RIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_RIGHT, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_RIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_RIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_LEFT, APA_RS_SEG_RIGHT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_RIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_RIGHT, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_LEFT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_RIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_RIGHT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_RIGHT, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_LEFT, APA_RS_SEG_RIGHT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_RIGHT, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_RIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_RIGHT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_RIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_LEFT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_RIGHT, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_LEFT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_RIGHT, APA_RS_SEG_NONE, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_RIGHT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_NONE, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_LEFT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_NONE, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_RIGHT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_RIGHT, APA_RS_SEG_NONE, APA_RS_SEG_NONE}},
    {{APA_RS_SEG_LEFT, APA_RS_SEG_RIGHT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_RIGHT}},
    {{APA_RS_SEG_RIGHT, APA_RS_SEG_LEFT, APA_RS_SEG_STRAIGHT, APA_RS_SEG_RIGHT, APA_RS_SEG_LEFT}}};

static inline void apaRsPathInit(ApaRsPath_t *path) {
  Integer i;

  for (i = 0; i < APA_RS_MAX_SEGMENTS; i = i + 1) {
    path->Type[i] = APA_RS_SEG_NONE;
    path->Lengths[i] = 0.0f;
  }
  path->TotalLength = 0.0f;
}

static inline Boolean apaRsPathLoadPattern(Integer patternIndex, ApaRsPath_t *path) {
  Integer i;

  if (patternIndex < 0 || patternIndex >= APA_RS_TYPE_COUNT) {
    apaRsPathInit(path);
    return false;
  }

  for (i = 0; i < APA_RS_MAX_SEGMENTS; i = i + 1) {
    path->Type[i] = APA_RS_PATH_TYPE_TABLE[patternIndex].SegType[i];
  }
  return true;
}

static inline Boolean apaRsPathInitWithLengths(
    ApaRsPath_t *path, Integer patternIndex, Real t, Real u, Real v, Real w, Real x) {
  Real totalLength;

  if (!apaRsPathLoadPattern(patternIndex, path)) {
    return false;
  }

  path->Lengths[0] = t;
  path->Lengths[1] = u;
  path->Lengths[2] = v;
  path->Lengths[3] = w;
  path->Lengths[4] = x;

  totalLength = apaMathAbs(t) + apaMathAbs(u) + apaMathAbs(v) + apaMathAbs(w) + apaMathAbs(x);
  path->TotalLength = totalLength;
  return true;
}

typedef enum {
  APA_NODE_STATE_FREE = 0,
  APA_NODE_STATE_OPEN = 1,
  APA_NODE_STATE_CLOSED = 2
} ApaNodeState_e;

#define APA_NODE_STATE_BITS_PER_SLOT 2
#define APA_NODE_STATE_SLOTS_PER_WORD 4
#define APA_NODE_STATE_WORD_COUNT                                                                  \
  ((APA_HASH_CAPACITY + APA_NODE_STATE_SLOTS_PER_WORD - 1) / APA_NODE_STATE_SLOTS_PER_WORD)

typedef struct {
  ApaHybridNode_t Nodes[APA_HASH_CAPACITY];
  UInteger8 NodeStateWords[APA_NODE_STATE_WORD_COUNT];
  Integer16 ValidNodeCnt;
  Integer16 FreeListHead;
} ApaNodePool_t;

typedef struct {
  Integer16 Head[APA_HASH_TABLE_SIZE];
  Integer16 Next[APA_HASH_CAPACITY];
  Integer16 ValidIndexCnt;
} ApaHashMap_t;

static inline Integer apaHashMapLinearHash(UInteger16 xidx, UInteger16 yidx, UInteger16 yawidx) {
  UInteger combined;
  UInteger bucket;

  combined = ((UInteger)yawidx * (UInteger)APA_GRID_HASH_STRIDE * (UInteger)APA_GRID_HASH_STRIDE) +
             ((UInteger)yidx * (UInteger)APA_GRID_HASH_STRIDE) + (UInteger)xidx;
  bucket = combined % (UInteger)APA_HASH_TABLE_SIZE;
  return (Integer)bucket;
}

static inline Integer apaNodePoolStateWordIndex(Integer index) {
  return index / APA_NODE_STATE_SLOTS_PER_WORD;
}

static inline Integer apaNodePoolStateShift(Integer index) {
  Integer wordOffset;

  wordOffset = index % APA_NODE_STATE_SLOTS_PER_WORD;
  return wordOffset * APA_NODE_STATE_BITS_PER_SLOT;
}

static inline UInteger8 apaNodePoolGetState(const ApaNodePool_t *pool, Integer index) {
  Integer wordIndex;
  Integer stateShift;
  UInteger stateWord;
  UInteger stateValue;

  if (index < 0 || index >= APA_HASH_CAPACITY) {
    return (UInteger8)APA_NODE_STATE_FREE;
  }

  wordIndex = apaNodePoolStateWordIndex(index);
  stateShift = apaNodePoolStateShift(index);
  stateWord = (UInteger)pool->NodeStateWords[wordIndex];
  stateValue = (stateWord >> (UInteger)stateShift) & 0x3U;
  return (UInteger8)stateValue;
}

static inline void apaNodePoolWriteState(ApaNodePool_t *pool, Integer index, UInteger8 nodeState) {
  Integer wordIndex;
  Integer stateShift;
  UInteger stateWord;
  UInteger stateMask;
  UInteger stateValue;

  if (index < 0 || index >= APA_HASH_CAPACITY) {
    return;
  }

  wordIndex = apaNodePoolStateWordIndex(index);
  stateShift = apaNodePoolStateShift(index);
  stateWord = (UInteger)pool->NodeStateWords[wordIndex];
  stateMask = (UInteger)0x3U << (UInteger)stateShift;
  stateValue = ((UInteger)nodeState & 0x3U) << (UInteger)stateShift;
  stateWord = (stateWord & (~stateMask)) | stateValue;
  pool->NodeStateWords[wordIndex] = (UInteger8)stateWord;
}

static inline void apaNodePoolInit(ApaNodePool_t *pool) {
  Integer i;

  pool->ValidNodeCnt = 0;
  pool->FreeListHead = 0;
  for (i = 0; i < APA_NODE_STATE_WORD_COUNT; i = i + 1) {
    pool->NodeStateWords[i] = 0U;
  }
  for (i = 0; i < APA_HASH_CAPACITY; i = i + 1) {
    apaNodeInit(&pool->Nodes[i]);
  }
}

static inline Boolean apaNodePoolHasNode(const ApaNodePool_t *pool, Integer index) {
  if (index < 0 || index >= APA_HASH_CAPACITY) {
    return false;
  }

  return apaNodePoolGetState(pool, index) != (UInteger8)APA_NODE_STATE_FREE;
}

static inline void apaHashMapInit(ApaHashMap_t *map) {
  Integer i;

  map->ValidIndexCnt = 0;

  for (i = 0; i < APA_HASH_TABLE_SIZE; i = i + 1) {
    map->Head[i] = -1;
  }
  for (i = 0; i < APA_HASH_CAPACITY; i = i + 1) {
    map->Next[i] = -1;
  }
}

static inline Integer apaNodePoolFindFreeSlot(ApaNodePool_t *pool) {
  Integer i;

  for (i = pool->FreeListHead; i < APA_HASH_CAPACITY; i = i + 1) {
    if (apaNodePoolGetState(pool, i) == (UInteger8)APA_NODE_STATE_FREE) {
      pool->FreeListHead = i + 1;
      return i;
    }
  }

  return -1;
}

static inline Integer apaNodePoolAdd(ApaNodePool_t *pool,
                                     const ApaHybridNode_t *node,
                                     UInteger8 nodeState,
                                     Integer *outIndex) {
  Integer freeIndex;

  freeIndex = apaNodePoolFindFreeSlot(pool);
  if (freeIndex < 0) {
    *outIndex = -1;
    return 1;
  }

  pool->Nodes[freeIndex] = *node;
  apaNodePoolWriteState(pool, freeIndex, nodeState);
  pool->ValidNodeCnt = pool->ValidNodeCnt + 1;
  *outIndex = freeIndex;
  return 0;
}

static inline void apaNodePoolSetState(ApaNodePool_t *pool, Integer index, UInteger8 nodeState) {
  if (index < 0 || index >= APA_HASH_CAPACITY) {
    return;
  }
  if (!apaNodePoolHasNode(pool, index)) {
    return;
  }

  apaNodePoolWriteState(pool, index, nodeState);
}

static inline void apaNodePoolRemove(ApaNodePool_t *pool, Integer index) {
  if (index < 0 || index >= APA_HASH_CAPACITY) {
    return;
  }
  if (!apaNodePoolHasNode(pool, index)) {
    return;
  }

  apaNodePoolWriteState(pool, index, (UInteger8)APA_NODE_STATE_FREE);
  apaNodeInit(&pool->Nodes[index]);
  pool->ValidNodeCnt = pool->ValidNodeCnt - 1;
  if (index < pool->FreeListHead) {
    pool->FreeListHead = index;
  }
}

static inline Boolean apaNodeMatchesIndex(const ApaHybridNode_t *node,
                                          UInteger16 xidx,
                                          UInteger16 yidx,
                                          UInteger16 yawidx) {
  Boolean match;

  match = (node->Xidx == xidx) && (node->Yidx == yidx) && (node->Yawidx == yawidx);
  return match;
}

static inline Boolean apaHashMapContains(const ApaHashMap_t *map,
                                         const ApaNodePool_t *pool,
                                         UInteger16 xidx,
                                         UInteger16 yidx,
                                         UInteger16 yawidx,
                                         Integer *outIndex) {
  Integer bucket;
  Integer currentIndex;

  bucket = apaHashMapLinearHash(xidx, yidx, yawidx);
  currentIndex = map->Head[bucket];

  while (currentIndex >= 0 && currentIndex < APA_HASH_CAPACITY) {
    if (apaNodePoolHasNode(pool, currentIndex)) {
      if (apaNodeMatchesIndex(&pool->Nodes[currentIndex], xidx, yidx, yawidx)) {
        *outIndex = currentIndex;
        return true;
      }
    }
    currentIndex = map->Next[currentIndex];
  }

  *outIndex = -1;
  return false;
}

static inline Integer apaHashMapBucketNodeCount(const ApaHashMap_t *map, Integer bucket) {
  Integer count;
  Integer currentIndex;

  count = 0;
  currentIndex = -1;
  if (bucket >= 0 && bucket < APA_HASH_TABLE_SIZE) {
    currentIndex = map->Head[bucket];
  }
  while (currentIndex >= 0 && currentIndex < APA_HASH_CAPACITY) {
    count = count + 1;
    currentIndex = map->Next[currentIndex];
  }

  return count;
}

static inline Boolean apaHashMapContainsState(const ApaHashMap_t *map,
                                              const ApaNodePool_t *pool,
                                              const ApaHybridNode_t *node,
                                              Integer *outIndex) {
  Integer index = map->Head[apaHashMapLinearHash(node->Xidx, node->Yidx, node->Yawidx)];
  while (index >= 0 && index < APA_HASH_CAPACITY) {
    const ApaHybridNode_t *stored = &pool->Nodes[index];
    if (apaNodePoolHasNode(pool, index) && stored->MotionClass == node->MotionClass &&
        apaNodeMatchesIndex(stored, node->Xidx, node->Yidx, node->Yawidx)) {
      *outIndex = index;
      return true;
    }
    index = map->Next[index];
  }
  *outIndex = -1;
  return false;
}

static inline Integer
apaHashMapAddIndex(ApaHashMap_t *map, const ApaNodePool_t *pool, Integer index) {
  Integer bucket;
  Integer existingIndex;

  if (index < 0 || index >= APA_HASH_CAPACITY) {
    return 2;
  }
  if (!apaNodePoolHasNode(pool, index)) {
    return 2;
  }

  if (apaHashMapContainsState(map, pool, &pool->Nodes[index], &existingIndex)) {
    return 1;
  }

  bucket = apaHashMapLinearHash(
      pool->Nodes[index].Xidx, pool->Nodes[index].Yidx, pool->Nodes[index].Yawidx);
  if (map->ValidIndexCnt >= APA_HASH_CAPACITY) {
    return 3;
  }
#if !APA_ENABLE_HT_SEARCH_MODEL
  if (apaHashMapBucketNodeCount(map, bucket) >= APA_HASH_MAX_NODES_PER_BUCKET)
    return 3;
#endif
  map->Next[index] = map->Head[bucket];
  map->Head[bucket] = index;
  map->ValidIndexCnt = map->ValidIndexCnt + 1;

  return 0;
}

static inline Integer apaHashMapAdd(ApaHashMap_t *map,
                                    ApaNodePool_t *pool,
                                    const ApaHybridNode_t *node,
                                    UInteger8 nodeState,
                                    Integer *outIndex) {
  Integer addReason;
  Integer nodeIndex;

  addReason = apaNodePoolAdd(pool, node, nodeState, &nodeIndex);
  if (addReason != 0) {
    *outIndex = -1;
    return 2;
  }

  addReason = apaHashMapAddIndex(map, pool, nodeIndex);
  if (addReason != 0) {
    apaNodePoolRemove(pool, nodeIndex);
    *outIndex = -1;
    if (addReason == 3) {
      return 3;
    }
    return 1;
  }

  *outIndex = nodeIndex;
  return 0;
}

static inline void apaHashMapRemove(ApaHashMap_t *map, const ApaNodePool_t *pool, Integer index) {
  Integer bucket;
  Integer prevIndex;
  Integer currentIndex;
  Boolean searchOpen;

  if (index < 0 || index >= APA_HASH_CAPACITY) {
    return;
  }
  if (!apaNodePoolHasNode(pool, index)) {
    return;
  }

  bucket = apaHashMapLinearHash(
      pool->Nodes[index].Xidx, pool->Nodes[index].Yidx, pool->Nodes[index].Yawidx);
  prevIndex = -1;
  currentIndex = map->Head[bucket];
  searchOpen = (currentIndex >= 0);
  if (searchOpen) {
    searchOpen = (currentIndex < APA_HASH_CAPACITY);
  }

  while (searchOpen) {
    Integer nextIndex;

    nextIndex = map->Next[currentIndex];
    if (currentIndex == index) {
      if (prevIndex < 0) {
        map->Head[bucket] = nextIndex;
      } else {
        map->Next[prevIndex] = nextIndex;
      }
      map->Next[currentIndex] = -1;
      map->ValidIndexCnt = map->ValidIndexCnt - 1;
      searchOpen = false;
    } else {
      prevIndex = currentIndex;
      currentIndex = nextIndex;
      searchOpen = (currentIndex >= 0);
      if (searchOpen) {
        searchOpen = (currentIndex < APA_HASH_CAPACITY);
      }
    }
  }
}

static inline Boolean apaHashMapFindMinCostNode(const ApaHashMap_t *map,
                                                const ApaNodePool_t *pool,
                                                ApaHybridNode_t *outNode,
                                                Integer *outIndex) {
  Boolean found;
  Real minCost;
  Integer bucket;

  found = false;
  minCost = 0.0f;
  *outIndex = -1;
  apaNodeInit(outNode);

  for (bucket = 0; bucket < APA_HASH_TABLE_SIZE; bucket = bucket + 1) {
    Integer currentIndex;

    currentIndex = map->Head[bucket];
    while (currentIndex >= 0 && currentIndex < APA_HASH_CAPACITY) {
      if (apaNodePoolHasNode(pool, currentIndex)) {
        if (!found || pool->Nodes[currentIndex].TotalCost < minCost) {
          *outNode = pool->Nodes[currentIndex];
          *outIndex = currentIndex;
          minCost = pool->Nodes[currentIndex].TotalCost;
          found = true;
        }
      }
      currentIndex = map->Next[currentIndex];
    }
  }

  return found;
}

typedef struct {
  Integer16 NodeIndexByHeap[APA_PQ_CAPACITY];
  Real TotalCostByHeap[APA_PQ_CAPACITY];
  Integer16 HeapPosByNode[APA_HASH_CAPACITY];
  Integer16 Count;
} ApaPriorityQueue_t;

static inline void apaPqSlotInit(ApaPriorityQueue_t *queue, Integer heapIndex) {
  queue->NodeIndexByHeap[heapIndex] = -1;
  queue->TotalCostByHeap[heapIndex] = 0.0f;
}

static inline void apaPqSwap(ApaPriorityQueue_t *queue, Integer leftIndex, Integer rightIndex) {
  Integer16 tempNodeIndex;
  Real tempTotalCost;

  if (leftIndex == rightIndex) {
    return;
  }

  tempNodeIndex = queue->NodeIndexByHeap[leftIndex];
  tempTotalCost = queue->TotalCostByHeap[leftIndex];
  queue->NodeIndexByHeap[leftIndex] = queue->NodeIndexByHeap[rightIndex];
  queue->TotalCostByHeap[leftIndex] = queue->TotalCostByHeap[rightIndex];
  queue->NodeIndexByHeap[rightIndex] = tempNodeIndex;
  queue->TotalCostByHeap[rightIndex] = tempTotalCost;
  queue->HeapPosByNode[queue->NodeIndexByHeap[leftIndex]] = leftIndex;
  queue->HeapPosByNode[queue->NodeIndexByHeap[rightIndex]] = rightIndex;
}

static inline void apaPqInit(ApaPriorityQueue_t *queue) {
  Integer i;

  queue->Count = 0;
  for (i = 0; i < APA_PQ_CAPACITY; i = i + 1) {
    apaPqSlotInit(queue, i);
  }
  for (i = 0; i < APA_HASH_CAPACITY; i = i + 1) {
    queue->HeapPosByNode[i] = -1;
  }
}

static inline void apaPqHeapifyUp(ApaPriorityQueue_t *queue, Integer index) {
  Boolean loopFlag;

  loopFlag = true;
  while (loopFlag && index > 0) {
    Integer parentIndex;

    parentIndex = (index - 1) / 2;
    if (queue->TotalCostByHeap[index] < queue->TotalCostByHeap[parentIndex]) {
      apaPqSwap(queue, index, parentIndex);
      index = parentIndex;
    } else {
      loopFlag = false;
    }
  }
}

static inline void apaPqHeapifyDown(ApaPriorityQueue_t *queue, Integer index) {
  Boolean loopFlag;

  loopFlag = true;
  while (loopFlag) {
    Integer leftIndex;
    Integer rightIndex;
    Integer smallestIndex;

    leftIndex = (2 * index) + 1;
    rightIndex = leftIndex + 1;
    smallestIndex = index;

    if (leftIndex < queue->Count) {
      if (queue->TotalCostByHeap[leftIndex] < queue->TotalCostByHeap[smallestIndex]) {
        smallestIndex = leftIndex;
      }
    }

    if (rightIndex < queue->Count) {
      if (queue->TotalCostByHeap[rightIndex] < queue->TotalCostByHeap[smallestIndex]) {
        smallestIndex = rightIndex;
      }
    }

    if (smallestIndex == index) {
      loopFlag = false;
    } else {
      apaPqSwap(queue, index, smallestIndex);
      index = smallestIndex;
    }
  }
}

static inline Boolean apaPqPush(ApaPriorityQueue_t *queue, Integer nodeIndex, Real totalCost) {
  Integer insertIndex;

  if (queue->Count >= APA_PQ_CAPACITY) {
    return false;
  }
  if (nodeIndex < 0 || nodeIndex >= APA_HASH_CAPACITY) {
    return false;
  }
  if (queue->HeapPosByNode[nodeIndex] >= 0) {
    return false;
  }

  insertIndex = queue->Count;
  queue->NodeIndexByHeap[insertIndex] = (Integer16)nodeIndex;
  queue->TotalCostByHeap[insertIndex] = totalCost;
  queue->HeapPosByNode[nodeIndex] = insertIndex;
  queue->Count = queue->Count + 1;
  apaPqHeapifyUp(queue, insertIndex);
  return true;
}

static inline Boolean
apaPqTop(const ApaPriorityQueue_t *queue, Integer *outNodeIndex, Real *outTotalCost) {
  if (queue->Count <= 0) {
    if (outNodeIndex != (Integer *)0) {
      *outNodeIndex = -1;
    }
    if (outTotalCost != (Real *)0) {
      *outTotalCost = 0.0f;
    }
    return false;
  }

  if (outNodeIndex != (Integer *)0) {
    *outNodeIndex = queue->NodeIndexByHeap[0];
  }
  if (outTotalCost != (Real *)0) {
    *outTotalCost = queue->TotalCostByHeap[0];
  }
  return true;
}

static inline Boolean
apaPqPop(ApaPriorityQueue_t *queue, Integer *outNodeIndex, Real *outTotalCost) {
  Integer poppedNodeIndex;
  Real poppedTotalCost;

  if (queue->Count <= 0) {
    if (outNodeIndex != (Integer *)0) {
      *outNodeIndex = -1;
    }
    if (outTotalCost != (Real *)0) {
      *outTotalCost = 0.0f;
    }
    return false;
  }

  poppedNodeIndex = queue->NodeIndexByHeap[0];
  poppedTotalCost = queue->TotalCostByHeap[0];
  queue->HeapPosByNode[poppedNodeIndex] = -1;
  queue->Count = queue->Count - 1;
  if (queue->Count > 0) {
    queue->NodeIndexByHeap[0] = queue->NodeIndexByHeap[queue->Count];
    queue->TotalCostByHeap[0] = queue->TotalCostByHeap[queue->Count];
    queue->HeapPosByNode[queue->NodeIndexByHeap[0]] = 0;
    apaPqSlotInit(queue, queue->Count);
    apaPqHeapifyDown(queue, 0);
  } else {
    apaPqSlotInit(queue, 0);
  }

  if (outNodeIndex != (Integer *)0) {
    *outNodeIndex = poppedNodeIndex;
  }
  if (outTotalCost != (Real *)0) {
    *outTotalCost = poppedTotalCost;
  }
  return true;
}

static inline Boolean
apaPqReplaceNode(ApaPriorityQueue_t *queue, Integer nodeIndex, Real totalCost) {
  Integer heapIndex;
  Real originalCost;

  if (nodeIndex < 0 || nodeIndex >= APA_HASH_CAPACITY) {
    return false;
  }

  heapIndex = queue->HeapPosByNode[nodeIndex];
  if (heapIndex < 0 || heapIndex >= queue->Count) {
    return false;
  }
  if (queue->NodeIndexByHeap[heapIndex] != nodeIndex) {
    return false;
  }

  originalCost = queue->TotalCostByHeap[heapIndex];
  queue->TotalCostByHeap[heapIndex] = totalCost;
  if (totalCost < originalCost) {
    apaPqHeapifyUp(queue, heapIndex);
  } else if (totalCost > originalCost) {
    apaPqHeapifyDown(queue, heapIndex);
  }

  return true;
}

static inline Boolean apaPqIsMinHeap(const ApaPriorityQueue_t *queue) {
  Integer i;

  for (i = 0; i < queue->Count; i = i + 1) {
    Integer leftIndex;
    Integer rightIndex;
    Integer nodeIndex;

    nodeIndex = queue->NodeIndexByHeap[i];
    if (nodeIndex < 0 || nodeIndex >= APA_HASH_CAPACITY) {
      return false;
    }
    if (queue->HeapPosByNode[nodeIndex] != i) {
      return false;
    }

    leftIndex = (2 * i) + 1;
    rightIndex = leftIndex + 1;

    if (leftIndex < queue->Count) {
      if (queue->TotalCostByHeap[leftIndex] < queue->TotalCostByHeap[i]) {
        return false;
      }
    }
    if (rightIndex < queue->Count) {
      if (queue->TotalCostByHeap[rightIndex] < queue->TotalCostByHeap[i]) {
        return false;
      }
    }
  }

  return true;
}

static inline Boolean apaPqEmpty(const ApaPriorityQueue_t *queue) {
  return (queue->Count <= 0);
}

static inline void apaPqRebuild(ApaPriorityQueue_t *queue) {
  Integer index;
  Integer i;

  for (i = 0; i < APA_HASH_CAPACITY; i = i + 1) {
    queue->HeapPosByNode[i] = -1;
  }
  for (i = 0; i < queue->Count; i = i + 1) {
    queue->HeapPosByNode[queue->NodeIndexByHeap[i]] = i;
  }

  if (queue->Count <= 1) {
    return;
  }

  for (index = (queue->Count / 2) - 1; index >= 0; index = index - 1) {
    apaPqHeapifyDown(queue, index);
  }
}

static inline Real
apaTotalCost(Real trajCost, Real heuristicCost, Real heuristicWeight, Boolean useGCost) {
  if (useGCost) {
    return trajCost + (heuristicWeight * heuristicCost);
  }

  return trajCost;
}

#endif
