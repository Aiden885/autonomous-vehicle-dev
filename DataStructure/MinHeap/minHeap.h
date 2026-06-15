#ifndef MINHEAP_H
#define MINHEAP_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 最小堆节点结构体（键值对）
 * @en_name HeapNode
 * @cn_name 堆节点
 * @tag STRUCT
 * @field name=Key,   type=double, unit=1, desc=优先级键值，越小优先级越高
 * @field name=Value, type=int,    unit=1, desc=存储的节点ID
 * @version 1.0
 * @date 2026-03-27
 * @author aiden
 */
typedef struct {
    double Key;
    int    Value;
} HeapNode;

/**
 * @brief 最小堆结构体，用于A*全局规划的开放列表
 * @en_name MinHeap
 * @cn_name 最小堆
 * @tag STRUCT
 * @field name=Data,     type=Array<HeapNode,[N]>, unit=1, desc=堆数组
 * @field name=Count,    type=int,                 unit=1, desc=当前元素数量
 * @field name=Capacity, type=int,                 unit=1, desc=最大容量
 * @version 1.0
 * @date 2026-03-27
 * @author aiden
 */
typedef struct {
    HeapNode *Data;
    int       Count;
    int       Capacity;
} MinHeap;

void minHeapInit(MinHeap *heap, int capacity);
void minHeapPush(MinHeap *heap, double key, int value);
void minHeapPop(MinHeap *heap, HeapNode *out);
bool minHeapIsEmpty(const MinHeap *heap);
void minHeapFree(MinHeap *heap);

#ifdef __cplusplus
}
#endif
#endif /* MINHEAP_H */
