#include "minHeap.h"
#include <stdlib.h>

static void minHeapSwapNode(HeapNode *leftNode, HeapNode *rightNode);
static bool minHeapEnsureCapacity(MinHeap *heap);
static void minHeapSiftUp(MinHeap *heap, int index);
static void minHeapSiftDown(MinHeap *heap, int index);

/**
 * @brief 交换最小堆中的两个节点
 * @en_name minHeapSwapNode
 * @cn_name 交换堆节点
 * @type widget
 * @param[IN] HeapNode *leftNode 左侧节点指针
 * @param[IN] HeapNode *rightNode 右侧节点指针
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static void minHeapSwapNode(HeapNode *leftNode, HeapNode *rightNode)
{
    bool leftValid;
    bool rightValid;
    bool allValid;
    HeapNode tempNode;

    leftValid = (leftNode != NULL);
    rightValid = (rightNode != NULL);
    allValid = (leftValid && rightValid);

    if (allValid) {
        tempNode = *leftNode;
        *leftNode = *rightNode;
        *rightNode = tempNode;
    }
}

/**
 * @brief 确保最小堆有可用容量，不足时自动扩容2倍
 * @en_name minHeapEnsureCapacity
 * @cn_name 扩展堆容量
 * @type widget
 * @param[IN] MinHeap *heap 最小堆指针
 * @retval bool true容量可用，false扩容失败
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static bool minHeapEnsureCapacity(MinHeap *heap)
{
    bool validHeap;
    bool missingStorage;
    bool fullCapacity;
    bool needGrow;
    bool hasPositiveCapacity;
    bool allocSuccess;
    bool result;
    int newCapacity;
    HeapNode *newData;
    size_t requiredSize;

    result = false;
    validHeap = (heap != NULL);
    missingStorage = false;
    fullCapacity = false;
    needGrow = false;
    hasPositiveCapacity = false;
    allocSuccess = false;
    newCapacity = 0;
    newData = NULL;
    requiredSize = 0U;

    if (validHeap) {
        missingStorage = (heap->Data == NULL);
        fullCapacity = (heap->Count >= heap->Capacity);
        needGrow = (missingStorage || fullCapacity);

        if (needGrow) {
            hasPositiveCapacity = (heap->Capacity > 0);
            if (hasPositiveCapacity) {
                if (missingStorage) {
                    newCapacity = heap->Capacity;
                } else {
                    newCapacity = heap->Capacity * 2;
                }
            } else {
                newCapacity = 1;
            }

            requiredSize = (size_t)newCapacity * sizeof(HeapNode);
            newData = (HeapNode *)realloc(heap->Data, requiredSize);
            allocSuccess = (newData != NULL);

            if (allocSuccess) {
                heap->Data = newData;
                heap->Capacity = newCapacity;
                result = true;
            }
        } else {
            result = true;
        }
    }

    return result;
}

/**
 * @brief 对新增节点执行上滤，恢复堆序
 * @en_name minHeapSiftUp
 * @cn_name 上滤调整堆
 * @type widget
 * @param[IN] MinHeap *heap 最小堆指针
 * @param[IN] int index 需要上滤的节点下标
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static void minHeapSiftUp(MinHeap *heap, int index)
{
    bool validHeap;
    bool validData;
    bool validIndex;
    bool allValid;
    bool keepRunning;
    bool needSwap;
    int currentIndex;
    int parentIndex;

    validHeap = (heap != NULL);
    validData = false;
    validIndex = false;
    allValid = false;
    currentIndex = index;
    parentIndex = 0;

    if (validHeap) {
        validData = (heap->Data != NULL);
        validIndex = (index >= 0) && (index < heap->Count);
        allValid = (validData && validIndex);
    }

    keepRunning = (allValid && (currentIndex > 0));
    while (keepRunning) {
        parentIndex = (currentIndex - 1) / 2;
        needSwap = (heap->Data[parentIndex].Key > heap->Data[currentIndex].Key);

        if (needSwap) {
            minHeapSwapNode(&heap->Data[parentIndex], &heap->Data[currentIndex]);
            currentIndex = parentIndex;
        } else {
            currentIndex = 0;
        }

        keepRunning = (currentIndex > 0);
    }
}

/**
 * @brief 对根节点执行下滤，恢复堆序
 * @en_name minHeapSiftDown
 * @cn_name 下滤调整堆
 * @type widget
 * @param[IN] MinHeap *heap 最小堆指针
 * @param[IN] int index 需要下滤的节点下标
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static void minHeapSiftDown(MinHeap *heap, int index)
{
    bool validHeap;
    bool validData;
    bool validIndex;
    bool allValid;
    bool keepRunning;
    bool hasLeftChild;
    bool hasRightChild;
    bool rightIsSmaller;
    bool childIsSmaller;
    int currentIndex;
    int lastIndex;
    int leftChildIndex;
    int rightChildIndex;
    int smallestIndex;

    validHeap = (heap != NULL);
    validData = false;
    validIndex = false;
    allValid = false;
    currentIndex = index;
    lastIndex = -1;
    leftChildIndex = -1;
    rightChildIndex = -1;
    smallestIndex = -1;

    if (validHeap) {
        validData = (heap->Data != NULL);
        validIndex = (index >= 0) && (index < heap->Count);
        allValid = (validData && validIndex);
    }

    keepRunning = allValid;
    while (keepRunning) {
        lastIndex = heap->Count - 1;
        leftChildIndex = (currentIndex * 2) + 1;
        rightChildIndex = leftChildIndex + 1;
        smallestIndex = currentIndex;
        hasLeftChild = (leftChildIndex <= lastIndex);

        if (hasLeftChild) {
            smallestIndex = leftChildIndex;
            hasRightChild = (rightChildIndex <= lastIndex);

            if (hasRightChild) {
                rightIsSmaller = (heap->Data[rightChildIndex].Key < heap->Data[leftChildIndex].Key);
                if (rightIsSmaller) {
                    smallestIndex = rightChildIndex;
                }
            }

            childIsSmaller = (heap->Data[smallestIndex].Key < heap->Data[currentIndex].Key);
            if (childIsSmaller) {
                minHeapSwapNode(&heap->Data[currentIndex], &heap->Data[smallestIndex]);
                currentIndex = smallestIndex;
            } else {
                keepRunning = false;
            }
        } else {
            keepRunning = false;
        }
    }
}

/**
 * @brief 初始化最小堆，分配初始内存
 * @en_name minHeapInit
 * @cn_name 最小堆初始化
 * @type widget
 * @param[IN] MinHeap *heap 最小堆指针
 * @param[IN] int capacity 初始容量
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void minHeapInit(MinHeap *heap, int capacity)
{
    bool validHeap;
    bool validCapacity;
    bool allocSuccess;
    size_t requiredSize;
    HeapNode *newData;

    validHeap = (heap != NULL);
    validCapacity = false;
    allocSuccess = false;
    requiredSize = 0U;
    newData = NULL;

    if (validHeap) {
        heap->Data = NULL;
        heap->Count = 0;
        heap->Capacity = 0;

        validCapacity = (capacity > 0);
        if (validCapacity) {
            requiredSize = (size_t)capacity * sizeof(HeapNode);
            newData = (HeapNode *)malloc(requiredSize);
            allocSuccess = (newData != NULL);

            if (allocSuccess) {
                heap->Data = newData;
                heap->Capacity = capacity;
            }
        }
    }
}

/**
 * @brief 向最小堆插入一个节点
 * @en_name minHeapPush
 * @cn_name 最小堆插入
 * @type widget
 * @param[IN] MinHeap *heap 最小堆指针
 * @param[IN] double key 节点键值（优先级）
 * @param[IN] int value 节点数据值（节点ID）
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void minHeapPush(MinHeap *heap, double key, int value)
{
    bool validHeap;
    bool readyCapacity;
    bool validData;
    bool canInsert;
    int insertIndex;

    validHeap = (heap != NULL);
    readyCapacity = false;
    validData = false;
    canInsert = false;
    insertIndex = -1;

    if (validHeap) {
        readyCapacity = minHeapEnsureCapacity(heap);
        validData = (heap->Data != NULL);
        canInsert = (readyCapacity && validData);

        if (canInsert) {
            insertIndex = heap->Count;
            heap->Data[insertIndex].Key = key;
            heap->Data[insertIndex].Value = value;
            heap->Count = heap->Count + 1;
            minHeapSiftUp(heap, insertIndex);
        }
    }
}

/**
 * @brief 弹出最小堆顶部节点，堆为空时out的Key=-1.0，Value=-1
 * @en_name minHeapPop
 * @cn_name 最小堆弹出
 * @type widget
 * @param[IN] MinHeap *heap 最小堆指针
 * @param[OUT] HeapNode *out 弹出节点输出指针
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void minHeapPop(MinHeap *heap, HeapNode *out)
{
    bool validOut;
    bool validHeap;
    bool validData;
    bool notEmpty;
    bool canPop;
    bool hasRemaining;
    int lastIndex;

    validOut = (out != NULL);
    validHeap = (heap != NULL);
    validData = false;
    notEmpty = false;
    canPop = false;
    hasRemaining = false;
    lastIndex = -1;

    if (validOut) {
        out->Key = -1.0;
        out->Value = -1;
    }

    if (validHeap) {
        validData = (heap->Data != NULL);
        notEmpty = (heap->Count > 0);
        canPop = (validData && notEmpty);

        if (canPop) {
            lastIndex = heap->Count - 1;

            if (validOut) {
                *out = heap->Data[0];
            }

            heap->Count = heap->Count - 1;
            hasRemaining = (heap->Count > 0);

            if (hasRemaining) {
                heap->Data[0] = heap->Data[lastIndex];
                minHeapSiftDown(heap, 0);
            }
        }
    }
}

/**
 * @brief 判断最小堆是否为空
 * @en_name minHeapIsEmpty
 * @cn_name 判断堆为空
 * @type widget
 * @param[IN] const MinHeap *heap 最小堆指针
 * @retval bool true堆为空或指针无效，false堆非空
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
bool minHeapIsEmpty(const MinHeap *heap)
{
    bool validHeap;
    bool result;

    validHeap = (heap != NULL);
    result = true;

    if (validHeap) {
        result = (heap->Count <= 0);
    }

    return result;
}

/**
 * @brief 释放最小堆占用的堆内存
 * @en_name minHeapFree
 * @cn_name 释放最小堆
 * @type widget
 * @param[IN] MinHeap *heap 最小堆指针
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void minHeapFree(MinHeap *heap)
{
    bool validHeap;
    bool hasData;

    validHeap = (heap != NULL);
    hasData = false;

    if (validHeap) {
        hasData = (heap->Data != NULL);

        if (hasData) {
            free(heap->Data);
        }

        heap->Data = NULL;
        heap->Count = 0;
        heap->Capacity = 0;
    }
}
