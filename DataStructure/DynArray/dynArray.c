#include "dynArray.h"
#include <string.h>

/**
 * @brief 计算动态数组指定索引元素的地址
 * @en_name dynArrayGetElemAddr
 * @cn_name 获取元素地址
 * @type widget
 * @param[IN] const DynArray *arr 动态数组对象
 * @param[IN] int index 元素索引
 * @retval void* 指定索引对应的元素地址
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static void *dynArrayGetElemAddr(const DynArray *arr, int index)
{
    void *addr;
    char *basePtr;
    bool condValid;

    addr = NULL;
    basePtr = NULL;
    condValid = (arr != NULL);

    if (condValid) {
        basePtr = (char *)arr->Data;
        addr = (void *)(basePtr + ((size_t)index * (size_t)arr->ElemSize));
    }

    return addr;
}

/**
 * @brief 为动态数组扩容到指定容量
 * @en_name dynArrayResize
 * @cn_name 动态数组扩容
 * @type widget
 * @param[IN] DynArray *arr 动态数组对象
 * @param[IN] int newCapacity 新容量（元素个数）
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static void dynArrayResize(DynArray *arr, int newCapacity)
{
    bool condArrValid;
    bool condCapValid;
    void *newData;
    size_t allocSize;

    condArrValid = (arr != NULL);
    condCapValid = false;
    newData = NULL;
    allocSize = 0U;

    if (condArrValid) {
        condCapValid = (newCapacity > 0);
        if (condCapValid) {
            allocSize = (size_t)newCapacity * (size_t)arr->ElemSize;
            newData = realloc(arr->Data, allocSize);
            if (newData != NULL) {
                arr->Data = newData;
                arr->Capacity = newCapacity;
            }
        }
    }
}

/**
 * @brief 初始化动态数组，分配初始内存
 * @en_name dynArrayInit
 * @cn_name 初始化动态数组
 * @type widget
 * @param[IN] DynArray *arr 动态数组对象
 * @param[IN] int elemSize 单个元素字节大小
 * @param[IN] int initCapacity 初始容量（元素个数）
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void dynArrayInit(DynArray *arr, int elemSize, int initCapacity)
{
    bool condArrValid;
    bool condElemSizeValid;
    bool condCapValid;
    size_t allocSize;

    condArrValid = (arr != NULL);
    condElemSizeValid = false;
    condCapValid = false;
    allocSize = 0U;

    if (condArrValid) {
        arr->Data = NULL;
        arr->Count = 0;
        arr->Capacity = 0;
        arr->ElemSize = 0;

        condElemSizeValid = (elemSize > 0);
        if (condElemSizeValid) {
            arr->ElemSize = elemSize;
            condCapValid = (initCapacity > 0);
            if (condCapValid) {
                allocSize = (size_t)elemSize * (size_t)initCapacity;
                arr->Data = malloc(allocSize);
                if (arr->Data != NULL) {
                    arr->Capacity = initCapacity;
                }
            }
        }
    }
}

/**
 * @brief 向动态数组尾部追加一个元素，容量不足时自动扩容2倍
 * @en_name dynArrayPush
 * @cn_name 动态数组尾插
 * @type widget
 * @param[IN] DynArray *arr 动态数组对象
 * @param[IN] const void *elem 待插入元素地址
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void dynArrayPush(DynArray *arr, const void *elem)
{
    bool condArrValid;
    bool condElemValid;
    bool condBothValid;
    bool condNeedInit;
    bool condNeedGrow;
    bool condCanWrite;
    int targetCapacity;
    void *dest;

    condArrValid = (arr != NULL);
    condElemValid = (elem != NULL);
    condBothValid = (condArrValid && condElemValid);
    condNeedInit = false;
    condNeedGrow = false;
    condCanWrite = false;
    targetCapacity = 0;
    dest = NULL;

    if (condBothValid) {
        condNeedInit = (arr->Capacity <= 0);
        if (condNeedInit) {
            targetCapacity = 1;
            dynArrayResize(arr, targetCapacity);
        }

        condNeedGrow = (arr->Count >= arr->Capacity);
        if (condNeedGrow) {
            targetCapacity = arr->Capacity * 2;
            dynArrayResize(arr, targetCapacity);
        }

        condCanWrite = (arr->Count < arr->Capacity);
        if (condCanWrite) {
            dest = dynArrayGetElemAddr(arr, arr->Count);
            memcpy(dest, elem, (size_t)arr->ElemSize);
            arr->Count = arr->Count + 1;
        }
    }
}

/**
 * @brief 读取动态数组指定索引的元素到out
 * @en_name dynArrayGet
 * @cn_name 获取动态数组元素
 * @type widget
 * @param[IN] const DynArray *arr 动态数组对象
 * @param[IN] int index 元素索引
 * @param[OUT] void *out 输出缓冲区，调用方保证空间足够
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void dynArrayGet(const DynArray *arr, int index, void *out)
{
    bool condArrValid;
    bool condOutValid;
    bool condBothValid;
    bool condIdxNotNeg;
    bool condIdxInRange;
    bool condIndexValid;
    void *src;

    condArrValid = (arr != NULL);
    condOutValid = (out != NULL);
    condBothValid = (condArrValid && condOutValid);
    condIdxNotNeg = false;
    condIdxInRange = false;
    condIndexValid = false;
    src = NULL;

    if (condBothValid) {
        condIdxNotNeg = (index >= 0);
        condIdxInRange = (index < arr->Count);
        condIndexValid = (condIdxNotNeg && condIdxInRange);
        if (condIndexValid) {
            src = dynArrayGetElemAddr(arr, index);
            memcpy(out, src, (size_t)arr->ElemSize);
        }
    }
}

/**
 * @brief 将elem写入动态数组指定索引位置
 * @en_name dynArraySet
 * @cn_name 设置动态数组元素
 * @type widget
 * @param[IN] DynArray *arr 动态数组对象
 * @param[IN] int index 元素索引
 * @param[IN] const void *elem 待写入元素地址
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void dynArraySet(DynArray *arr, int index, const void *elem)
{
    bool condArrValid;
    bool condElemValid;
    bool condBothValid;
    bool condIdxNotNeg;
    bool condIdxInRange;
    bool condIndexValid;
    void *dest;

    condArrValid = (arr != NULL);
    condElemValid = (elem != NULL);
    condBothValid = (condArrValid && condElemValid);
    condIdxNotNeg = false;
    condIdxInRange = false;
    condIndexValid = false;
    dest = NULL;

    if (condBothValid) {
        condIdxNotNeg = (index >= 0);
        condIdxInRange = (index < arr->Count);
        condIndexValid = (condIdxNotNeg && condIdxInRange);
        if (condIndexValid) {
            dest = dynArrayGetElemAddr(arr, index);
            memcpy(dest, elem, (size_t)arr->ElemSize);
        }
    }
}

/**
 * @brief 释放动态数组占用的堆内存，将所有字段归零
 * @en_name dynArrayFree
 * @cn_name 释放动态数组
 * @type widget
 * @param[IN] DynArray *arr 动态数组对象
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void dynArrayFree(DynArray *arr)
{
    bool condValid;

    condValid = (arr != NULL);

    if (condValid) {
        free(arr->Data);
        arr->Data = NULL;
        arr->Count = 0;
        arr->Capacity = 0;
        arr->ElemSize = 0;
    }
}

/**
 * @brief 返回动态数组当前元素数量
 * @en_name dynArraySize
 * @cn_name 获取动态数组大小
 * @type widget
 * @param[IN] const DynArray *arr 动态数组对象
 * @retval int 当前元素数量，arr为NULL时返回0
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
int dynArraySize(const DynArray *arr)
{
    bool condValid;
    int sizeValue;

    condValid = (arr != NULL);
    sizeValue = 0;

    if (condValid) {
        sizeValue = arr->Count;
    }

    return sizeValue;
}
