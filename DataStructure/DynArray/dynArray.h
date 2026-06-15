#ifndef DYNARRAY_H
#define DYNARRAY_H

#include <stdlib.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 通用动态数组结构体
 * @en_name DynArray
 * @cn_name 动态数组
 * @tag STRUCT
 * @field name=Data,     type=void*,  unit=1, desc=数据缓冲区指针
 * @field name=Count,    type=int,    unit=1, desc=当前元素数量
 * @field name=Capacity, type=int,    unit=1, desc=当前分配容量（元素个数）
 * @field name=ElemSize, type=int,    unit=B, desc=单个元素字节数
 * @version 1.0
 * @date 2026-03-27
 * @author aiden
 */
typedef struct {
    void *Data;
    int   Count;
    int   Capacity;
    int   ElemSize;
} DynArray;

void dynArrayInit(DynArray *arr, int elemSize, int initCapacity);
void dynArrayPush(DynArray *arr, const void *elem);
void dynArrayGet(const DynArray *arr, int index, void *out);
void dynArraySet(DynArray *arr, int index, const void *elem);
void dynArrayFree(DynArray *arr);
int  dynArraySize(const DynArray *arr);

#ifdef __cplusplus
}
#endif
#endif /* DYNARRAY_H */
