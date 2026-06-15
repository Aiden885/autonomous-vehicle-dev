#ifndef HASHMAP_H
#define HASHMAP_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 哈希表条目结构体（开放寻址法）
 * @en_name HashEntry
 * @cn_name 哈希条目
 * @tag STRUCT
 * @field name=Key,      type=int,    unit=1, desc=整数键
 * @field name=Value,    type=double, unit=1, desc=浮点值
 * @field name=Occupied, type=bool,   unit=1, desc=该槽位是否已占用
 * @version 1.0
 * @date 2026-03-27
 * @author aiden
 */
typedef struct {
    int    Key;
    double Value;
    bool   Occupied;
} HashEntry;

/**
 * @brief 哈希表结构体，用于A*算法存储g值和closed列表
 * @en_name HashMap
 * @cn_name 哈希表
 * @tag STRUCT
 * @field name=Data,     type=Array<HashEntry,[N]>, unit=1, desc=条目数组
 * @field name=Capacity, type=int,                  unit=1, desc=桶数量
 * @field name=Count,    type=int,                  unit=1, desc=已存条目数
 * @version 1.0
 * @date 2026-03-27
 * @author aiden
 */
typedef struct {
    HashEntry *Data;
    int        Capacity;
    int        Count;
} HashMap;

void hashMapInit(HashMap *map, int capacity);
void hashMapSet(HashMap *map, int key, double value);
bool hashMapGet(const HashMap *map, int key, double *value);
bool hashMapContains(const HashMap *map, int key);
void hashMapFree(HashMap *map);

#ifdef __cplusplus
}
#endif
#endif /* HASHMAP_H */
