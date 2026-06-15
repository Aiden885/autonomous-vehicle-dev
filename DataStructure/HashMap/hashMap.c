#include "hashMap.h"
#include <stdlib.h>
#include <string.h>

/**
 * @brief 计算归一化哈希槽位
 * @en_name hashMapComputeSlot
 * @cn_name 计算哈希槽
 * @type widget
 * @param[IN] int key 键值
 * @param[IN] int capacity 哈希表容量
 * @retval int 归一化后的槽位索引
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static int hashMapComputeSlot(int key, int capacity)
{
    int slot;
    bool validCapacity;

    slot = 0;
    validCapacity = (capacity > 0);

    if (validCapacity) {
        slot = ((key % capacity) + capacity) % capacity;
    }

    return slot;
}

/**
 * @brief 判断输入整数是否为质数
 * @en_name hashMapIsPrime
 * @cn_name 判断质数
 * @type widget
 * @param[IN] int value 待判断整数
 * @retval bool true质数，false非质数
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static bool hashMapIsPrime(int value)
{
    bool result;
    bool smallValue;
    bool divisorFound;
    bool needCheck;
    bool divisible;
    int divisor;
    int step;
    int end;

    result = true;
    smallValue = (value < 2);

    if (smallValue) {
        result = false;
    } else {
        divisorFound = false;
        divisor = 2;
        step = 1;
        end = value;
        for (divisor; divisor < end; divisor = divisor + step) {
            needCheck = !divisorFound;
            if (needCheck) {
                divisible = ((value % divisor) == 0);
                if (divisible) {
                    divisorFound = true;
                }
            }
        }
        result = !divisorFound;
    }

    return result;
}

/**
 * @brief 选择不小于输入值的最小质数作为容量
 * @en_name hashMapSelectCapacity
 * @cn_name 选择质数容量
 * @type widget
 * @param[IN] int capacity 期望容量
 * @retval int 质数容量值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static int hashMapSelectCapacity(int capacity)
{
    int result;
    bool validCapacity;
    bool primeFound;
    bool searching;

    validCapacity = (capacity >= 2);

    if (validCapacity) {
        result = capacity;
    } else {
        result = 2;
    }

    primeFound = hashMapIsPrime(result);
    searching = !primeFound;
    while (searching) {
        result = result + 1;
        primeFound = hashMapIsPrime(result);
        searching = !primeFound;
    }

    return result;
}

/**
 * @brief 线性探测查找键对应槽位或可插入的空槽位
 * @en_name hashMapFindSlot
 * @cn_name 查找槽位
 * @type widget
 * @param[IN] const HashMap *map 哈希表指针
 * @param[IN] int key 键值
 * @param[OUT] int *slot 输出槽位索引，-1表示未找到
 * @param[OUT] bool *found 输出是否命中已存键
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
static void hashMapFindSlot(const HashMap *map, int key, int *slot, bool *found)
{
    bool slotValid;
    bool foundValid;
    bool outputValid;
    bool mapNotNull;
    bool dataNotNull;
    bool capPositive;
    bool mapValid;
    int baseSlot;
    bool slotResolved;
    bool needProbe;
    bool occupied;
    bool keyMatched;
    int probe;
    int step;
    int end;
    int currentSlot;

    slotValid = (slot != NULL);
    foundValid = (found != NULL);
    outputValid = (slotValid && foundValid);

    if (outputValid) {
        *slot = -1;
        *found = false;

        mapNotNull = (map != NULL);
        dataNotNull = false;
        capPositive = false;

        if (mapNotNull) {
            dataNotNull = (map->Data != NULL);
            capPositive = (map->Capacity > 0);
        }

        mapValid = (mapNotNull && dataNotNull && capPositive);

        if (mapValid) {
            baseSlot = hashMapComputeSlot(key, map->Capacity);
            slotResolved = false;
            probe = 0;
            step = 1;
            end = map->Capacity;

            for (probe; probe < end; probe = probe + step) {
                needProbe = !slotResolved;
                if (needProbe) {
                    currentSlot = (baseSlot + probe) % map->Capacity;
                    occupied = map->Data[currentSlot].Occupied;

                    if (occupied) {
                        keyMatched = (map->Data[currentSlot].Key == key);
                        if (keyMatched) {
                            *slot = currentSlot;
                            *found = true;
                            slotResolved = true;
                        }
                    } else {
                        *slot = currentSlot;
                        *found = false;
                        slotResolved = true;
                    }
                }
            }
        }
    }
}

/**
 * @brief 初始化哈希表，容量自动调整为质数
 * @en_name hashMapInit
 * @cn_name 初始化哈希表
 * @type widget
 * @param[IN] HashMap *map 哈希表指针
 * @param[IN] int capacity 期望容量（实际分配为≥capacity的最小质数）
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void hashMapInit(HashMap *map, int capacity)
{
    bool mapValid;
    int actualCapacity;
    bool capPositive;
    bool allocated;
    size_t allocationCount;

    mapValid = (map != NULL);

    if (mapValid) {
        memset(map, 0, sizeof(HashMap));
        actualCapacity = hashMapSelectCapacity(capacity);
        capPositive = (actualCapacity > 0);

        if (capPositive) {
            allocationCount = (size_t)actualCapacity;
            map->Data = (HashEntry *)calloc(allocationCount, sizeof(HashEntry));
            allocated = (map->Data != NULL);

            if (allocated) {
                map->Capacity = actualCapacity;
                map->Count = 0;
            }
        }
    }
}

/**
 * @brief 设置键值对，键已存在则更新，不存在则插入
 * @en_name hashMapSet
 * @cn_name 设置键值
 * @type widget
 * @param[IN] HashMap *map 哈希表指针
 * @param[IN] int key 键值
 * @param[IN] double value 数值
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void hashMapSet(HashMap *map, int key, double value)
{
    bool mapValid;
    bool hasSlot;
    bool shouldUpdate;
    int slot;
    bool found;

    mapValid = (map != NULL);
    slot = -1;
    found = false;

    if (mapValid) {
        hashMapFindSlot(map, key, &slot, &found);
        hasSlot = (slot >= 0);

        if (hasSlot) {
            shouldUpdate = found;
            if (shouldUpdate) {
                map->Data[slot].Value = value;
            } else {
                map->Data[slot].Key = key;
                map->Data[slot].Value = value;
                map->Data[slot].Occupied = true;
                map->Count = map->Count + 1;
            }
        }
    }
}

/**
 * @brief 获取哈希表中指定键的数值
 * @en_name hashMapGet
 * @cn_name 获取键值
 * @type widget
 * @param[IN] const HashMap *map 哈希表指针
 * @param[IN] int key 键值
 * @param[OUT] double *value 输出数值，未找到时不写入
 * @retval bool true找到，false未找到
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
bool hashMapGet(const HashMap *map, int key, double *value)
{
    bool result;
    bool canWrite;
    int slot;
    bool found;

    result = false;
    slot = -1;
    found = false;

    hashMapFindSlot(map, key, &slot, &found);
    result = found;
    canWrite = (found && (value != NULL));

    if (canWrite) {
        *value = map->Data[slot].Value;
    }

    return result;
}

/**
 * @brief 判断哈希表是否包含指定键
 * @en_name hashMapContains
 * @cn_name 判断是否包含键
 * @type widget
 * @param[IN] const HashMap *map 哈希表指针
 * @param[IN] int key 键值
 * @retval bool true包含，false不包含
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
bool hashMapContains(const HashMap *map, int key)
{
    bool result;
    int slot;
    bool found;

    result = false;
    slot = -1;
    found = false;

    hashMapFindSlot(map, key, &slot, &found);
    result = found;

    return result;
}

/**
 * @brief 释放哈希表占用的内存，所有字段归零
 * @en_name hashMapFree
 * @cn_name 释放哈希表
 * @type widget
 * @param[IN] HashMap *map 哈希表指针
 * @retval void 无返回值
 * @granularity atomic
 * @version 1.0
 * @date 2026-03-27
 * @author Codex/aiden
 */
void hashMapFree(HashMap *map)
{
    bool mapValid;
    bool dataValid;

    mapValid = (map != NULL);

    if (mapValid) {
        dataValid = (map->Data != NULL);
        if (dataValid) {
            free(map->Data);
        }
        memset(map, 0, sizeof(HashMap));
    }
}
