#ifndef JUDGE_DIRECTION_OF_MEB_TYPES_H
#define JUDGE_DIRECTION_OF_MEB_TYPES_H

#include "baseTypes.h"

/**
 * @brief MEB 碰撞超声 ID 到方向的映射输入
 * @en_name JudgeDirectionOfMebInput
 * @cn_name MEB碰撞方向判断输入
 * @tag STRUCT
 * @field name=MebCollisionUsId, type=Integer, unit=1, desc=MEB碰撞超声ID:1-4前向,5-8后向
 * @version 1.0
 * @date 2026-04-30
 * @author aide
 */
typedef struct {
  Integer MebCollisionUsId;
} JudgeDirectionOfMebInput_t;

/**
 * @brief MEB 碰撞方向判断输出
 * @en_name JudgeDirectionOfMebOutput
 * @cn_name MEB碰撞方向判断输出
 * @tag STRUCT
 * @field name=Dir, type=Integer, unit=1, desc=方向:1=forward,-1=backward,0=none
 * @version 1.0
 * @date 2026-04-30
 * @author aide
 */
typedef struct {
  Integer Dir;
} JudgeDirectionOfMebOutput_t;

void judgeDirectionOfMeb(const JudgeDirectionOfMebInput_t *input,
                         JudgeDirectionOfMebOutput_t *output);

#endif
