#include "judgeDirectionOfMebTypes.h"

/**
 * @brief 根据 MEB 碰撞超声 ID 判断需要限制的行驶方向
 * @en_name judgeDirectionOfMeb
 * @cn_name MEB碰撞方向判断
 * @type module
 * @param[IN] Integer MebCollisionUsId MEB碰撞超声ID
 * @param[OUT] Integer Dir 方向:1=forward,-1=backward,0=none
 * @granularity atomic
 * @tag_level1 泊车规划
 * @tag_level2 碰撞检测
 * @source matlab_rewrite judge_direction_of_MEB.m
 * @version 1.0
 * @date 2026-04-30
 * @author aide
 */
void judgeDirectionOfMeb(const JudgeDirectionOfMebInput_t *input,
                         JudgeDirectionOfMebOutput_t *output) {
  Integer usId;
  Integer dir;
  Boolean condFront;
  Boolean condRear;

  usId = input->MebCollisionUsId;
  dir = 0;
  condFront = (usId > 0) && (usId < 5);
  condRear = (usId > 4) && (usId < 9);

  if (condFront) {
    dir = 1;
  } else if (condRear) {
    dir = -1;
  }

  output->Dir = dir;
}
