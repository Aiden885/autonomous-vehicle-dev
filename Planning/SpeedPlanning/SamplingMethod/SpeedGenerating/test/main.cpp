/**
 * @brief 测试用例
 * @file main.cpp
 * @version 0.0.1
 * @author Chaoyi Chen (chency2023@tsinghua.edu.com)
 * @date 2023-11-25
 */

#include <iostream>
#include "../src/initTrajectorySpeed.hpp"

int main(int argc, const char * argv[]) {
  // insert code here...
  std::cout << "Hello, World!\n";

  prediction::PredictPoint *predictPoint;
  prediction::Object *object;
  prediction::ObjectList objectList;
  object = objectList.add_object();

  for (int i = 0; i < 20; i++){   
    predictPoint = object->add_predictpoint();

    predictPoint->set_x(5);
    predictPoint->set_y(2);
    predictPoint->set_vx(0);
    predictPoint->set_vy(0);

    object->set_z(0);
    object->set_w(1);
    object->set_h(0);
    object->set_l(1);
    object->set_type(0);
    object->set_trackid(0);
  }

  std::vector<PlanningPoint> planningPoints;
  for (int i=0; i < 20; i++){
    PlanningPoint PP{(double)i,0,0,
                     0,0,0,
                     0,0,0,
                     1,0,0, 
                     0,0,0,0,0};
    planningPoints.push_back(PP);
  }

  PlanningTrajectory PT{planningPoints};

  TrajSpeedInitParam param{};

  TrajSpeedInitInput input{
    .trajectory = PT,
    .prediction = objectList
  };

  TrajSpeedInitOutput output{
    .globalTrajectory = PT
  };

  initSpeedForTrajectory(param, input, output);
  
  for (int i=0; i < 20; i++){
    std::cout << output.globalTrajectory.planningPoints.at(i).v << std::endl;
  }
  
  return 0;
}
