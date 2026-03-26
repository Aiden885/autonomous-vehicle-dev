/**
 * @brief 测试用例
 * @file main.cpp
 * @version 0.0.1
 * @author Chaoyi Chen (chency2023@tsinghua.edu.com)
 * @date 2023-11-25
 */

#include <iostream>
#include "../src/STgraphOpt.hpp"

int main(int argc, const char * argv[]) {
  // insert code here...
  std::cout << "Hello, World!\n";
  
  std::vector<PlanningPoint> planningPoints;
  for (int i=0; i < 20; i++){
    PlanningPoint PP{0,0,0,
                     0,0,0,
                     0,0,0,
                     0.1*((double)i*i),0,0, 
                     0,0,0,0,0};
    planningPoints.push_back(PP);
  }

  PlanningTrajectory PT{planningPoints};

  STgraphOptParam param{};

  STgraphOptInput input{PT};

  STgraphOptOutput output{PT};

  STOptimization(param, input, output);

  
  for (int i=0; i < 20; i++){
    std::cout << output.trajectory.planningPoints.at(i).v << std::endl;
  }
  
     
  return 0;
}
