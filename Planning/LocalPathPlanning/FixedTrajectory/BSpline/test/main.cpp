/**
 * @brief 测试用例
 * @file main.cpp
 * @version 0.0.1
 * @author Chaoyi Chen (chency2023@tsinghua.edu.com)
 * @date 2023-11-25
 */

#include <iostream>
#include "../src/bSpline.hpp"

int main(int argc, const char * argv[]) {
  // insert code here...
  std::cout << "Hello, World!\n";
  
  std::vector<PlanningPoint> planningPoints;
  
  PlanningPoint SP = {0,0,0,
                      1,2,90,
                      0,0,0,
                      0,0,0,
                      0,0,0,0,0};
  PlanningPoint EP = {0,0,0,
                      20,10,0,
                      0,0,0,
                      0,0,0,
                      0,0,0,0,0};


  PlanningTrajectory PT = {planningPoints};

  BSplineParam BP;
  BSplineInput BI = {SP, EP};
  BSplineOutput BO = {PT};

  generateBsplinePath(BP, BI, BO);
     
  return 0;
}
