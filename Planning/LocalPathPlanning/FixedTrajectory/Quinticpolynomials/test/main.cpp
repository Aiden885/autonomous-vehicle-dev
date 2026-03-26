/**
 * @brief 测试用例
 * @file main.cpp
 * @version 0.0.1
 * @author Chaoyi Chen (chency2023@tsinghua.edu.com)
 * @date 2023-11-25
 */

#include <iostream>
#include <fstream>
#include "../src/quinticpolynomials.h"

int main(int argc, const char * argv[]) {
  // insert code here...
  std::cout << "Hello, World!\n";


  Curve path;
  for (int i = 0; i < CURVE_POINT_NUM; ++i)
  {
    path.points[i].x = 0;
    path.points[i].y = 0;
    path.points[i].angle = 0;
  }

  GaussRoadPoint SP = {0,0,0,0};
  GaussRoadPoint EP = {3,8,0,0};

  GQPPParam GP = {};
  GQPPInput GI = {SP, EP, path};
  GQPPOutput GO = {path};

  generateQuinticPolynomialsPath(GP, GI, GO);

  path = GO.path;

  std::ofstream fs0("quinticpolynomials.txt", std::ios::out);
  for (int i = 0; i < CURVE_POINT_NUM; ++i)
  {
    fs0 << path.points[i].x << std::endl;
  }
  
  for (int i = 0; i < CURVE_POINT_NUM; ++i)
  {
    fs0 << path.points[i].y << std::endl;
  }
        
  fs0.close();
  
  return 0;
}
