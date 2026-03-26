/**
 * @brief DijkstraLattice功能头文件
 * @file dijLatticeFunctional.h
 * @version 0.0.1
 * @author Zihan Xie (770178863@qq.com)
 * @date 2023-12-19
 */
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "dijkstra_latticeMap.h"
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/types_c.h>
using namespace cv;
using namespace std;

/**
 * \namespace planning
 * \brief planning namespace
 */

namespace planning{

struct rmPara{};

struct rmInput{string filename;};

struct rmOutput{
    vector<vector<int>> map;
    vector<vector<int>> hvalue;};

struct  dijPara{};

struct dijInput
{
    Point1 start;
    Point1 end;
    vector<vector<int>> map;
    vector<vector<int>> hvalue;
};

struct dijOutput
{
    list<Point1 *> path;
};

struct sePara{};

struct seInput
{
    Mat img;
};

struct seOutput
{
    Point1 start;
    Point1 end;
};

struct pmPara{};

struct pmInput
{
    Mat *img;
    Point1 *start;
    Point1 *end;
    list<Point1 *> *path;
    pmInput()
    {
        img = nullptr;
        start = nullptr;
        end = nullptr;
        path = nullptr;
    }
};

struct pmOutput{Mat img;};

struct InitDijPara{};

struct InitDijInput
{
    vector<std::vector<int>> maze;
    vector<std::vector<int>> hvalue;
};

struct InitDijOutput{Astar A;};

//void InitAstar(const std::vector<std::vector<int>>& _maze, const std::vector<std::vector<int>>& _hvalue);

void on_mouse(int event, int x, int y, int flags, void* ustc);

void DijkstraReadMap(const rmPara &para,const rmInput &input,rmOutput &output);

void DijkstraGetStartEnd(const sePara &para,const seInput &input,seOutput &output);

void DijkstraGetPathDijMap(const dijPara &para,const dijInput &input,dijOutput &output);

void DijkstraPrintMap(const pmPara &para,const pmInput &input,pmOutput &output);

void DijkstraInitDij(InitDijPara &para,InitDijInput &input,InitDijOutput &output);

}