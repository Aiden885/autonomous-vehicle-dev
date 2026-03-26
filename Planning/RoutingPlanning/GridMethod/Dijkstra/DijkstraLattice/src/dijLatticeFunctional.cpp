/**
 * @brief DijkstraLattice功能源文件
 * @file dijLatticeFunctional.cpp
 * @version 0.0.1
 * @author Zihan Xie (770178863@qq.com)
 * @date 2023-12-19
 */
#include"dijLatticeFunctional.h"
using namespace cv;
using namespace std;

namespace planning{

void on_mouse(int event, int x, int y, int flags, void* ustc)
{
	if (event == EVENT_LBUTTONDOWN)    
	{
		static_cast<Point1*>(ustc)->x = x;   
        static_cast<Point1*>(ustc)->y = y;     
	}
}
/**
 * @brief Dijkstra读取地图图片
 * @param[IN] rmPara 无效占位
 * @param[IN] rmInput 文件路径
 * @param[IN] rmOutput 地图数组、halue数组
 
 * @cn_name: Dijkstra读取地图图片
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraReadMap(const rmPara &para,const rmInput &input,rmOutput &output)
{
    Mat img1 = imread("./test/map5.jpg");
    if (img1.empty())
	{
		cout << "could not load image...\n" << endl;
		return;
	}
    Mat gray_img;
	cvtColor(img1, gray_img, CV_BGR2GRAY);
	int height = gray_img.rows;
	int width = gray_img.cols;
    for (int row = 0; row < height; row++)
	{
		
		for (int col = 0; col < width; col++)
		{
			if (gray_img.at<uchar>(row, col) > 0 || gray_img.at<uchar>(row, col) == 255)
			{
				gray_img.at<uchar>(row, col) = 1;
			}
			else if (gray_img.at<uchar>(row, col) == 0)
			{
				gray_img.at<uchar>(row, col) = 0;
			}
		}
	}
		
	vector<vector<int>> map(height, vector<int>(width,0));
	vector<vector<int>> hvalue(height, vector<int>(width, 0));
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			map[i][j] = gray_img.at<uchar>(i, j);
			//cout << map[i][j];
			hvalue[i][j] = 0;
		}
	}
    output.map = map;
    output.hvalue = hvalue;
}

/**
 * @brief Dijkstra在地图上获取起始点和终点
 * @param[IN] sePara 无效占位
 * @param[IN] seInput Mat对象
 * @param[IN] seOutput 起点和终点
 
 * @cn_name: Dijkstra在地图上获取起始点和终点
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraGetStartEnd(const sePara &para,const seInput &input,seOutput &output)
{
    namedWindow("Choose start");   
	setMouseCallback("Choose start", on_mouse,&(output.start));	
    imshow("Choose start", input.img);
	waitKey(0);
	destroyAllWindows();

    namedWindow("Choose end");   
	setMouseCallback("Choose end", on_mouse,&(output.end));	
    imshow("Choose end", input.img);
	waitKey(0);
	destroyAllWindows();
}
/**
 * @brief Dijkstra求最短路径
 * @param[IN] dijPara 无效占位
 * @param[IN] dijInput 起点、终点、地图数组、hvalue数组
 * @param[IN] dijOutput 路径列表
 
 * @cn_name: Dijkstra求最短路径
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraGetPathDijMap(const dijPara &para,const dijInput &input,dijOutput &output)
{
    Astar astar;
	astar.InitAstar(input.map,input.hvalue);
    Point1 start(input.start.x,input.start.y);
    Point1 end(input.end.x,input.end.y);
    output.path = astar.GetPath(start, end, false);
}
/**
 * @brief Dijkstra打印地图图片
 * @param[IN] pmPara 无效占位
 * @param[IN] pmInput Map对象指针、起点指针、终点指针、路径列表指针
 * @param[IN] pmOutput Map对象
 
 * @cn_name: Dijkstra打印地图图片
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraPrintMap(const pmPara &para,const pmInput &input,pmOutput &output)
{
    output.img = *(input.img);
    if(input.img)
    {
        if(input.start)
            circle(output.img, Point(input.start->x, input.start->y), 3, Scalar(0, 255, 0), -1);
        if(input.end)
            circle(output.img, Point(input.end->x, input.end->y), 3, Scalar(0, 0, 255), -1);
        if(input.path)
        {
            for (auto& p : *(input.path)) 
            {
                std::cout << "(" << p->x << "," << p->y << ") ";
                line(output.img, Point(p->x, p->y), Point(p->x, p->y), Scalar(255, 0, 0), 1);
            }
        }
        namedWindow("img");
        imshow("img",output.img);
        waitKey(0);
	    destroyAllWindows();
    }
}
/**
 * @brief 初始化Dijkstra
 * @param[IN] InitDijPara 无效占位
 * @param[IN] InitDijInput 地图数组、hvalue数组
 * @param[IN] InitDijOutput Astar对象
 
 * @cn_name: 初始化Dijkstra
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: planning
 */
void DijkstraInitDij(InitDijPara &para,InitDijInput &input,InitDijOutput &output)
{
	output.A.InitAstar(input.maze,input.hvalue);
}

}