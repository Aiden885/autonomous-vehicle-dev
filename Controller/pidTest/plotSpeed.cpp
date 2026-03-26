#include "plotSpeed.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>

static cv::Mat canvas;
static std::vector<double> vHist;
static std::vector<double> refHist;
static std::vector<double> timeHist;

static int WIDTH = 1000;
static int HEIGHT = 600;
static int margin = 60;
static double vmax = 20;   // 最大速度，可根据需求调整

void speedPlotInit()
{
    canvas = cv::Mat(HEIGHT, WIDTH, CV_8UC3);
    cv::namedWindow("SpeedPlot", cv::WINDOW_AUTOSIZE);
    canvas.setTo(cv::Scalar(30,30,30));
}

// 保存数据到CSV文件，可选
void speedPlotSaveData(const std::string& filename)
{
    std::ofstream fout(filename);
    if(!fout.is_open()) return;
    fout << "time,v,v_ref\n";
    for(size_t i=0;i<vHist.size();i++)
        fout << timeHist[i] << "," << vHist[i] << "," << refHist[i] << "\n";
    fout.close();
}

void speedPlotUpdate(double time, double v, double v_ref)
{
    // 保存数据
    timeHist.push_back(time);
    vHist.push_back(v);
    refHist.push_back(v_ref);

    // 清空画布
    canvas.setTo(cv::Scalar(30,30,30));

    int plotW = WIDTH - 2*margin;
    int plotH = HEIGHT - 2*margin;
    int n = vHist.size();

    // x轴缩放，按最多500个点显示
    int displayPoints = std::min(n, 500);

    for(int i = 1; i < displayPoints; i++)
    {
        int idx1 = n - displayPoints + i - 1;
        int idx2 = n - displayPoints + i;

        int x1 = margin + (i-1) * plotW / 500;
        int x2 = margin + i * plotW / 500;

        int y1 = HEIGHT - margin - vHist[idx1]/vmax * plotH;
        int y2 = HEIGHT - margin - vHist[idx2]/vmax * plotH;
        cv::line(canvas, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(0,255,0),2);

        int ry1 = HEIGHT - margin - refHist[idx1]/vmax * plotH;
        int ry2 = HEIGHT - margin - refHist[idx2]/vmax * plotH;
        cv::line(canvas, cv::Point(x1,ry1), cv::Point(x2,ry2), cv::Scalar(0,0,255),2);
    }

    // 坐标轴
    cv::line(canvas, cv::Point(margin,HEIGHT-margin), cv::Point(WIDTH-margin,HEIGHT-margin), cv::Scalar(255,255,255),2);
    cv::line(canvas, cv::Point(margin,margin), cv::Point(margin,HEIGHT-margin), cv::Scalar(255,255,255),2);

    // 标题和图例
    cv::putText(canvas,"Velocity Tracking", cv::Point(380,40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255),2);
    cv::putText(canvas,"Green: v", cv::Point(60,50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0),2);
    cv::putText(canvas,"Red: v_ref", cv::Point(200,50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255),2);

    // 显示画布
    cv::imshow("SpeedPlot", canvas);
    cv::waitKey(1); // 1ms延迟刷新，不阻塞
}

void speedPlotClose()
{
    cv::destroyAllWindows();
}