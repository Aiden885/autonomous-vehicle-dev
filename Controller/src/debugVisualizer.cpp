#include "debugVisualizer.hpp"
#include <deque>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace Control {

// 静态历史转向队列，跨帧保存
static std::deque<double> steerHist;

void debugVisualizer(
    bool debugFlag,
    const State& s,
    const Traj& traj,
    const std::vector<double>& kappaList,
    int previewIdx,
    double previewDist,
    double k,
    int NearestIndex,
    double NearestIdxLatDis,
    double NearestIdxDis2IMU,
    double pureSteerDeg,
    double feedbackSteerDeg,
    double finalSteerDeg,
    double dT
) {
    if (!debugFlag) return;

    cv::Mat img(1000, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    // ---------------- 绘制转向历史 ----------------
    {
        int panelW = img.cols * 0.72;
        int panelH = img.rows * 0.42;
        cv::Rect roiTL(0, 0, panelW, panelH);
        cv::Mat panel = img(roiTL);
        panel.setTo(cv::Scalar(255,255,255));

        const int MAX_SAMPLES = 250;
        double curSteer = std::isfinite(finalSteerDeg) ? finalSteerDeg : 0.0;
        steerHist.push_back(curSteer);
        if ((int)steerHist.size() > MAX_SAMPLES) steerHist.pop_front();

        double maxAbs = 1.0;
        for (double v : steerHist) maxAbs = std::max(maxAbs, std::abs(v));
        maxAbs = std::max(1.0, maxAbs*1.1);

        int L=60,R=10,T=25,B=30;
        int plotW = panelW-(L+R);
        int plotH = panelH-(T+B);
        cv::Point origin(L,T+plotH);

        cv::line(panel, origin, origin+cv::Point(plotW,0), cv::Scalar(0,0,0),1);
        cv::line(panel, origin, origin-cv::Point(0,plotH), cv::Scalar(0,0,0),1);

        auto yToPix = [&](double v)->int{
            return (int)std::lround(origin.y-(v-(-maxAbs))*(plotH/(2*maxAbs)));
        };

        int y0 = yToPix(0.0);
        cv::line(panel, cv::Point(L,y0), cv::Point(L+plotW,y0), cv::Scalar(200,200,200),1);

        std::vector<double> yticks={-maxAbs,-maxAbs*0.5,0.0,maxAbs*0.5,maxAbs};
        for(double vy:yticks){
            int py=yToPix(vy);
            cv::line(panel,cv::Point(L-4,py),cv::Point(L,py),cv::Scalar(0,0,0),1);
            char buf[32]; std::snprintf(buf,sizeof(buf),"%.1f",vy);
            cv::putText(panel,buf,cv::Point(5,py+4),
                        cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(0,0,0),1);
        }

        int numTicks = 14;
        double totalTime = MAX_SAMPLES*dT;
        double tickStep = totalTime/(numTicks-1);
        for(int i=0;i<numTicks;++i){
            double t=i*tickStep;
            int px=L+static_cast<int>((i/(double)(numTicks-1))*plotW);
            cv::line(panel, cv::Point(px, origin.y), cv::Point(px, origin.y+4), cv::Scalar(0,0,0), 1);
            char buf[16]; std::snprintf(buf,sizeof(buf),"%.1f",t);
            cv::putText(panel, buf, cv::Point(px-10, origin.y+18),
                        cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(0,0,0),1);
        }
        cv::putText(panel,"Time(s)", cv::Point(L+plotW/2-30,origin.y+28),
                    cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,0),1);

        int N=(int)steerHist.size();
        if(N>=2){
            std::vector<cv::Point> poly(N);
            double step=(double)plotW/(MAX_SAMPLES-1);
            for(int i=0;i<N;++i){
                int px=L+(int)std::round(i*step);
                int py=yToPix(steerHist[i]);
                poly[i]=cv::Point(px,py);
            }
            cv::polylines(panel, poly, false, cv::Scalar(0,0,255),2);
        }

        cv::putText(panel,"finalSteerDeg",cv::Point(L,T-8),
                    cv::FONT_HERSHEY_SIMPLEX,0.6,cv::Scalar(0,0,0),2);
        cv::putText(panel,cv::format(": %.2f",curSteer),
                    cv::Point(L+130,T-8),cv::FONT_HERSHEY_SIMPLEX,0.65,cv::Scalar(0,0,0),2);
    }

    // ---------------- 绘制轨迹 ----------------
    double centerX=s.x;
    double centerY=s.y;
    double scale=22;
    cv::Point imgCenter(img.cols/2, img.rows*3/5);
    double alpha_rad=(M_PI/180.0)*(180-s.yaw);
    double c=std::cos(-alpha_rad);
    double ss=std::sin(-alpha_rad);

    auto worldToImg=[&](double x,double y)->cv::Point{
        double dx=x-centerX;
        double dy=y-centerY;
        double x_rot=dx*c-dy*ss;
        double y_rot=dx*ss+dy*c;
        int col=(int)(imgCenter.x+y_rot*scale);
        int row=(int)(imgCenter.y-x_rot*scale);
        return cv::Point(col,row);
    };

    for(size_t i=0;i<traj.size();++i){
        cv::Point pt=worldToImg(traj[i].x,traj[i].y);
        cv::circle(img,pt,1.8,cv::Scalar(100,100,100),-1);
    }

    cv::Point pt_trajP=worldToImg(traj[previewIdx].x,traj[previewIdx].y);
    cv::Scalar color=cv::Scalar(0,0,255);
    cv::circle(img, pt_trajP, 9, color,-1);
    cv::putText(img,"P",pt_trajP+cv::Point(10,-10),
                cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,0),2.0);

    cv::Point pt_traj=worldToImg(traj[NearestIndex].x,traj[NearestIndex].y);
    cv::circle(img,pt_traj,8,cv::Scalar(0,165,255),-1);

    cv::Point pt_vehicle=worldToImg(s.x,s.y);
    cv::circle(img,pt_vehicle,8,cv::Scalar(255,0,0),-1);
    cv::putText(img,"V",pt_vehicle+cv::Point(10,-10),
                cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(255,0,0),2.0);

    cv::line(img,pt_vehicle,pt_trajP,cv::Scalar(255,0,255),1.0);


    // ---------------- 绘制文字信息 ----------------
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.58;
    int thickness = 1;

    std::vector<std::string> texts = {
        cv::format("prevIdx: %d", previewIdx),
        cv::format("prevDist(m): %.2f", previewDist),
        cv::format("prevKappA: %.6f", k),
        cv::format("NearIdx: %d", NearestIndex),
        cv::format("NearIdxLatDis(m): %.4f", NearestIdxLatDis),
        cv::format("NearIdxDis2IMU(m): %.4f", NearestIdxDis2IMU),
        cv::format("pureSteerDeg: %.2f", pureSteerDeg),
        cv::format("fbSteerDeg: %.2f", feedbackSteerDeg),
        cv::format("NearKappA: %.6f", kappaList[NearestIndex])
    };

    // ===== 核心修改：基于画布宽度动态计算起始位置 =====
    int lineStep = 25;
    int textBaseY = 45;

    // 画布右侧 3/4 处
    int textBaseX = static_cast<int>(img.cols * 0.75);

    // 预留右侧安全边距，防止文字越界
    int rightMargin = 300;
    textBaseX = std::min(textBaseX, img.cols - rightMargin);

    cv::Point pos(textBaseX, textBaseY);

    for (size_t i = 0; i < texts.size(); ++i) {
        cv::putText(
            img,
            texts[i],
            pos + cv::Point(0, i * lineStep),
            fontFace,
            fontScale,
            cv::Scalar(0, 0, 0),
            thickness
        );
    }


    // ---------------- 绘制曲率图 ----------------
    const int MAXN=30;
    int N=std::min<int>(MAXN,std::min<int>((int)traj.size(),(int)kappaList.size()));
    if(N>1){
        int chartH=img.rows/4;
        int chartY=img.rows-chartH;
        int chartX=0;
        int chartW=img.cols;
        cv::Rect roi(chartX,chartY,chartW,chartH);
        cv::Mat panel=img(roi);
        panel.setTo(cv::Scalar(250,250,250));

        std::vector<double> y_orig(N),y_smooth(N);
        for(int i=0;i<N;++i){ y_orig[i]=traj[i].k; y_smooth[i]=kappaList[i]; }

        auto mm1=std::minmax_element(y_orig.begin(),y_orig.end());
        auto mm2=std::minmax_element(y_smooth.begin(),y_smooth.end());
        double ymin=std::min(*mm1.first,*mm2.first);
        double ymax=std::max(*mm1.second,*mm2.second);
        if(!std::isfinite(ymin)||!std::isfinite(ymax)) return;
        if(std::abs(ymax-ymin)<1e-9){ymax+=1.0;ymin-=1.0;}
        double pad=0.1*(ymax-ymin); ymin-=pad;ymax+=pad;

        int L=55,R=10,T=15,B=35;
        int plotW=std::max(20,chartW-(L+R));
        int plotH=std::max(20,chartH-(T+B));
        cv::Point origin(L,T+plotH);

        cv::line(panel,origin,origin+cv::Point(plotW,0),cv::Scalar(0,0,0),1,cv::LINE_AA);
        cv::line(panel,origin,origin-cv::Point(0,plotH),cv::Scalar(0,0,0),1,cv::LINE_AA);

        auto mapPoint=[&](int i_disp,double v)->cv::Point{
            int px=L+i_disp*(plotW/(N-1));
            double yy=origin.y-(v-ymin)*(plotH/(ymax-ymin));
            return cv::Point(px,(int)std::lround(yy));
        };

        std::vector<cv::Point> polyOrig(N),polySmooth(N);
        for(int i=0;i<N;++i){ polyOrig[i]=mapPoint(i,y_orig[i]); polySmooth[i]=mapPoint(i,y_smooth[i]); }

        cv::polylines(panel,polyOrig,false,cv::Scalar(0,255,0),2,cv::LINE_AA);
        cv::polylines(panel,polySmooth,false,cv::Scalar(0,0,255),2,cv::LINE_AA);
        for(int i=0;i<N;++i){
            cv::circle(panel,polyOrig[i],3,cv::Scalar(0,120,0),cv::FILLED,cv::LINE_AA);
            cv::circle(panel,polySmooth[i],3,cv::Scalar(0,0,150),cv::FILLED,cv::LINE_AA);
        }

        cv::rectangle(panel,cv::Rect(L+5,T+5,12,12),cv::Scalar(0,255,0),cv::FILLED);
        cv::putText(panel,"origTraj.K",cv::Point(L+22,T+16),cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(0,0,0),1.2);
        cv::rectangle(panel,cv::Rect(L+120,T+5,12,12),cv::Scalar(0,0,255),cv::FILLED);
        cv::putText(panel,"smoothed.K",cv::Point(L+137,T+16),cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(0,0,0),1.2);
    }

    cv::imshow("Trajectory Debug",img);
    cv::waitKey(1);
}

} // namespace Control
