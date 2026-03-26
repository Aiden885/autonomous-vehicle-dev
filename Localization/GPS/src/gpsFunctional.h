/**
 * @brief GPS头文件
 * @file gpsFunctional.h
 * @version 0.0.1
 * @author Zihan Xie (770178863@qq.com)
 * @date 2023-11-27
 */
#include<iostream>
#include<QApplication>
#include "configure.h"
#include "mainwindow.h"
#include "GGA.h"
#include "gpfpd.h"
#include "GPYBM.h"
using namespace std;

/**
 * \namespace Localization
 * \brief Localization namespace
 */

namespace Localization{

struct paraAPP{};

struct inputAPP{
    int argc;
    char **argv;
};


struct outputAPP{};

struct paraGGA{};

struct inputGGA{string data;};

struct outputGGA{NMEAGGA data;};

struct paraGPYBM{};

struct inputGPYBM{string data;};

struct outputGPYBM{NMEAGPYBM data;};

struct paraGPFPD{};

struct inputGPFPD{string data;};

struct outputGPFPD{GPFPD data;};

struct calcCheckPara{};

struct calcCheckInput{string data;};

struct calcCheckOutput{long check;};

struct dmm2degPara{};

struct dmm2degInput{double dmm;};

struct dmm2degOutput{double data;};

int openQTWindow(int argc,char **argv);


void getGGA(const paraGGA &para,const inputGGA &input,outputGGA &output);

void getGPFPD(const paraGPFPD &para,const inputGPFPD &input,outputGPFPD &output);

void getGPYBM(const paraGPYBM &para,const inputGPYBM &input,outputGPYBM &output);


void calcCheckSum(calcCheckPara &para,calcCheckInput &input,calcCheckOutput &output);

void dmm2deg(dmm2degPara &para,dmm2degInput &input,dmm2degOutput &output);

}