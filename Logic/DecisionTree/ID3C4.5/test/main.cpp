
#include <iostream>
#include "../src/ID3C45Functional.h"
#include "../dependence/LogicDecisionTreeHead.hpp"
using namespace std;
#include <map>
#include <vector>
#include <cmath>



int main() {

    DecisionTree DT;
    vector<vector<int>> trainData = {
        {0, 0, 0, 0},
        {0, 0, 0, 1},
        {0, 1, 0, 1},
        {0, 1, 1, 0},
        {0, 0, 0, 0},
        {1, 0, 0, 0},
        {1, 0, 0, 1},
        {1, 1, 1, 1},
        {1, 0, 1, 2},
        {1, 0, 1, 2},
        {2, 0, 1, 2},
        {2, 0, 1, 1},
        {2, 1, 0, 1},
        {2, 1, 0, 2},
        {2, 0, 0, 0}
    };
    vector<int> trainLabel = {0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0};
    vector<int> dataset = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14};
        //训练数据
    vector<vector<int>> verifyData = {
        {0, 1, 1, 1},
        {1, 1, 0, 2},
        {1, 1, 1, 2},
        {2, 0, 1, 0},
        {2, 1, 1, 1},
        {2, 0, 0, 2}
    };
    //训练标签
    vector<int> verifyLabel = {0, 0, 1, 1, 0, 0};

    ofstream out; 
    out.open("dt.txt", ios::out);
    out.close();
    out.open("dt1.txt", ios::out);
    out.close();

    DecisionTree dt(trainData,trainLabel,false);
    
    dt.saveTree(dt.decisionTreeRoot, "dt.txt", 0);

    //treePruning(dt, verifyData, verifyLabel);
    dt.saveTree(dt.decisionTreeRoot, "dt1.txt", 0);

    //测试数据
    vector<int> testData = {2,0,0,1};
    TreeNode *root = dt.decisionTreeRoot;
    int type = dt.classifyData(testData,root);
    cout << "Predict Result: " << type << endl;
    dt.printTree(dt.decisionTreeRoot, 0);  

    cout << "Done" << endl;
    //导入数据
    loadDataParam LDP{
        .Dt = DT,
    };
    loadDataInput LDI{
        .trainDataInput =  trainData,
    };
    loadDataOutput LDO{
        .trainLabelOutput = trainLabel,
    };
    
    loadDecisionTreeData(LDP, LDI, LDO);
    // for (auto i : LDO.trainLabelOutput)
    // {
    //     cout << i;
    // }
    // for (int k = 0 ; k < LDI.trainDataInput.size();k++)
    // {
    //     for (int j = 0; j < LDI.trainDataInput[k].size();j++)
    //     {
    //         cout <<  LDI.trainDataInput[k][j];
    //     }cout << endl;
    // }
    
    //统计数据集中每个标签的数量，比如结果为1的数量和结果为2的数量
    labelCountParam LCP{
        .Dt = DT,
    };
    labelCountInput LCI{.datasetInput = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14}};
    labelCountOutput LCO;
    getDecisionTreeLabelNum(LCP, LCI, LCO);
    // for (auto i : LCO.LabelCountOutput){
    //     cout << "Label:" << i.first << "Count:" << i.second << endl;
    // }
    //计算信息熵，公式为-pi * log(pi)求和
    calculateEntropyParam CEP{
        .Dt = DT,
    };
    calculateEntropyInput CEI{
        .dataset = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14}};
    calculateEntropyOutput CEO;
    calcDecisionTreeEntropy(CEP, CEI, CEO);
    std::cout << "this is CalculateEntropy output: " << CEO.entropyOutput << std::endl;
    //计算信息增益
    calculateGainOrRateParam CGORP{
        .Dt = DT,
    };
    calculateGainOrRateInput CGORI{
        .datasetInput = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14},
        .featureInput = 0,
        .rateFlagInput = true,
    };
    calculateGainOrRateOutput CGORO;
    calcDecisionTreeGainOrRate(CGORP, CGORI, CGORO);
    std::cout << "this is CalculateGainOrRate output: " << CGORO.CalculateGainOrRateOutput << std::endl;
    //获取特征集信息增益中最大的分类信息
    getMaxResultFeatureParam GMRFP{
        .Dt = DT,
    };
    getMaxResultFeatureInput GMRFI{
        .resultsInput =  {{0,0.6},{1,0.5},{2,0.8}},
    };
    getMaxResultFeatureOutput GMRFO;
    getDecisionTreeGetMaxResultFeature(GMRFP, GMRFI, GMRFO);
    std::cout << "this is GetMaxResultFeature output: " << GMRFO.getMaxResultFeatureOutput << std::endl;
    //获取标签统计中出现次数最多的标签
    getMaxTimesLabelParam GMTLP{
        .Dt = DT,
    };
    getMaxTimesLabelInput GMTLI{
        .labelCountInput =  {{0,9},{1,6}},
    };
    getMaxTimesLabelOutput GMTLO;
    getDecisionTreeGetMaxTimesLabel(GMTLP, GMTLI, GMTLO);
    std::cout << "this is GetMaxTimesLabel output: " << GMTLO.getMaxTimesLabelOutput << std::endl;
    //根据特征名来划分子集
    splitDatasetParam SDP{
        .Dt = DT,
    };
    splitDatasetInput SDI{
        .datasetInput =  {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14},
        .featureInput = 1,
        .valueInput = 1,
    };
    splitDatasetOutput SDO;
    splitDecisionTreeDataset(SDP, SDI, SDO);
    for (auto i : SDO.SplitDatasetOutput)
    {
        cout << i << endl;
    }cout << endl;
    //分类
    classifyDataParam CDP{
        .Dt = DT,
    };
    classifyDataInput CDI{
        .testDataInput = testData,
        .rootInput = root,
    };
    classifyDataOutput CDO;
    classifyDecisionTreeData(CDP, CDI, CDO);
    std::cout << "this is ClassifyData output: " << CDO.ClassifyDataOutput << std::endl;
    //创建决策树，默认为ID3算法
    createTreeParam CTP{
        .Dt = DT,
    };
    createTreeInput CTI{
        .datasetInput = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14},
        .featuresInput = {0,1,2,3},
        .rateFlagInput = false,
    };
    createTreeOutput CTO;
    createDecisionTree(CTP, CTI, CTO);
    std::cout << "this is CreateTree output: " << CTO.createTreeOutput << std::endl;
    //打印决策树
    printTreeParam PTP{
        .Dt = DT,
    };
    printTreeInput PTI{
        .NodeInput = root,
        .depthInput = 0,

    };
    printTreeOutput PTO;
    printDecisionTree(PTP, PTI, PTO);
    //保存决策树到文件
    saveTreeParam STP{
        .Dt = DT,
    };
    saveTreeInput STI{
        .NodeInput =  CTO.createTreeOutput,
        .filenameInput = "dt.txt",
        .depthInput = 0,
    };
    saveTreeOutput STO;
    saveDecisionTree(STP, STI, STO);
    //读取文件，加载决策树
    loadTreeParam LTP{
        .Dt = DT,
    };
    loadTreeInput LTI{
        .filenameInput = "/home/pnc/ID3C4.5demo/dt.txt",
        .depthInput = 0,
    };
    loadTreeOutput LTO;
    loadDecisionTree(LTP, LTI, LTO);
    
    testTreeParam TTP{
        .Dt = DT,
    };
    testTreeInput TTI{
        .testDataInput = {
        {0, 1, 1, 1},
        {1, 1, 0, 2},
        {1, 1, 1, 2},
        {2, 0, 1, 0},
        {2, 1, 1, 1},
        {2, 0, 0, 2}},
        .testLabelInput = {0, 1, 1, 1, 1, 1},
    };
    testTreeOutput TTO;
    testDecisionTree(TTP, TTI, TTO);
    
    findPruningNodeParam FPNP{
        .Dt = DT,
    };
    findPruningNodeInput FPNI{
        .NodeInput = root,
    };
    findPruningNodeOutput FPNO;
    findDecisionTreePruningNode(FPNP, FPNI, FPNO);
   
    treePruningParam TPP{
        .Dt = DT,
    };
    treePruningInput TPI{
        .verifyDataInput = verifyData,
        .verifyLabelInput = verifyLabel,
    };
    treePruningOutput TPO;
    pruningDecisionTree(TPP, TPI, TPO);
    

    return 0;
}