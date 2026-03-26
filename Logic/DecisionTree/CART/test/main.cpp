/**
 * @brief 测试用例
 * @file main.cpp
 * @version 0.0.1
 * @author Zehang Zhu (zzh22@tsinghua.edu.com)
 * @date 2023-11-25
 */

#include <iostream>
#include "../src/cartFunctional.h"
using namespace std;

void coutData(vector<int> data)
{
    for(auto i:data) cout<<i<<' ';
    cout<<endl;
}

void coutData(vector<vector<double>> data)
{
    for(auto i:data)
    {
        for(auto j:i)
            cout<<j<<' ';
        cout<<endl;
    }
}

void coutData(map<int,int> data)
{
    for(auto i:data)
    {
        cout<<i.first<<":"<<i.second<<endl;
    }
}

int main(int argc, const char * argv[]) {
    //训练数据
//     struct trainInput traininput = {
    
//  {{0, 0, 0, 0},
//         {0, 0, 0, 1},
//         {0, 1, 0, 1},
//         {0, 1, 1, 0},
//         {0, 0, 0, 0},
//         {1, 0, 0, 0},
//         {1, 0, 0, 1},
//         {1, 1, 1, 1},
//         {1, 0, 1, 2},
//         {1, 0, 1, 2},
//         {2, 0, 1, 2},
//         {2, 0, 1, 1},
//         {2, 1, 0, 1},
//         {2, 1, 0, 2},
//         {2, 0, 0, 0}}
//     ,{0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0}
//     };

//     DecisionTree dt;
//     cartTree t = {dt};
//     cartTrain(treeParam(),traininput,t);
//     t.Tree.saveTree(t.Tree.decisionTreeRoot, "dt1.txt", 0);
    verifyInput verifyData = {
        {
        {0, 1, 1, 1},
        {1, 1, 0, 2},
        {1, 1, 1, 2},
        {2, 0, 1, 0},
        {2, 1, 1, 1},
        {2, 0, 0, 2}
    },{0, 0, 1, 1, 0, 0}
    };


//     cartPruning(treeParam(), verifyData,t);
//     t.Tree.saveTree(t.Tree.decisionTreeRoot, "dt1.txt", 0);

    
//     //测试数据
//     predictInput predictdata = {{2,0,0,1}};
//     predictOutput type;
//     cartPredict(t,predictdata,type);
//     cout << "Predict Result: " << type.type << endl;
//     dt.printTree(t.Tree.decisionTreeRoot, 0);
//     cout << "Done" << endl;

    cartTree Tree1;

    loadDataPara para1;


    loadDataInput input1
    {
       {{0, 0, 0, 0},
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
        {2, 0, 0, 0}},
        {0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0}
    };

    // calcGiniIndexInput input2{{0,3,6,10,14,7,9},2,3};
    // calcGiniIndexOutput output2;

    // splitDataInput input5{{0,3,6,10},0,1,true};
    // splitDataOutput output5;

    labelCountPara para5;
    labelCountInput input5{{0,1,2,3,4,5,6,7,8,9,10,11,12,13,14}};
    labelCountOutput output5;

    createPara para3;
    createInput input3;

    printPara para4;
    printOutput output4;

    classInput input6{{0, 1, 1, 1}};
    classOutput output6;

    testTreeInput input8
    {{
        {0, 1, 1, 1},
        {1, 1, 0, 2},
        {1, 1, 1, 2},
        {2, 0, 1, 0},
        {2, 1, 1, 1},
        {2, 0, 0, 2}
    },{0, 0, 1, 1, 0, 0}       };
    testTreeOutput output8{0};
    const char *c = "/home/pnc/for_tsinghua/dependencies/Logic/decision tree demo/cart/t.txt";
    loadTreePara para7{};
    loadTreeInput input7{c};
    loadCARTTree(para7,input7,Tree1);
    // cartTestTree(Tree1,input8,output8);
    // calcGiniValueInput input3{{1,2,3,4}};
    // calcGiniValueOutput output3; 
    // cartLoadData(para1,input1,Tree1);
    // cartCalculateGiniValue(Tree1,input3,output3);
    // cout<<output3.giniValue<<endl;
    // cout<<"trainData:"<<endl;
    // coutData(Tree1.Tree.trainData);
    // cout<<"trainlabel:"<<endl;
    // coutData(Tree1.Tree.trainLabel);
    // cartCreateTree(para3,input3,Tree1);
    // savePara para7;
    // saveInput input7{Tree1.Tree,"t.txt"};
    // saveOutput output7;
    // cartSaveTree(para7,input7,output7);
    // cartClassifyData(Tree1,input6,output6);
    // cout<<output6.type<<endl;
    cout<<"before:"<<endl;
    printCARTTree(para4,Tree1,output4);
    pruningCART(treeParam(),verifyData,Tree1);
    cout<<"after:"<<endl;
    printCARTTree(para4,Tree1,output4);
    // cartLabelCount(Tree1,input5,output5);
    // getMaxInput input6{output5.labelCounts};
    // getMaxOutput output6;
    // cartGetMaxTimesLabel(Tree1,input6,output6);
    // cout<<output6.type<<endl;
    // cartSplitDataset(Tree1,input5,output5);
    // coutData(output5.outputDataset);
    // cartSplitDataset(Tree1,input5,output5);
    //cartCalculateGiniIndex(Tree1,input2,output2);
    // coutData(Tree1.Tree.trainData);
    // coutData(Tree1.Tree.trainLabel);
    // cartLabelCount(Tree1,input5,output5);
    // coutData(output5.labelCounts);
    // coutData(output2.giniIndex);
    //cout<<"giniIndex:"<<output2.giniIndex;
    // calculateGiniValue(Tree1,input3,output3);
    // cout<<output3.giniValue;
    return 0;
}
