/**
 * @brief ID3C45头文件
 * @file ID3C45Functional.h
 * @version 0.0.1
 * @author Hongyu Liu
 * @date 2023-12-22
 */
#include "LogicDecisionTreeHead.hpp"
#include <iostream>
using namespace std;

/**
 * \namespace Logic
 * \brief Logic namespace
 */

namespace Logic{

struct loadDataParam
{
    DecisionTree &Dt;
};
struct loadDataInput
{
    vector<vector<int>> &trainDataInput;
};
struct loadDataOutput
{
    vector<int> &trainLabelOutput;
};

struct labelCountParam
{
    DecisionTree &Dt;
};
struct labelCountInput
{
    vector<int> datasetInput;
};
struct labelCountOutput
{
    map<int,int> LabelCountOutput;
};

struct calculateEntropyParam
{
    DecisionTree &Dt;
};
struct calculateEntropyInput
{
    vector<int> dataset;
};
struct calculateEntropyOutput
{
    double entropyOutput;
};

struct calculateGainOrRateParam
{
    DecisionTree &Dt;
};
struct calculateGainOrRateInput
{
    vector<int> datasetInput;
    int featureInput;
    bool rateFlagInput;
};
struct calculateGainOrRateOutput
{
    double CalculateGainOrRateOutput;
};

struct classifyDataParam
{
    DecisionTree &Dt;
};
struct classifyDataInput
{
    vector<int> testDataInput;
    TreeNode *rootInput;
};
struct classifyDataOutput
{
    int ClassifyDataOutput;
};

struct createTreeParam
{
    DecisionTree &Dt;
};
struct createTreeInput
{
    vector<int> datasetInput;
    vector<int> featuresInput;
    bool rateFlagInput;
};
struct createTreeOutput
{
    TreeNode* createTreeOutput;
};

struct getMaxResultFeatureParam
{
    DecisionTree &Dt;
};
struct getMaxResultFeatureInput
{
    map<int,double> resultsInput;
};
struct getMaxResultFeatureOutput
{
    int getMaxResultFeatureOutput;
};

struct getMaxTimesLabelParam
{
    DecisionTree &Dt;
};
struct getMaxTimesLabelInput
{
    map<int,int> labelCountInput;
};
struct getMaxTimesLabelOutput
{
    int getMaxTimesLabelOutput;
};

struct printTreeParam
{
    DecisionTree &Dt;
};
struct printTreeInput
{
    TreeNode* NodeInput;
    int depthInput;
};
struct printTreeOutput
{
    
};

struct splitDatasetParam
{
    DecisionTree &Dt;
};
struct splitDatasetInput
{
    vector<int> datasetInput;
    int featureInput;
    int valueInput;
};
struct splitDatasetOutput
{
    vector<int> SplitDatasetOutput;
};

struct saveTreeParam
{
    DecisionTree &Dt;
};
struct saveTreeInput
{
    TreeNode* NodeInput;
    const char* filenameInput;
    int depthInput;
};
struct saveTreeOutput
{
    
};

struct loadTreeParam
{
    DecisionTree &Dt;
};
struct loadTreeInput
{
    const char* filenameInput;
    int depthInput;
};
struct loadTreeOutput
{
    TreeNode* LoadTreeOutput;
};

struct testTreeParam
{
    DecisionTree &Dt;
};
struct testTreeInput
{
    vector<vector<int>> testDataInput;
    vector<int> testLabelInput;
};
struct testTreeOutput
{
    double testTreeOutput;
};
struct findPruningNodeParam
{
    DecisionTree &Dt;
};
struct findPruningNodeInput
{
    TreeNode *NodeInput;
};
struct findPruningNodeOutput
{
    TreeNode* findPruningNodeOutput;
};
struct treePruningParam
{
    DecisionTree &Dt;
};
struct treePruningInput
{
    vector<vector<int>> verifyDataInput;
    vector<int> verifyLabelInput;
};
struct treePruningOutput
{

};
/**
 * @brief 计算决策树信息熵
 * @param[IN] param 决策树对象
 * @param[IN] input 数据集
 * @param[IN] output 信息熵
 * @cn_name: 计算决策树信息熵
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void calcDecisionTreeEntropy(calculateEntropyParam &param01, calculateEntropyInput &input01, calculateEntropyOutput &output01);
/**
 * @brief 计算决策树信息增益
 * @param[IN] param 决策树对象
 * @param[IN] input 数据集、对应特征、true返回信息增益率，false返回信息增益
 * @param[IN] output 分类信息（信息增益、信息增益率）
 * @cn_name: 计算决策树信息增益
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void calcDecisionTreeGainOrRate(calculateGainOrRateParam &param02, calculateGainOrRateInput &input02, calculateGainOrRateOutput &output02);
/**
 * @brief 决策树分类
 * @param[IN] param 决策树对象
 * @param[IN] input 测试数据 、决策树根节点
 * @param[IN] output 返回分类结果
 * @cn_name: 决策树分类
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void classifyDecisionTreeData(classifyDataParam &param03, classifyDataInput &input03, classifyDataOutput &output03);
/**
 * @brief 创建决策树
 * @param[IN] param 决策树对象
 * @param[IN] input 数据集、特征集 
 * @param[IN] output 返回决策树根节点指针
 * @cn_name: 创建决策树
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void createDecisionTree(createTreeParam &param04, createTreeInput &input04, createTreeOutput &output04);
/**
 * @brief 决策树获取特征集信息增益中最大的分类信息（信息增益、信息增益率）和所对应的特征
 * @param[IN] param 决策树对象
 * @param[IN] input 特征集的信息增益
 * @param[IN] output 最大分类信息对应的特征
 * @cn_name: 获取决策树最大增益及特征
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void getDecisionTreeGetMaxResultFeature(getMaxResultFeatureParam &param05, getMaxResultFeatureInput &input05, getMaxResultFeatureOutput &output05);
/**
 * @brief 获取标签统计中出现次数最多的标签
 * @param[IN] param 决策树对象
 * @param[IN] input 标签统计
 * @param[IN] output 返回出现次数最多的标签名
 * @cn_name: 获取标签统计中出现次数最多的标签
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void getDecisionTreeGetMaxTimesLabel(getMaxTimesLabelParam &param06, getMaxTimesLabelInput &input06, getMaxTimesLabelOutput &output06);
/**
 * @brief 决策树类别数量
 * @param[IN] param 决策树对象
 * @param[IN] input 样本列表
 * @param[IN] output 类别->数量映射
 * @cn_name: 决策树类别数量
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void getDecisionTreeLabelNum(labelCountParam &param07, labelCountInput &input07, labelCountOutput &output07);
/**
 * @brief 导入决策树数据
 * @param[IN] param 决策树对象
 * @param[IN] input 训练数据，训练标签
 * @param[IN] output 需要装在数据的决策树
 * @cn_name: 导入决策树数据
 * @granularity: atomic //函数组件的粒度，atomic:基础组件（原子）,composite:复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void loadDecisionTreeData(loadDataParam &param08, loadDataInput &input08, loadDataOutput &output08);
/**
 * @brief 打印决策树
 * @param[IN] param 决策树对象
 * @param[IN] input 起始节点 、打印缩进
 * @param[IN] output 没有返回值
 * @cn_name: 打印决策树
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void printDecisionTree(printTreeParam &param09, printTreeInput &input09, printTreeOutput &output09);
/**
 * @brief 划分决策树子集
 * @param[IN] param 决策树对象
 * @param[IN] input 数据集、第几个特征、第几个特征的特征值
 * @param[IN] output 返回划分的子集
 * @cn_name: 划分决策树子集
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void splitDecisionTreeDataset(splitDatasetParam &param10, splitDatasetInput &input10, splitDatasetOutput &output10);
/**
 * @brief 保存决策树到文件
 * @param[IN] param 决策树对象
 * @param[IN] input 决策树根节点 、文件名、文件缩进值
 * @param[IN] output 没有返回值
 * @cn_name: 保存决策树
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void saveDecisionTree(saveTreeParam &param11, saveTreeInput &input11, saveTreeOutput &output11);
/**
 * @brief 决策树读取文件，加载决策树
 * @param[IN] param 决策树对象
 * @param[IN] input 文件名 、文件缩进值、读取文件起始行数
 * @param[IN] output 根节点指针
 * @cn_name: 加载决策树
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void loadDecisionTree(loadTreeParam &param12, loadTreeInput &input12, loadTreeOutput &output12);
/**
 * @brief 测试决策树
 * @param[IN] param 决策树对象
 * @param[IN] input 决策树 、测试数据、测试数据标签真值
 * @param[IN] output 准确率
 * @cn_name: 测试决策树
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void testDecisionTree(testTreeParam &param, testTreeInput &input, testTreeOutput &output);
/**
 * @brief 寻找决策树剪枝节点
 * @param[IN] param 决策树对象
 * @param[IN] input 决策树根节点指针
 * @param[IN] output 剪枝节点指针
 * @cn_name: 寻找决策树剪枝节点
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void findDecisionTreePruningNode(findPruningNodeParam &param, findPruningNodeInput &input, findPruningNodeOutput &output);
/**
 * @brief 决策树剪枝
 * @param[IN] param 决策树对象
 * @param[IN] input 决策树、验证数据、验证数据标签
 * @param[IN] output 没有返回
 * @cn_name: 决策树剪枝
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void pruningDecisionTree(treePruningParam &param, treePruningInput &input, treePruningOutput &output);

}