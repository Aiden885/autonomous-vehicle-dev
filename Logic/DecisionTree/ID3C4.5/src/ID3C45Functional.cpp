/**
 * @brief ID3C45源文件
 * @file ID3C45Functional.cpp
 * @version 0.0.1
 * @author Hongyu Liu
 * @date 2023-12-22
 */
#include <iostream>
#include"./ID3C45Functional.h"
using namespace std;
#include <map>

namespace Logic{

/**
 * @brief 导入决策树数据
 * @param[IN] param 决策树对象
 * @param[IN] input 训练数据，训练标签
 * @param[IN] output 需要装在数据的决策树
 
 * @cn_name: 导入决策树数据
 
 * @granularity: atomic //函数组件的粒度，atomic:基础组件（原子）,composite:复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void loadDecisionTreeData(loadDataParam &param, loadDataInput &input, loadDataOutput &output)
{
    param.Dt.loadData(input.trainDataInput, output.trainLabelOutput);
}
/**
 * @brief 决策树类别数量
 * @param[IN] param 决策树对象
 * @param[IN] input 样本列表
 * @param[IN] output 类别->数量映射
 
 * @cn_name: 决策树类别数量
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void getDecisionTreeLabelNum(labelCountParam &param, labelCountInput &input, labelCountOutput &output)
{
    output.LabelCountOutput = param.Dt.labelCount(input.datasetInput);
}
/**
 * @brief 计算决策树信息熵
 * @param[IN] param 决策树对象
 * @param[IN] input 数据集
 * @param[IN] output 信息熵
 
 * @cn_name: 计算决策树信息熵
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void calcDecisionTreeEntropy(calculateEntropyParam &param, calculateEntropyInput &input, calculateEntropyOutput &output)
{
     output.entropyOutput = param.Dt.calculateEntropy(input.dataset);
}
/**
 * @brief 计算决策树信息增益
 * @param[IN] param 决策树对象
 * @param[IN] input 数据集、对应特征、true返回信息增益率，false返回信息增益
 * @param[IN] output 分类信息（信息增益、信息增益率）
 
 * @cn_name: 计算决策树信息增益
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void calcDecisionTreeGainOrRate(calculateGainOrRateParam &param, calculateGainOrRateInput &input, calculateGainOrRateOutput &output)
{
    output.CalculateGainOrRateOutput = param.Dt.calculateGainOrRate(input.datasetInput, input.featureInput, input.rateFlagInput);
}
/**
 * @brief 决策树获取特征集信息增益中最大的分类信息（信息增益、信息增益率）和所对应的特征
 * @param[IN] param 决策树对象
 * @param[IN] input 特征集的信息增益
 * @param[IN] output 最大分类信息对应的特征
 
 * @cn_name: 获取决策树最大增益及特征
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void getDecisionTreeGetMaxResultFeature(getMaxResultFeatureParam &param, getMaxResultFeatureInput &input, getMaxResultFeatureOutput &output)
{
    output.getMaxResultFeatureOutput = param.Dt.getMaxResultFeature(input.resultsInput);
}
/**
 * @brief 获取标签统计中出现次数最多的标签
 * @param[IN] param 决策树对象
 * @param[IN] input 标签统计
 * @param[IN] output 返回出现次数最多的标签名
 
 * @cn_name: 获取标签统计中出现次数最多的标签
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void getDecisionTreeGetMaxTimesLabel(getMaxTimesLabelParam &param, getMaxTimesLabelInput &input, getMaxTimesLabelOutput &output)
{
    output.getMaxTimesLabelOutput = param.Dt.getMaxTimesLabel(input.labelCountInput);
}
/**
 * @brief 划分决策树子集
 * @param[IN] param 决策树对象
 * @param[IN] input 数据集、第几个特征、第几个特征的特征值
 * @param[IN] output 返回划分的子集
 
 * @cn_name: 划分决策树子集
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void splitDecisionTreeDataset(splitDatasetParam &param, splitDatasetInput &input, splitDatasetOutput &output)
{
    output.SplitDatasetOutput = param.Dt.splitDataset(input.datasetInput, input.featureInput, input.valueInput);
}
/**
 * @brief 创建决策树
 * @param[IN] param 决策树对象
 * @param[IN] input 数据集、特征集 
 * @param[IN] output 返回决策树根节点指针
 
 * @cn_name: 创建决策树
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void createDecisionTree(createTreeParam &param, createTreeInput &input, createTreeOutput &output)
{
    output.createTreeOutput = param.Dt.createTree(input.datasetInput, input.featuresInput, input.rateFlagInput);
}
/**
 * @brief 决策树分类
 * @param[IN] param 决策树对象
 * @param[IN] input 测试数据 、决策树根节点
 * @param[IN] output 返回分类结果
 
 * @cn_name: 决策树分类
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void classifyDecisionTreeData(classifyDataParam &param, classifyDataInput &input, classifyDataOutput &output)
{
    output.ClassifyDataOutput = param.Dt.classifyData(input.testDataInput, input.rootInput);
}
/**
 * @brief 打印决策树
 * @param[IN] param 决策树对象
 * @param[IN] input 起始节点 、打印缩进
 * @param[IN] output 没有返回值
 
 * @cn_name: 打印决策树
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void printDecisionTree(printTreeParam &param, printTreeInput &input, printTreeOutput &output)
{
    param.Dt.printTree(input.NodeInput, input.depthInput);
}
/**
 * @brief 保存决策树
 * @param[IN] param 决策树对象
 * @param[IN] input 决策树根节点 、文件名、文件缩进值
 * @param[IN] output 没有返回值
 
 * @cn_name: 保存决策树
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void saveDecisionTree(saveTreeParam &param, saveTreeInput &input, saveTreeOutput &output)
{
    param.Dt.saveTree(input.NodeInput, input.filenameInput, input.depthInput);
}
/**
 * @brief 决策树读取文件，加载决策树
 * @param[IN] param 决策树对象
 * @param[IN] input 文件名 、文件缩进值、读取文件起始行数
 * @param[IN] output 根节点指针
 
 * @cn_name: 加载决策树
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void loadDecisionTree(loadTreeParam &param, loadTreeInput &input, loadTreeOutput &output)
{
    int l = 0;
    int *line = &l;
    output.LoadTreeOutput = param.Dt.loadTree(input.filenameInput, input.depthInput, line);
}
/**
 * @brief 测试决策树
 * @param[IN] param 决策树对象
 * @param[IN] input 决策树 、测试数据、测试数据标签真值
 * @param[IN] output 准确率
 
 * @cn_name: 测试决策树
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void testDecisionTree(testTreeParam &param, testTreeInput &input, testTreeOutput &output)
{
    output.testTreeOutput = param.Dt.testTree(param.Dt, input.testDataInput, input.testLabelInput);
}
/**
 * @brief 寻找决策树剪枝节点
 * @param[IN] param 决策树对象
 * @param[IN] input 决策树根节点指针
 * @param[IN] output 剪枝节点指针
 
 * @cn_name: 寻找决策树剪枝节点
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void findDecisionTreePruningNode(findPruningNodeParam &param, findPruningNodeInput &input, findPruningNodeOutput &output)
{
    output.findPruningNodeOutput = param.Dt.findPruningNode(input.NodeInput);
}
/**
 * @brief 决策树剪枝
 * @param[IN] param 决策树对象
 * @param[IN] input 决策树、验证数据、验证数据标签
 * @param[IN] output 没有返回
 
 * @cn_name: 决策树剪枝
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void pruningDecisionTree(treePruningParam &param, treePruningInput &input, treePruningOutput &output)
{
    param.Dt.treePruning(param.Dt, input.verifyDataInput, input.verifyLabelInput);
}
}