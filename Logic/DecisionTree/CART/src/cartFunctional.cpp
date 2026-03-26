/**
 * @brief cart源文件 
 * @file cartFunctional.cpp
 * @version 0.0.1
 * @author Zihan Xie (770178863@qq.com)
 * @date 2023-11-27
 */
#include"cartFunctional.h"
#include"iostream"


namespace Logic{

/**
 * @brief 训练CART决策树
 * @param[IN] treeParam 无效占位
 * @param[IN] trainInput 训练集
 * @param[IN] cartTree 决策树
 
 * @cn_name: 训练CART决策树
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */

void trainCART(const treeParam &param,const trainInput &input,cartTree &output)
{
    long long unsigned int i = 0;
    int sample_num = input.trainLabel.size();
    int feature_num = input.trainData[0].size();
    vector<int> dataset(sample_num);
    vector<int> features(feature_num);
    vector<vector<double>>().swap(output.Tree.trainData);//delete trainDate
    vector<int>().swap(output.Tree.trainLabel);//delate trainLabel
    map<int,set<int>>().swap(output.Tree.featureValues);//delate featureValues
    output.Tree.loadData(input.trainData,input.trainLabel);   
    for (i = 0; i < sample_num; i++)
    {
        dataset[i] = i;
    }
    
    for(i = 0;i < feature_num;i++){
        features[i] = i;
    }
    output.Tree.decisionTreeRoot = output.Tree.createTree(dataset,features);    
}
/**
 * @brief 预测CART样本
 * @param[IN] cartTree 预测决策树
 * @param[IN] predictInput 预测样本
 * @param[IN] predictOutput 预测类别
 
 * @cn_name: 预测CART样本
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void predictCART(const cartTree &param,const predictInput &input,predictOutput &output)
{

    output.type = param.Tree.classifyData(input.sample,param.Tree.decisionTreeRoot);
}

/**
 * @brief 剪枝CART
 * @param[IN] treeParam 无效占位
 * @param[IN] verifyInput 剪枝测试集
 * @param[IN] cartTree 剪枝后的决策树
 
 * @cn_name: 剪枝CART
 
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void pruningCART(const treeParam &param,const verifyInput &input,cartTree &output)
{
    output.Tree.treePruning(output.Tree,input.verifyData,input.verifyLabel);
}
/**
 * @brief 加载CART数据
 * @param[IN] loadDataPara 无效占位
 * @param[IN] loadDataInput 测试数据、测试标签
 * @param[IN] cartTree 需要装载数据的决策树
 
 * @cn_name: 加载CART数据
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void loadCARTData(const loadDataPara &para,const loadDataInput &input,cartTree &output)
{
    output.Tree.loadData(input.trainData,input.trainLabel);
}
/**
 * @brief 获取CART计算类别数量
 * @param[IN] cartTree 决策树对象
 * @param[IN] labelCountInput 样本列表
 * @param[IN] labelCountOutput 类别->数量映射
 
 * @cn_name: 获取CART计算类别数量
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void getcartLabelCount(cartTree &para,labelCountInput &input,labelCountOutput &output)
{
    output.labelCounts = para.Tree.labelCount(input.dataset);
}
/**
 * @brief 计算CART基尼值
 * @param[IN] cartTree 决策树对象
 * @param[IN] calcGiniValueInput 样本列表
 * @param[IN] calcGiniValueOutput 基尼值
 
 * @cn_name: 计算CART基尼值
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void calcCARTGiniValue(cartTree &para,calcGiniValueInput &input,calcGiniValueOutput &output)
{
    output.giniValue = para.Tree.calculateGiniValue(input.dataset);
}
/**
 * @brief 分离CART数据集
 * @param[IN] cartTree 决策树对象
 * @param[IN] splitDataInput 样本列表、特征、阈值，大于/小于阈值
 * @param[IN] splitDataOutput 分离出来的样本列表
 
 * @cn_name: 分离CART数据集
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void splitCARTDataset(cartTree &para,splitDataInput &input,splitDataOutput &output)
{
    output.outputDataset = para.Tree.splitDataset(input.dataset,input.feature,input.threshold,input.bigger);
}
/**
 * @brief 计算CART基尼系数
 * @param[IN] cartTree 决策树对象
 * @param[IN] calcGiniIndexInput 样本列表、特征、阈值
 * @param[IN] calcGiniIndexOutput 基尼系数
 
 * @cn_name: 计算CART基尼系数
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void calcCARTGiniIndex(cartTree &para,calcGiniIndexInput &input,calcGiniIndexOutput &output)
{
    output.giniIndex = para.Tree.calculateGiniIndex(input.dataset,input.feature,input.threshold);
}
/**
 * @brief 建立CART决策树
 * @param[IN] createPara 空，占位
 * @param[IN] createInput 空、占位
 * @param[IN] cartTree 决策树对象
 
 * @cn_name: 建立CART决策树
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void createCARTTree(createPara &para,createInput &input,cartTree &output)
{
    vector<int> dataset(output.Tree.trainData.size()); //数据集
    long long unsigned int i = 0;
    for (i = 0; i < output.Tree.trainData.size(); i++)
    {
        dataset[i] = i;
    }
    vector<int> features(output.Tree.trainData[0].size());//属性集合
    for(i = 0;i < output.Tree.trainData[0].size();i++){
        features[i] = i;
    }
    output.Tree.decisionTreeRoot = output.Tree.createTree(dataset,features);
}
/**
 * @brief 打印CART决策树
 * @param[IN] printPara 空，占位
 * @param[IN] cartTree 决策树对象
 * @param[IN] printOutput 空、占位
 
 * @cn_name: 打印CART决策树
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void printCARTTree(printPara &para,cartTree &input,printOutput &output)
{
    input.Tree.printTree(input.Tree.decisionTreeRoot,0);
}
/**
 * @brief CART分类
 * @param[IN] cartTree 决策树对象
 * @param[IN] classInput 样本数据
 * @param[IN] classOutput 类别
 
 * @cn_name: CART分类
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void classifyCARTData(cartTree &para,classInput &input,classOutput &output)
{
    output.type = para.Tree.classifyData(input.testData,para.Tree.decisionTreeRoot);
}
/**
 * @brief 保存CART决策树
 * @param[IN] savePara 空、占位
 * @param[IN] saveInput 决策树对象、文件路径
 * @param[IN] saveOutput 空、占位
 
 * @cn_name: 保存CART决策树
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void saveCARTTree(savePara &para,saveInput &input,saveOutput &output)
{
    input.Tree.saveTree(input.Tree.decisionTreeRoot,input.filename,0);
}
/**
 * @brief 加载CART决策树
 * @param[IN] loadTreePara 空、占位
 * @param[IN] loadTreeInput 文件路径
 * @param[IN] cartTree 决策树对象
 
 * @cn_name: 加载CART决策树
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void loadCARTTree(loadTreePara &para,loadTreeInput &input,cartTree &output)
{
    int l = 0;

    int *line;
    line = &l;
    output.Tree.decisionTreeRoot = output.Tree.loadTree(input.filename,0,line);
}
/**
 * @brief 测试CART决策树
 * @param[IN] cartTree 决策树对象
 * @param[IN] testTreeInput 测试样本
 * @param[IN] testTreeOutput 正确率
 
 * @cn_name: 测试CART决策树
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void testCARTTree(cartTree &para,testTreeInput &input,testTreeOutput &output)
{
    output.correctRate = para.Tree.testTree(para.Tree,input.testData,input.testLabel);
}
/**
 * @brief 获取CART最多的类别
 * @param[IN] cartTree 决策树对象
 * @param[IN] getMaxInput 类别数量映射
 * @param[IN] getMaxOutput 类别
 
 * @cn_name: 获取CART最多的类别
 
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 
 * @tag: logic
 */
void getCARTMaxTimesLabel(cartTree &para,getMaxInput &input,getMaxOutput &output)
{
    output.type = para.Tree.getMaxTimesLabel(input.labelCount);
}

}