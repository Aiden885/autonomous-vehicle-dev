/**
 * @brief cart头文件
 * @file cartFunctional.h
 * @version 0.0.1
 * @author Zihan Xie (770178863@qq.com)
 * @date 2023-11-27
 */
#include"LogicDecisionTreeCARTHead.hpp"
using namespace std;

/**
 * \namespace Logic
 * \brief Logic namespace
 */

namespace Logic{

struct treeParam{};
struct trainInput{
    vector<vector<double>> trainData;
    vector<int> trainLabel;
};


struct verifyInput{
    vector<vector<double>> verifyData;
    vector<int> verifyLabel;
};

struct predictInput{
    vector<double> sample;
};

struct predictOutput{
    int type;
};

//决策树节点结构


struct cartTree{
    DecisionTree Tree;
};

struct loadDataPara{};


struct loadDataInput
{
    vector<vector<double>> trainData;
    vector<int> trainLabel;
};

struct labelCountPara{};

struct labelCountInput{vector<int> dataset;};

struct labelCountOutput{map<int,int> labelCounts;};

struct calcGiniValueInput{vector<int> dataset;};

struct calcGiniValueOutput{double giniValue;};


struct splitDataInput
{
    vector<int> dataset;
    int feature;
    double threshold;
    bool bigger;
};

struct calcGiniIndexInput
{
    vector<int> dataset;
    int feature;
    double threshold;
};

struct calcGiniIndexOutput{double giniIndex;};

struct splitDataOutput{vector<int> outputDataset;};

struct createPara{};

struct createInput{};

struct classInput
{
    vector<double> testData;
};

struct classOutput
{
    int type;
};


struct printPara{};

struct printOutput{};

struct savePara{};

struct saveInput
{
    DecisionTree Tree;
    char *filename;
};

struct saveOutput{};

struct loadTreePara{};

struct loadTreeInput
{
    const char *filename;
};



struct testTreeInput
{
    vector<vector<double>> testData;
    vector<int> testLabel;
};

struct testTreeOutput{double correctRate;};

struct getMaxInput{map<int,int> &labelCount;};

struct getMaxOutput{int type;};


/**
 * @brief 加载CART数据
 * @param[IN] loadDataPara 无效占位
 * @param[IN] loadDataInput 测试数据、测试标签
 * @param[IN] cartTree 需要装载数据的决策树
 * @cn_name: 加载CART数据
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void loadCARTData(const loadDataPara &para,const loadDataInput &input,cartTree &output);
/**
 * @brief 获取CART计算类别数量
 * @param[IN] cartTree 决策树对象
 * @param[IN] labelCountInput 样本列表
 * @param[IN] labelCountOutput 类别->数量映射
 * @cn_name: 获取CART计算类别数量
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void getcartLabelCount(cartTree &para,labelCountInput &input,labelCountOutput &output);
/**
 * @brief 计算CART基尼值
 * @param[IN] cartTree 决策树对象
 * @param[IN] calcGiniValueInput 样本列表
 * @param[IN] calcGiniValueOutput 基尼值
 * @cn_name: 计算CART基尼值
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void calcCARTGiniValue(cartTree &para,calcGiniValueInput &input,calcGiniValueOutput &output);
/**
 * @brief 训练CART决策树
 * @param[IN] treeParam 无效占位
 * @param[IN] trainInput 训练集
 * @param[IN] cartTree 决策树
 * @cn_name: 训练CART决策树
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void trainCART(const treeParam &param,const trainInput &input,cartTree &output);
/**
 * @brief 预测CART样本
 * @param[IN] cartTree 预测决策树
 * @param[IN] predictInput 预测样本
 * @param[IN] predictOutput 预测类别
 * @cn_name: 预测CART样本
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void predictCART(const cartTree &param,const predictInput &input,predictOutput &output);
/**
 * @brief 剪枝CART
 * @param[IN] treeParam 无效占位
 * @param[IN] verifyInput 剪枝测试集
 * @param[IN] cartTree 剪枝后的决策树
 * @cn_name: 剪枝CART
 * @granularity: composite //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void pruningCART(const treeParam &param,const verifyInput &input,cartTree &output);
/**
 * @brief 分离CART数据集
 * @param[IN] cartTree 决策树对象
 * @param[IN] splitDataInput 样本列表、特征、阈值，大于/小于阈值
 * @param[IN] splitDataOutput 分离出来的样本列表
 * @cn_name: 分离CART数据集
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void splitCARTDataset(cartTree &para,splitDataInput &input,splitDataOutput &output);
/**
 * @brief 计算CART基尼系数
 * @param[IN] cartTree 决策树对象
 * @param[IN] calcGiniIndexInput 样本列表、特征、阈值
 * @param[IN] calcGiniIndexOutput 基尼系数
 * @cn_name: 计算CART基尼系数
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void calcCARTGiniIndex(cartTree &para,calcGiniIndexInput &input,calcGiniIndexOutput &output);
/**
 * @brief 建立CART决策树
 * @param[IN] createPara 空，占位
 * @param[IN] createInput 空、占位
 * @param[IN] cartTree 决策树对象
 * @cn_name: 建立CART决策树
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void createCARTTree(createPara &para,createInput &input,cartTree &output);
/**
 * @brief CART分类
 * @param[IN] cartTree 决策树对象
 * @param[IN] classInput 样本数据
 * @param[IN] classOutput 类别
 * @cn_name: CART分类
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void classifyCARTData(cartTree &para,classInput &input,classOutput &output);
/**
 * @brief 打印CART决策树
 * @param[IN] printPara 空，占位
 * @param[IN] cartTree 决策树对象
 * @param[IN] printOutput 空、占位
 * @cn_name: 打印CART决策树
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */

void printCARTTree(printPara &para,cartTree &input,printOutput &output);
/**
 * @brief 保存CART决策树
 * @param[IN] savePara 空、占位
 * @param[IN] saveInput 决策树对象、文件路径
 * @param[IN] saveOutput 空、占位
 * @cn_name: 保存CART决策树
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void saveCARTTree(savePara &para,saveInput &input,saveOutput &output);
/**
 * @brief 加载CART决策树
 * @param[IN] loadTreePara 空、占位
 * @param[IN] loadTreeInput 文件路径
 * @param[IN] cartTree 决策树对象
 * @cn_name: 加载CART决策树
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void loadCARTTree(loadTreePara &para,loadTreeInput &input,cartTree &output);
/**
 * @brief 测试CART决策树
 * @param[IN] cartTree 决策树对象
 * @param[IN] testTreeInput 测试样本
 * @param[IN] testTreeOutput 正确率
 * @cn_name: 测试CART决策树
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void testCARTTree(cartTree &para,testTreeInput &input,testTreeOutput &output);
/**
 * @brief 获取CART最多的类别
 * @param[IN] cartTree 决策树对象
 * @param[IN] getMaxInput 类别数量映射
 * @param[IN] getMaxOutput 类别
 * @cn_name: 获取CART最多的类别
 * @granularity: atomic //函数组件的粒度， atomic： 基础组件（原子），composite： 复合组件（在前端可以向下一层展开）
 * @tag: logic
 */
void getCARTMaxTimesLabel(cartTree &para,getMaxInput &input,getMaxOutput &output);


}