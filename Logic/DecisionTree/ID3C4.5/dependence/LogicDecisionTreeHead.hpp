/*
 * @Description: 决策树头文件
 */ 
#pragma once
#include<vector>
#include<map>
#include<set>
#include<fstream>
#include<string>
using namespace std;

/**
 * \namespace Logic
 * \brief Logic namespace
 */

namespace Logic{

//决策树节点结构
struct TreeNode{
    bool isLeaf = false;//是否是叶子节点
    int result = -1;//如果是叶子节点的话，对应的label索引
    vector<TreeNode*> branches;//分支节点
    int attrId = -1;//特征
    int attrValue = -1;//特征值
    bool isPruning = false;//是否被剪枝操作过
};

class DecisionTree{
public:
    vector<vector<int>> trainData;//训练数据集的特征数据
    vector<int> trainLabel;//训练数据集对应的标签
    map<int,set<int>> featureValues;//每个特征的类别
    TreeNode *decisionTreeRoot;//决策树的根节点
    vector<int> dataset;
    DecisionTree();
    DecisionTree(const char *filename, int depth, int line);
    DecisionTree(const vector<vector<int>> &trainData, const vector<int> &trainLabel,const bool rateFlag);
    void loadData(const vector<vector<int>> &trainData,const vector<int> &trainLabel);// 导入数据
    map<int,int> labelCount(vector<int> &dataset);//统计数据集中每个标签的数量，比如结果为1的数量和结果为2的数量
    double calculateEntropy(vector<int> &dataset);//计算信息熵
    vector<int> splitDataset(vector<int> &dataset,int &feature,int &value);//分割数据集
    double calculateGainOrRate(vector<int> &dataset,int &feature,bool rateFlag);//计算信息增益或信息增益率
    int getMaxTimesLabel(map<int,int> &labelCount);//获取出现次数最多的标签
    int getMaxResultFeature(map<int,double> &gains);//获取最大分类信息的特征
    TreeNode *createTree(vector<int> &dataset, vector<int> &features,bool rateFlag);//创建决策树
    int classifyData(const vector<int> &testData,TreeNode *root) const;//测试数据分类
    void printTree(TreeNode* Node, int depth);//打印决策树
    void saveTree(TreeNode* Node, const char *filename, int depth);//保存决策树
    TreeNode *loadTree(const char *filename, int depth, int *line);//加载决策树
    double testTree(DecisionTree &dt,const vector<vector<int>> &testData,const vector<int> &testLabel);
    TreeNode* findPruningNode(TreeNode *Node);
    void treePruning(DecisionTree &dt,const vector<vector<int>> &verifyData,const vector<int> &verifyLabel);

    
};

}