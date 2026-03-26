/*
 * @Description: 决策树头文件
 */ 
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

struct TreeNode{
    bool isLeaf = false;//是否是叶子节点
    bool isPruning = false;//是否剪枝处理过
    double result = -1; //分类结果
    TreeNode *left;//左节点，小于阈值
    TreeNode *right;//右节点，大于阈值
    int attrId = -1;//特征
    double attrThreshold = -99;//特征分类阈值
};

class DecisionTree{
public:
    vector<vector<double>> trainData;//训练数据集的特征数据
    vector<int> trainLabel;//训练数据集对应的标签
    map<int,set<int>> featureValues;//每个特征的类别
    TreeNode *decisionTreeRoot;//决策树的根节点

    DecisionTree();
    DecisionTree(const char *filename, int depth, int line);
    DecisionTree(const vector<vector<double>> &trainData, const vector<int> &trainLabel);
    void loadData(const vector<vector<double>> &trainData,const vector<int> &trainLabel);// 导入数据
    map<int,int> labelCount(vector<int> &dataset);//统计数据集中每个标签的数量，比如结果为1的数量和结果为2的数量
    double calculateGiniValue(vector<int> &dataset);//计算基尼值
    vector<int> splitDataset(vector<int> &dataset,int &feature,double &threshold, bool bigger);//分割数据集
    double calculateGiniIndex(vector<int> &dataset,int &feature,double &threshold);//计算基尼系数
    int getMaxTimesLabel(map<int,int> &labelCount);//获取出现次数最多的标签
    void findSplit(vector<int> &dataset, vector<int> &features, vector<int> &leftDataset, vector<int> &rightDataset, int &featureId, double &threshold);
    TreeNode *createTree(vector<int> &dataset, vector<int> &features); //创建决策树
    int classifyData(const vector<double> &testData,TreeNode *root) const;//测试数据分类
    void printTree(TreeNode* Node, int depth);//打印决策树
    void saveTree(TreeNode* Node, const char *filename, int depth);//保存决策树
    TreeNode *loadTree(const char *filename, int depth, int *line);//加载决策树
    double testTree(DecisionTree &dt,const vector<vector<double>> &testData,const vector<int> &testLabel);
    TreeNode* findPruningNode(TreeNode *Node);
    void treePruning(DecisionTree &dt,const vector<vector<double>> &verifyData,const vector<int> &verifyLabel);
};


}