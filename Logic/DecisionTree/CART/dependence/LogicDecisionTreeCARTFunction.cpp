#include<iostream>
#include<math.h>
#include<algorithm>
#include<iostream>
#include<fstream>
#include<string>
#include"LogicDecisionTreeCARTHead.hpp"
using namespace std;

namespace Logic{

/*
 * @description: 计算基尼系数值，公式为1-pi^2求和
 * @param dataset: 数据集
 * @return: 基尼系数值
 */

double DecisionTree::calculateGiniValue(vector<int> &dataset){
    map<int,int> labelCountData = labelCount(dataset);
    int len = dataset.size();
    double result = 0;
    for(auto count : labelCountData){
        double pi = count.second / static_cast<double>(len);
        result -= pi * pi;
    }
    return result + 1;
}

/*
 * @description: 计算基尼系数
 * @param dataset: 数据集
 * @param feature: 特征
 * @return: 基尼系数
 */

double DecisionTree::calculateGiniIndex(vector<int> &dataset,int &feature,double &threshold) {
    vector<int> leftDataset, rightDataset;
    leftDataset = splitDataset(dataset, feature, threshold, false);
    rightDataset = splitDataset(dataset, feature, threshold, true);
    double giniIndex = leftDataset.size()/dataset.size()*calculateGiniValue(leftDataset);
    giniIndex += rightDataset.size()/dataset.size()*calculateGiniValue(rightDataset);
    return giniIndex;
}


/*
 * @description: 分类
 * @param testData: 测试数据 
 * @param root: 决策树根节点
 * @return: 返回分类结果
 */
int DecisionTree::classifyData(const vector<double> &testData,TreeNode *root) const{
    //如果决策树节点是叶子节点，直接返回结果
    if(root->isLeaf){
        return root->result;
    }
    //cout << "按Feature" << root->attrId << "分类" << endl;
    if(testData[root->attrId] < root->attrThreshold){
        return classifyData(testData,root->left);
    }
    else{
        return classifyData(testData,root->right);
    }
    return -1;
}

/*
 * @description: 创建决策树
 * @param dataset: 数据集
 * @param features: 特征集 
 * @return: 返回决策树根节点指针
 */
TreeNode* DecisionTree::createTree(vector<int> &dataset,vector<int> &features){
    TreeNode *root = new TreeNode();
    map<int,int> labelCountData = labelCount(dataset);
    //如果特征集为空，则该树为单节点树，类别为标签中出现次数最多的标签
    if(features.size() == 0){
        root->result = getMaxTimesLabel(labelCountData);
        root->isLeaf = true;
        return root;
    }
    //如果数据集中只包含一种标签，则该树为单节点树，类别为该标签
    if(labelCountData.size() == 1){
        root->result = labelCountData.begin()->first;
        root->isLeaf = true;
        return root;
    }

    vector<int> leftDataset, rightDataset;
    int featureId;
    double threshold;
    // data不为空且包含不同标签
    findSplit(dataset, features, leftDataset, rightDataset, featureId, threshold);
    vector<int> subFeatures = features;
    subFeatures.erase(find(subFeatures.begin(),subFeatures.end(),featureId));
    if (leftDataset.size() == 0){
        root->left->isLeaf = true;
        root->left->result = getMaxTimesLabel(labelCountData);
    }
    else
        root->left = createTree(leftDataset,subFeatures);
    if (rightDataset.size() == 0){
        root->right->isLeaf = true;
        root->right->result = getMaxTimesLabel(labelCountData);
    }
    else
        root->right = createTree(rightDataset,subFeatures);
    root->attrId = featureId;
    root->attrThreshold = threshold;
    root->result = getMaxTimesLabel(labelCountData);
    return root;
}
/*
 * @description: 构造函数
 * @param trainData: 训练数据
 * @param trainLabel: 训练数据标签
 * @param threshold: 阈值
 * @return: 没有返回值
 */

DecisionTree::DecisionTree(){}

DecisionTree::DecisionTree(const char *filename, int depth, int line)
{
    int *linePtr = &line;
    decisionTreeRoot = loadTree(filename, depth, linePtr);
}

DecisionTree::DecisionTree(const vector<vector<double>> &trainData,const vector<int> &trainLabel){
    loadData(trainData,trainLabel);//导入数据
    vector<int> dataset(trainData.size()); //数据集
    long long unsigned int i = 0;
    for (i = 0; i < trainData.size(); i++)
    {
        dataset[i] = i;
    }
    vector<int> features(trainData[0].size());//属性集合
    for(i = 0;i < trainData[0].size();i++){
        features[i] = i;
    }
    decisionTreeRoot = createTree(dataset,features);//创建决策树
}

/*
 * @description: 划分数据集
 * @param dataset: 数据集
 * @param features: 特征集
 * @param leftDataset: 左节点数据集
 * @param rightDataset: 右节点数据集
 * @param threshold: 阈值
 * @return: 没有返回值
 */
void DecisionTree::findSplit(vector<int> &dataset, vector<int> &features, vector<int> &leftDataset, vector<int> &rightDataset, int &featureId, double &threshold) {
    double final = 9999;
    for(auto feature:features) {
        for(auto j:dataset) {
            double giniIndex = calculateGiniIndex(dataset, feature, trainData[j][feature]);
            // cout << j << " " << feature << " " << giniIndex << endl;
            if(giniIndex < final) {
                final = giniIndex;
                featureId = feature;
                threshold = trainData[j][feature];
            }
        }
    }
    leftDataset = splitDataset(dataset, featureId, threshold, false);
    rightDataset = splitDataset(dataset, featureId, threshold, true);
}


/*
 * @description: 获取标签统计中出现次数最多的标签
 * @param labelCount: 标签统计
 * @return: 返回出现次数最多的标签名
 */
int DecisionTree::getMaxTimesLabel(map<int,int> &labelCount){
    int maxCount = 0;
    int res = 0;
    for(auto label : labelCount){
        if(maxCount <= label.second){
            maxCount = label.second;
            res = label.first;
        }
    }
    return res;
}

/*
 * @description: 统计数据集中每个标签的数量，比如结果为1的数量和结果为2的数量
 * @param {type} dataset：数据集，是训练数据的子集，整型数组表示，每一个整数表示第几个训练数据
 * @return: 标签名，标签数的map
 */
map<int,int> DecisionTree::labelCount(vector<int> &dataset){
    map<int,int> res;
    //遍历数据集，统计标签出现的次数
    for(int index : dataset){
        res[trainLabel[index]]++;
    }
    return res;
}

/*
 * @description: 导入数据
 * @param trainData：训练数据  
 * @param trainLabel：训练标签
 * @return: 没有返回值
 */
void DecisionTree::loadData(const vector<vector<double>> &trainData,const vector<int> &trainLabel){
    //如果数据特征向量的数量和数据集标签的数量不一样的时候，数据有问题
    if(trainData.size() != trainLabel.size()){
        cerr << "input error" << endl;
        return;
    }
    //初始化
    this->trainData = trainData;
    this->trainLabel = trainLabel;

    //计算featureValues
    long long unsigned int i = 0;
    for(auto data : trainData){
        for(i = 0;i < data.size();++i){
            featureValues[i].insert(data[i]);
        }
    }
}


/*
 * @description: 打印决策树
 * @param Node: 起始节点
 * @param depth: 打印缩进
 * @return: 没有返回值
 */
void DecisionTree::printTree(TreeNode* Node, int depth){
    int i;
    //不是叶节点，打印分割的特征
	if (!Node->isLeaf) {
        for (i = 0; i < depth; i++) 
            cout << '\t';  
		cout << "Feature: " << Node->attrId << endl;
        for (i = 0; i < depth; i++)
            cout << '\t';
        cout << "Threshold: " << Node->attrThreshold << endl; 
	}
    //叶节点，打印分类结果
    else {
        for (i = 0; i < depth; i++) 
            cout << '\t';
        cout << "Result: " << Node->result << endl;
        return;
    }
    //递归
    printTree(Node->left, depth+1);
    printTree(Node->right, depth+1);
    return;
}

/*
 * @description: 根据特征名来划分子集
 * @param dataset: 数据集
 * @param feature: 第几个特征
 * @param value: 第几个特征的特征值
 * @return: 返回划分的子集
 */    
vector<int> DecisionTree::splitDataset(vector<int> &dataset,int &feature,double &threshold, bool bigger){
    vector<int> res;
    for(int index : dataset){
        if (bigger){
            if(trainData[index][feature] >= threshold){
                res.push_back(index);
            }
        }
        else {
            if(trainData[index][feature] < threshold){
                res.push_back(index);
            }
        }
    }
    return res;
}

/*
 * @description: 保存决策树到文件
 * @param Node: 决策树根节点
 * @param filename: 文件名
 * @param depth: 文件缩进值，应与load时保持一致
 * @return: 没有返回值
 */  
void DecisionTree::saveTree(TreeNode *Node, const char *filename, int depth = 0){
    int i;
    ofstream out; 
    out.open(filename, ios::app);
    //不是叶节点，打印分割的特征
	if (!Node->isLeaf) {
        for (i = 0; i < depth; i++) 
            out << '\t';  
		out << "Feature: " << Node->attrId << endl;
        for (i = 0; i < depth; i++)
            out << '\t';
        out << "Threshold: " << Node->attrThreshold << endl; 
	}
    //叶节点，打印分类结果
    else {
        for (i = 0; i < depth; i++) 
            out << '\t';
        out << "Result: " << Node->result << endl;
        return;
    }
    //递归
    saveTree(Node->left, filename, depth+1);
    saveTree(Node->right, filename, depth+1);
    if(!depth)
        out.close();
    return;
}

/*
 * @description: 读取文件，加载决策树
 * @param filename: 文件名
 * @param depth: 文件缩进值，应与save时保持一致
 * @param line: 读取文件起始行数（因为该函数通过迭代完成，而重新打开文件会定位到文件开头）
 * @return: 根节点指针
 */  
TreeNode *DecisionTree::loadTree(const char *filename, int depth, int *line){
    //对于Feature，处理两行后进入迭代；对于Result，返回叶节点指针
    int i;
    TreeNode *Node = new TreeNode;
    char c[100];
    string s0, s1, s2;

    //打开文件，忽略前line行的信息
    ifstream in;   
    in.open(filename, ios::in);
    for (i = 0; i <= *line; i++)
        in.getline(c, sizeof(c));
    s0.assign(c);
    s1 = s0.substr(depth);//截去tab
    if (s1[0] == 'F')//Feature
    {
        s2 = s1.substr(9);   
        Node->isLeaf = false;
        Node->attrId = atoi(s2.c_str());
        in.getline(c, sizeof(c));
        s0.assign(c);
        s1 = s0.substr(depth);
        if (s1[0] == 'T'){
            s2 = s1.substr(11);
            Node->attrThreshold = (double)atoi(s2.c_str());
            *line += 2;
        }

    }
    else if (s1[0] == 'R')//Result
    {
        Node->isLeaf = true;
        s2 = s1.substr(8);
        Node->result = atoi(s2.c_str());
        *line += 1;
        return Node;
    }
    //Threshold，处理子节点和迭代
    while (in.getline(c,sizeof(c))) {
        s0.assign(c);
        s1 = s0.substr(depth + 1);
        if (s1[0] == 'F' || s1[0] == 'R'){
            TreeNode *leftBranch = new TreeNode();
            leftBranch = loadTree(filename, depth+1, line);
            Node->left = leftBranch;
            break;
        }
	}

    while (in.getline(c,sizeof(c))) {
        s0.assign(c);
        s1 = s0.substr(depth + 1);
        if (s1[0] == 'F' || s1[0] == 'R'){
            TreeNode *rightBranch = new TreeNode();
            rightBranch = loadTree(filename, depth+1, line);
            Node->right = rightBranch;
            break;
        }
	}
    in.close();
    return Node;
}

/*
 * @description: 测试决策树
 * @param dt: 决策树
 * @param testData: 测试数据
 * @param testLabel: 测试数据标签真值
 * @return: 准确率
 */  
double DecisionTree::testTree(DecisionTree &dt,const vector<vector<double>> &testData,const vector<int> &testLabel){
    if (testData.size() != testLabel.size()){
        cout << "input error" << endl;
        return 0;
    }
    int rightTest = 0;
    long long unsigned int i = 0;
    for (i = 0; i < testData.size(); i++){
        if (dt.classifyData(testData[i], dt.decisionTreeRoot) == testLabel[i]){
            rightTest++;
        }
    }
    return static_cast<double> (rightTest) / static_cast<double> (testData.size());
}

/*
 * @description: 寻找剪枝节点
 * @param Node: 决策树根节点指针
 * @return: 剪枝节点指针
 */  
TreeNode* DecisionTree::findPruningNode(TreeNode *Node){
    if (Node->isPruning)
        return 0;
    vector<TreeNode*> branches;
    branches.push_back(Node->left);
    branches.push_back(Node->right);
    for (auto branch:branches){
        if (branch->isPruning){
            Node->isPruning = true;
            return 0;
        }
    }
    bool flag = true;
    for (auto branch:branches){
        if (!branch->isLeaf){
            flag = false;
            TreeNode *tempNode = new TreeNode();
            tempNode = findPruningNode(branch);
            if (tempNode != 0)
                return tempNode;
        }
    }
    if (flag)
        return Node;
    else
        return 0;
}

/*
 * @description: 决策树剪枝
 * @param dt: 决策树
 * @param verifyData: 验证数据
 * @param verifyLabel: 验证数据标签
 * @return: 没有返回
 */  
void DecisionTree::treePruning(DecisionTree &dt,const vector<vector<double>> &verifyData,const vector<int> &verifyLabel){
    while (true){
        TreeNode *tempNode = new TreeNode();
        tempNode = findPruningNode(dt.decisionTreeRoot);
        if (tempNode == 0)
            break;
        double rate0, rate1;
        rate0 = testTree(dt, verifyData, verifyLabel);
        tempNode->isLeaf = true;
        rate1 = testTree(dt, verifyData, verifyLabel);
        if (rate1 < rate0){
            tempNode->isLeaf = false;
            tempNode->isPruning = true;
        }
    }
}

}