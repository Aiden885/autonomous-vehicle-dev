#include<iostream>
#include<math.h>
#include<algorithm>
#include<iostream>
#include<fstream>
#include<string>
#include"./LogicDecisionTreeHead.hpp"
using namespace std;

namespace Logic{
/*
 * @description: 构造函数
 * @param trainData: 训练数据
 * @param trainLabel: 训练数据标签
 * @param rateFlag：ID3为false，C4.5为true
 * @return: 没有返回值
 */
DecisionTree::DecisionTree(){}

DecisionTree::DecisionTree(const char *filename, int depth, int line)
{
    int *linePtr = &line;
    decisionTreeRoot = loadTree(filename, depth, linePtr);
}

DecisionTree::DecisionTree(const vector<vector<int>> &trainData,const vector<int> &trainLabel,const bool rateFlag){
    loadData(trainData,trainLabel);//导入数据
    vector<int> dataset(trainData.size()); //数据集
    long long unsigned int i = 0;
    for (i = 0; i < trainData.size(); i++){
        dataset[i] = i;
    }
    vector<int> features(trainData[0].size());//属性集合
    for(i = 0;i < trainData[0].size();i++){
        features[i] = i;
        cout << features[i] << endl;
    }
    decisionTreeRoot = createTree(dataset,features,false);//创建决策树
}
/*
 * @description: 计算信息熵，公式为-pi * log(pi)求和
 * @param dataset: 数据集
 * @return: 信息熵
 */

double DecisionTree::calculateEntropy(vector<int> &dataset){
    map<int,int> labelCountData = labelCount(dataset);
    int len = dataset.size();
    double result = 0;
    for(auto count : labelCountData){
        double pi = count.second / static_cast<double>(len);
        result -= pi * log2(pi);
    }
    return result;
}

/*
 * @description: 计算信息增益
 * @param dataset: 数据集
 * @param feature: 对应特征
 * @param rateFlag: true返回信息增益率，false返回信息增益
 * @return: 分类信息（信息增益、信息增益率）
 */
double DecisionTree::calculateGainOrRate(vector<int> &dataset,int &feature,bool rateFlag){
    set<int> values = featureValues[feature];
    double entropy = 0;
    double intrinsic = 0;
    double proportion = 0;
    for(int value : values){
        vector<int> subDataset = splitDataset(dataset,feature,value);
        proportion = subDataset.size() / static_cast<double>(dataset.size());
        entropy += proportion * calculateEntropy(subDataset);
        intrinsic -= proportion * log2(proportion);
    }
    if (rateFlag)
        return (calculateEntropy(dataset) - entropy)/intrinsic;
    else
        return calculateEntropy(dataset) - entropy;
}

/*
 * @description: 分类
 * @param testData: 测试数据 
 * @param root: 决策树根节点
 * @return: 返回分类结果
 */
int DecisionTree::classifyData(const vector<int> &testData,TreeNode *root) const{
    //如果决策树节点是叶子节点，直接返回结果
    if(root->isLeaf){
        return root->result;
    }
    //cout << "按Feature" << root->attrId << "分类" << endl;
    for(auto node : root->branches){
        //找到分支，并在分支中再细分
        if(testData[root->attrId] == node->attrValue){
            return classifyData(testData,node);
        }
    }
    cout << "分类失败" << endl;
    return -1;
}

/*
 * @description: 创建决策树，默认为ID3算法，修改rateFlag更改算法
 * @param dataset: 数据集
 * @param features: 特征集 
 * @return: 返回决策树根节点指针
 */
TreeNode* DecisionTree::createTree(vector<int> &dataset,vector<int> &features,bool rateFlag){
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

    //计算特征集中每个特征的分类信息
    map<int,double> results;
    for (auto feature:features)
    {
        results[feature] = calculateGainOrRate(dataset,feature,rateFlag);
    }

    //获取最大分类信息的特征和最大的分类信息
    int maxResultFeature = getMaxResultFeature(results);
    vector<int> subFeatures = features;
    subFeatures.erase(find(subFeatures.begin(),subFeatures.end(),maxResultFeature));
    for(int value : featureValues[maxResultFeature]){
        TreeNode *branch = new TreeNode();//创建分支
        vector<int> subDataset = splitDataset(dataset,maxResultFeature,value);
               
        //如果子集为空，将分支节点标记为叶节点，类别为标签中出现次数最多的标签
        if(subDataset.size() == 0){
            branch->isLeaf = true;
            branch->result = getMaxTimesLabel(labelCountData);
            root->attrId = maxResultFeature;
            branch->attrValue = value;
            root->branches.push_back(branch);
        }
        //否则递归创建树
        else{
            branch = createTree(subDataset,subFeatures,false);
            root->attrId = maxResultFeature;
            branch->attrValue = value;
            root->branches.push_back(branch);
            root->result = getMaxTimesLabel(labelCountData);
        }
    }
    return root;
}



/*
 * @description: 获取特征集信息增益中最大的分类信息（信息增益、信息增益率）和所对应的特征
 * @param gains: 特征集的信息增益
 * @return: 最大分类信息对应的特征
 */
int DecisionTree::getMaxResultFeature(map<int,double> &results){
    double maxResult = 0;
    int maxResultFeature = 0;
    for(auto result : results){
        if(maxResult <= result.second){
            maxResult = result.second;
            maxResultFeature = result.first;
        }
    }
    return maxResultFeature;
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
void DecisionTree::loadData(const vector<vector<int>> &trainData,const vector<int> &trainLabel){
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
	}
    //叶节点，打印分类结果
    else {
        for (i = 0; i < depth; i++) 
            cout << '\t';
        cout << "Result: " << Node->result << endl;
        return;
    }
    //打印子节点对应的特征值，然后递归
	for (auto it:Node->branches) {
        for (i = 0; i < depth + 1; i++)
            cout << '\t'; 
        cout << "Value: " << it->attrValue << endl;
		printTree(it, depth+1);
	}
}

/*
 * @description: 根据特征名来划分子集
 * @param dataset: 数据集
 * @param feature: 第几个特征
 * @param value: 第几个特征的特征值
 * @return: 返回划分的子集
 */    
vector<int> DecisionTree::splitDataset(vector<int> &dataset,int &feature,int &value){
    vector<int> res;
    for(int index : dataset){
        if(trainData[index][feature] == value){
            res.push_back(index);
        }
    }
    return res;
}

/*
 * @description: 保存决策树到文件
 * @param Node: 决策树根节点
 * @param filename: 文件名
 * @param depth: 文件缩进值，应与load时保持一致
 * @return: 无
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
	}
    //叶节点，打印分类结果
    else {
        for (i = 0; i < depth; i++) 
            out << '\t';
        out << "Result: " << Node->result << endl;
        return;
    }
    //打印子节点对应的特征值，然后递归
	for (auto it:Node->branches) {
        for (i = 0; i < depth + 1; i++)
            out << '\t'; 
        out << "Value: " << it->attrValue << endl;
		saveTree(it, filename, depth+1);
	}
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
    //文件奇数行为Feature或Result，偶数行为Value
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
    }
    else if (s1[0] == 'R')//Result
    {
        Node->isLeaf = true;
        s2 = s1.substr(8);
        Node->result = atoi(s2.c_str());
        return Node;
    }
    //Value，处理子节点和迭代
	while (in.getline(c,sizeof(c))) {
        s0.assign(c);
        s1 = s0.substr(depth + 1);
        if (s1[0] == 'V'){
            *line += 2;
            TreeNode *branch = new TreeNode();
            branch = loadTree(filename, depth+1, line);
            s2 = s1.substr(7);
            branch->attrValue = atoi(s2.c_str());
            Node->branches.push_back(branch);
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
double DecisionTree::testTree(DecisionTree &dt,const vector<vector<int>> &testData,const vector<int> &testLabel){
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
    cout << "tingzhi ";
}

/*
 * @description: 寻找剪枝节点
 * @param Node: 决策树根节点指针
 * @return: 剪枝节点指针
 */  
TreeNode* DecisionTree::findPruningNode(TreeNode *Node){
    if (Node->isPruning)
        return 0;
    for (auto branch:Node->branches){
        if (branch->isPruning){
            Node->isPruning = true;
            return 0;
        }
    }
    bool flag = true;
    for (auto branch:Node->branches){
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
void DecisionTree::treePruning(DecisionTree &dt,const vector<vector<int>> &verifyData,const vector<int> &verifyLabel){
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