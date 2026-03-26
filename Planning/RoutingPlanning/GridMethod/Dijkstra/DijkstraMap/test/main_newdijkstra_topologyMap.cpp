#include"dijkstraMapFunctional.h"
#include<math.h>
using namespace std;


int main()
{
	// int originRoad = 3;
	// int originLane = 0;
	// int destinationRoad = 6;
	// int destinationLane = 1;
	// vector<pair<int,int>> p;
	// Map m;
	// rmPara para1;
	// rmInput input1{"./roadMap(1).xodr"};
	// rmOutput output1{m};
	Map m;
	mapAnalysisPara para1;
	mapAnalysisInput input1{"./roadMap(1).xodr"};
	mapAnalysisOutput output1{m};
	DijkstraMapAnalysis(para1,input1,output1);
	mapToAstarPara para2;
	mapToAstarInput input2;
	mapToAstarOutput output2;
	input2.m = output1.m;
	DijkstraMapToAstar(para2,input2,output2);
	// initRoadPara para3;
	// initRoadInput input3{6,1,2,3,4,sqrt((1 - 2) * (1 - 2) + (3 - 4) * (3 - 4))};
	// initRoadOutput output3;
	// initRoad(para3,input3,output3);
	// initLinkPara para3;
	// initLinkInput input3{1,3};
	// initLinkOutput output3{output2.A};
	// initLink(para3,input3,output3);
	getPathPara para3;
	getPathInput input3{output2.A,3,6};
	getPathOutput output3;
	DijkstraGetPath(para3,input3,output3);
	findLanePara para4;
	findLaneInput input4{output1.m,output3.path,3,0,6,1};
	findLaneOutput output4{output2.A};
	DijkstraFindLane(para4,input4,output4);
	// for(auto pl:output4.A.pathLanes)
	// {
	// 	cout<<"road:"<<pl.first<<" lane:"<<pl.second<<endl;
	// }
	// moduleSelfCheckPara para4;
	// moduleSelfCheckInput input4{output2.A};
	// moduleSelfCheckOutput output4;
	// moduleSelfCheck(para4,input4,output4);
	moduleSelfCheckPrintPara para5;
	moduleSelfCheckPrintInput input5{output2.A,output2.A.pathLanes};
	moduleSelfCheckPrintOutput output5;
	DijkstraModuleSelfCheckPrint(para5,input5,output5);
	// for(auto p:output3.path)
	// 	cout<<p<<' ';



	// cout<<"from:"<<input3.from<<' '<<"to:";
	// for(auto to:output3.A.roadList[input3.from].to)
	// 	cout<<to<<",";
	// cout<<"xBegin:"<<output3.A.roadList[6].xBegin<<endl
	// <<"xEnd:"<<output3.A.roadList[6].xEnd<<endl
	// <<"yBegin:"<<output3.A.roadList[6].yBegin<<endl
	// <<"yEnd:"<<output3.A.roadList[6].yEnd<<endl
	// <<"length:"<<output3.A.roadList[6].length;
// void initRoad(initRoadPara &para,initRoadInput &input,initRoadOutput &output);

// void initLink(initLinkPara &para,initLinkInput &input,initLinkOutput &output);

// void reset(resetPara &para,resetInput &input,resetOutput &output);

// void moduleSelfCheck(moduleSelfCheckPara &para,moduleSelfCheckInput &input,moduleSelfCheckOutput &output);

// void moduleSelfCheckPrint(moduleSelfCheckPrintPara &para,moduleSelfCheckPrintInput &input,moduleSelfCheckPrintOutput &output);

// void mapToAstar(mapToAstarPara &para,mapToAstarInput &input,mapToAstarOutput &output);

// void findLane(findLanePara &para,findLaneInput &input,findLaneOutput &output);

// void getPath(getPathPara &para,getPathInput &input,getPathOutput &output);
	// mapPrintInput input2{output1.m};
	// mapPrintPara para2;
	// mapPrintOutput output2;
	// moduleSelfCheckPara para3;
	// moduleSelfCheckInput input3;
	// moduleSelfCheckOutput output3;
	// moduleSelfCheck(para3,input3,output3);
	// cout<<"before:"<<endl;
	// cout<<"leftID:"<<endl;
	// for(auto road:output1.m.roads)
	// {
	// 	for(auto lane:road.lanes)
	// 	{
	// 		if(!lane.leftLaneId.empty())
	// 		{
	// 			cout<<"roadID:"<<road.id<<':'<<"laneID:"<<lane.id<<':';
	// 			for(auto id:lane.leftLaneId)
	// 				cout<<id<<' ';
	// 		}
	// 	}
	// }
	// cout<<endl<<"rightID:"<<endl;
	// for(auto road:output1.m.roads)
	// {
	// 	for(auto lane:road.lanes)
	// 	{
	// 		if(!lane.rightLaneId.empty())
	// 		{
	// 			cout<<"roadID:"<<road.id<<':'<<"laneID:"<<lane.id<<':';
	// 			for(auto id:lane.rightLaneId)
	// 				cout<<id<<' ';
	// 		}
	// 	}
	// }	
	// neighborLaneSortPara para3;
	// neighborLaneSortInput input3;
	// neighborLaneSortOutput output3{output1.m};
	// neighborLaneSort(para3,input3,output3);
	// cout<<"after:"<<endl;
	// cout<<"leftID:"<<endl;
	// for(auto road:output1.m.roads)
	// {
	// 	for(auto lane:road.lanes)
	// 	{
	// 		if(!lane.leftLaneId.empty())
	// 		{
	// 			cout<<"roadID:"<<road.id<<':'<<"laneID:"<<lane.id<<':';
	// 			for(auto id:lane.leftLaneId)
	// 				cout<<id<<' ';
	// 		}
	// 	}
	// }
	// cout<<endl<<"rightID:"<<endl;
	// for(auto road:output1.m.roads)
	// {
	// 	for(auto lane:road.lanes)
	// 	{
	// 		if(!lane.rightLaneId.empty())
	// 		{
	// 			cout<<"roadID:"<<road.id<<':'<<"laneID:"<<lane.id<<':';
	// 			for(auto id:lane.rightLaneId)
	// 				cout<<id<<' ';
	// 		}
	// 	}
	// }
	// mapPrint(para2,input2,output2);

	// readMap(para1,input1,output1);

	// dijPara para2;
	// dijInput input2{3,0,6,1};
	// dijOutput output2{p};
	// input2.map.roads = output1.map.roads;
	// getPathDijMap(para2,input2,output2);

	// cout << endl << "* * * 用Astar找到的最短路为 : " << endl;
	// Astar().moduleSelfCheckPrint(output2.path);

}
