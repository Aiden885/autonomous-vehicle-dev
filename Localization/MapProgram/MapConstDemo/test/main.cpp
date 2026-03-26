#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "localization_MapConstruction.h"
#include "localizationDeclaration.h"

int main() {

	MapCon map ;
	Empty n0 ;
	Path roadPointPath,roadSuccessorPath,laneNeighborPath,
		laneSuccessorPath,laneChangePath,saveMapPath;
	roadPointPath.path = "roadpoint";
	roadSuccessorPath.path = "road_successor.txt";
	laneNeighborPath.path = "lane_neighbor.txt";
	laneSuccessorPath.path = "lane_successor.txt";
	laneChangePath.path = "lane_change.txt";
	saveMapPath.path = "roadMap.xodr";

	mapconstLoadRawRoadPoint(map,roadPointPath,n0);
	mapconstProcessRawRoadPoint(map,n0,n0);
	mapconstLoadSuccessor(map,roadSuccessorPath,n0);
	mapconstLoadLaneNeighborId(map,laneNeighborPath,n0);
	mapconstLoadLaneSuccessorId(map,laneSuccessorPath,n0);
	mapconstLoadLaneChange(map,laneChangePath,n0);
	mapconstSaveMapToXML(map,saveMapPath,n0);

	//Lane成员函数测试用例
	// LaneStru la;
	// IntStru id;
	// id.i = 2;
	// laneUpdateId(la, id, n0);
	// laneGetGaussRoadPoint(la, n0, n0);
	// laneGetYawCruvature(la, n0, n0);
	// laneCorectGaussXY(la, n0, n0);
	// laneCalcSpeedProfile(la, n0, n0);

	//Road成员函数测试用例
	// RoadStru ro;
	// roadUpdateId(ro, id, n0);

	//MapConstruction成员函数测试用例
	// mapconstGetAllFileNames(map, roadPointPath, n0);
	// mapconstGetXMLInfo(map, n0, n0);
	// mapconstModuleSelfCheck(map, n0, n0);
	// mapconstModuleSelfCheckPrint(map, n0, n0);
	
}