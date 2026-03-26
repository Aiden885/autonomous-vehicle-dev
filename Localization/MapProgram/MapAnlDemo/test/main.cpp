#include"../dependence/localization_MapAnalysis.h"
#include"../src/mapAnalysisAdd.h"

int main() {
	// RoadMap m("roadMap.xodr");
	Empty n0 ;
	Path roadMapPath,osmPath;
	roadMapPath.path = "/home/xwl/learning/算子改写已完成/map_program/MapAnlDemo/test/roadMap.xodr";  //绝对路径
	osmPath.path = "/home/xwl/learning/算子改写已完成/map_program/MapAnlDemo/test/roadMap.osm";       //绝对路径
	RoadM rm;
	RoadMap map(roadMapPath.path);
	rm.m = map;
	OSMFormatStru OSMF;

	osmformatFormatConversion(OSMF,roadMapPath,n0);
	osmformatSaveMapToOSM(OSMF,osmPath,n0);

	return 0;
}