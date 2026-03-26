#include"dijLatticeFunctional.h"
using namespace std;
int main()
{
    rmPara para1;

    rmInput input1{"./test/map5.jpg"};

    rmOutput output1;

    dijPara para2;

    dijInput input2;

    dijOutput output2;

    sePara para3;

    seInput input3;

    seOutput output3;

    pmPara para4;

    pmInput input4;
    
    pmOutput output4;

    DijkstraReadMap(para1,input1,output1);
    input3.img = imread("./test/map5.jpg");
    // InitDijPara para5;
    // InitDijInput input5{output1.map,output1.hvalue};
    // InitDijOutput output5;
    // InitDij(para5,input5,output5);
    DijkstraGetStartEnd(para3,input3,output3);
    input4.img = &(input3.img);
    input4.start = &(output3.start);
    input4.end = &(output3.end);
    DijkstraPrintMap(para4,input4,output4);
    input2.map = output1.map;

    input2.hvalue = output1.hvalue;
    input2.start = output3.start;
    input2.end = output3.end;

    DijkstraGetPathDijMap(para2,input2,output2);
    input4.path = &(output2.path);
    DijkstraPrintMap(para4,input4,output4);
}
