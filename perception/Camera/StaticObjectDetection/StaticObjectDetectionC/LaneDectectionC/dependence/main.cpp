#include <dirent.h>
#include <sys/types.h>
#include "../include/ClassicLaneDetect.hpp"

using namespace std;
using namespace cv;

// * parameters below used for offline images test
string iamgePath = "../origin_pictures";

/***
 * @description: get all files name in certain dir path
 * @param [string] path: input path
 * @param [vector<string>] imageNames: all the filenames in the input path
 * @return []
 */
// ! Please make sure all the files in the path are images
void getImagesName(string path, vector<string> &imageNames)
{
    DIR *pDir;
    struct dirent *ptr;
    if (!(pDir = opendir(path.c_str())))
        return;
    while ((ptr = readdir(pDir)) != 0)
    {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            imageNames.push_back(path + "/" + ptr->d_name);
    }
    closedir(pDir);
}

int main()
{
    // Mat img = imread("C:\\Users\\11640\\Desktop\\lane_detect\\origin_pictures\\MVI_1717_004.BMP");
    // cv::imshow("image", img);
    // cv::waitKey();
    vector<string> imageNames;
    getImagesName(iamgePath, imageNames);
    ClassicLaneDetect laneDetect;
    for (string name : imageNames)
    {
        // name = "C:\\Users\\11640\\Desktop\\lane_detect\\origin_pictures\\MVI_1717_283.BMP";
        cout << name << endl;
        laneDetect.image = imread(name);
        laneDetect.run();
    }
    return 0;    
}
