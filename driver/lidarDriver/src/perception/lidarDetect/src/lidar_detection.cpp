/* Naming rule
file: file_file.txt
var: int var_a, float g_index
class: MyClass
func: OpenFile
*/
#include "lidar_process.cpp"

#include <pcl/point_cloud.h>

#include <zmq.h>

using namespace std;

int main(int argc, char **argv)
{
    // void *context = zmq_ctx_new();
    // void *subPointCloud = zmq_socket(context, ZMQ_SUB);
    // zmq_connect(subPointCloud, "tcp://127.0.0.1:5005");
    // zmq_setsockopt(subPointCloud, ZMQ_SUBSCRIBE, "", 0);

    // void *publishObjects = zmq_socket(context, ZMQ_PUB);
    // zmq_bind(publishObjects, "tcp://127.0.0.1:5008");

    PointCloudsDetector<pcl::PointXYZI> pcl_detector;

    pcl_detector.InitView();
    pcl_detector.run();

    return 0;
}