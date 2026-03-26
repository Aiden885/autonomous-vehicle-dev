#ifndef PTCLOUDCOMMUNICATOR_H
#define PTCLOUDCOMMUNICATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "CommunicatorBase.h"

using namespace std;

class PtCloudCommunicator : public CommunicatorBase
{
private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr;

public:
    PtCloudCommunicator();

    void publishCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr_);

    virtual void subscribe() override;

    inline pcl::PointCloud<pcl::PointXYZI>::Ptr getPointCloud() { return cloud_ptr; };

    ~PtCloudCommunicator();
};

#endif /*PTCLOUDCOMMUNICATOR_H*/
