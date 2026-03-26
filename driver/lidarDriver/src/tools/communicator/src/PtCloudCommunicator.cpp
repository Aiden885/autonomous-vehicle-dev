#include <zmq.h>

#include "PtCloudCommunicator.h"
#include "cloud_data.pb.h"

PtCloudCommunicator::PtCloudCommunicator() : cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>)
{
}

PtCloudCommunicator::~PtCloudCommunicator()
{
}

void PtCloudCommunicator::publishCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr_)
{
    if (pub_port > 0)
    {
        CloudProto::CloudData cloudData;

        cloudData.set_height(cloud_ptr_->height);
        cloudData.set_width(cloud_ptr_->width);
        cloudData.set_isdense(cloud_ptr_->is_dense);

        int dataSize = cloud_ptr_->height * cloud_ptr_->width * sizeof(pcl::PointXYZI);

        cloudData.set_pointsdata(cloud_ptr_->points.data(), dataSize);

        size_t size = cloudData.ByteSize();
        void *buffer = malloc(size);
        cloudData.SerializePartialToArray(buffer, size);

        zmq_msg_t msg;
        zmq_msg_init_size(&msg, size);
        memcpy(zmq_msg_data(&msg), buffer, size);
        zmq_msg_send(&msg, publisher, 0);
    }
    else
    {
        cerr << "you must call InitPublisher function first!" << endl;
    }
}

void PtCloudCommunicator::subscribe()
{
    if (sub_port > 0)
    {
        zmq_msg_t msg;
        zmq_msg_init(&msg);
        int size = zmq_msg_recv(&msg, subscriber, 0);

        if (size == -1)
        {
            return;
        }

        void *recvStr = malloc(size);
        memcpy(recvStr, zmq_msg_data(&msg), size);

        CloudProto::CloudData cloudData;

        cloudData.ParseFromArray(recvStr, size);

        cloud_ptr->width = cloudData.width();
        cloud_ptr->height = cloudData.height();
        cloud_ptr->is_dense = cloudData.isdense();
        cloud_ptr->resize(cloudData.width() * cloudData.height());

        size_t dataSize = cloudData.width() * cloudData.height() * sizeof(pcl::PointXYZI);

        memcpy(cloud_ptr->points.data(), cloudData.pointsdata().data(), dataSize);
    }
    else
    {
        cerr << "you must call InitSubscriber function first!" << endl;
    }
}
