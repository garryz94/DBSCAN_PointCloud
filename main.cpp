#include "cloud_io/impl/cloud_io.hpp"
#include "cloud_cluster/impl/dbscan.hpp"

using namespace sem2vec;

typedef PointTypeCluster PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

int main()
{
    std::string in_file = "/home/ubuntu/Desktop/pole.ply";
    std::string out_file = "/home/ubuntu/Desktop/pole_dbscan2.ply";

    CloudIO<PointType> cloud_io;

    PointCloudPtr input(new PointCloud);
    PointCloudPtr cloud(new PointCloud);
    PointCloudPtr output(new PointCloud);

    LOG(INFO) << "load cloud...";
    if(cloud_io.readCloud(input, in_file) == -1)
    {
        LOG(ERROR) << "ERROR: readCloud " << in_file;
        return -1;
    }

    for(int i = 0; i < input->points.size(); i++)
    {
        if(input->points[i].z < 1)
        {
            cloud->points.push_back(input->points[i]);
        }
    }

    LOG(INFO) << "clustering...";
    float32_t eps = 0.5;
    int32_t min_samples = 100;
    DBSCAN<PointType> dbscan(eps, min_samples);
    dbscan.setCloud(cloud);
    if(!dbscan.cluster(output))
    {
        LOG(ERROR) << "ERROR: cluster";
        return -1;
    }

    LOG(INFO) << "save cloud...";
    if(!cloud_io.writeCloud(output, out_file))
    {
        LOG(ERROR) << "writeCloud " << out_file;
        return -1;
    }

    return 0;
}
