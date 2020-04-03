#ifndef SEM2VEC_CLOUD_IO_H
#define SEM2VEC_CLOUD_IO_H

#include "core/defines.h"
#include <map>

namespace sem2vec
{
template <typename PointT>
class CloudIO
{
public:
    typedef typename pcl::PointCloud<PointT> PointTCloud;
    typedef typename PointTCloud::Ptr PointTCloudPtr;

    CloudIO();

    bool writeCloud(const PointTCloudPtr& cloud, const std::string& file_name);

    bool writeCloud(const PointTCloudPtr& cloud, const std::string& file_name, const std::string& format);

    bool writeClouds(const std::vector<PointTCloudPtr>& clouds, const std::string& out_dir, const std::string& format,
                     const std::string& fname="");

    int readCloud(PointTCloudPtr& cloud, const std::string& file_name);

    int readCloud(PointTCloudPtr& cloud, const std::string& file_name, const std::string& format);

private:
    std::map<std::string, int32_t> format_map_;
};
}

#endif
