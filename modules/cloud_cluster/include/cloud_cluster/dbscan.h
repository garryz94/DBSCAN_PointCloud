#ifndef SEM2VEC_DBSCAN_H
#define SEM2VEC_DBSCAN_H

#include "core/defines.h"

namespace sem2vec
{
template <typename PointT>
class DBSCAN
{
public:
    typedef typename pcl::PointCloud<PointT> PointTCloud;
    typedef typename PointTCloud::Ptr PointTCloudPtr;

    struct point
    {
        float x;
        float y;
        float z;
        int visited;
        int pointtype; //1 noise_point, 2 boundary_point, 3 core_point
        int cluster;
        std::vector<int> corepts; //save indexes in the neighborhood
        point();

        point(float a, float b, float c);
    };

    DBSCAN(const float32_t eps, const uint32_t min_samples);

    inline void reInit(const float32_t eps, const uint32_t min_samples)
    {
        assert(eps > 0);
        eps_ = eps;
        min_samples_ = min_samples;
        num_clusters_ = 0;
        allcloud_.clear();
        cloud_->points.clear();
        corecloud_->points.clear();
    }

    void setCloud(const PointTCloudPtr& cloud);

    void searchCorePoints();

    void mergeCorePoints();

    void assignCluster();

    bool cluster(PointTCloudPtr& output);

private:
    float32_t eps_;
    uint32_t min_samples_;
    uint32_t num_clusters_;
    PointTCloudPtr cloud_;
    PointTCloudPtr corecloud_;
    std::vector<point> allcloud_;
};
}

#endif
