#ifndef SEM2VEC_DBSCAN_HPP
#define SEM2VEC_DBSCAN_HPP

#include "cloud_cluster/dbscan.h"
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <boost/make_shared.hpp>
#include <glog/logging.h>
#include <stack>

namespace sem2vec
{
template <typename PointT>
DBSCAN<PointT>::DBSCAN(const sem2vec::float32_t eps, const uint32_t min_samples) :
                       eps_(eps), min_samples_(min_samples), num_clusters_(0)
{
    cloud_ = boost::make_shared<PointTCloud>(*(new PointTCloud));
    corecloud_ = boost::make_shared<PointTCloud>(*(new PointTCloud));
}

template <typename PointT>
DBSCAN<PointT>::point::point()
{
    visited = 0;
    pointtype = 1;
    cluster = 0;
}

template <typename PointT>
DBSCAN<PointT>::point::point(float a, float b, float c)
{
    x = a;
    y = b;
    z = c;
    visited = 0;
    pointtype = 1;
    cluster = 0;
}

template <typename PointT>
void DBSCAN<PointT>::setCloud(const PointTCloudPtr& cloud)
{
    assert(cloud != NULL);
    for(uint32_t i = 0; i < cloud->points.size(); i++)
    {
        point pt = point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        allcloud_.push_back(pt);
        cloud_->points.push_back(cloud->points[i]);
    }
}

template <typename PointT>
void DBSCAN<PointT>::searchCorePoints()
{
    assert(!cloud_->points.empty());
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud_);
    for (uint32_t i = 0; i < allcloud_.size(); i++)
    {
        if(i + 1 % 100000 == 0)
        {
            LOG(INFO) << "Searching " << i + 1 << " / " << allcloud_.size();
        }
        std::vector<int> index;
        std::vector<float> dist;
        kdtree.radiusSearch(cloud_->points[i], eps_, index, dist);
        if (index.size() > min_samples_)
        {
            corecloud_->points.push_back(cloud_->points[i]);
            allcloud_[i].pointtype = 3;
            for (int j = 0; j < index.size(); j++)
            {
                allcloud_[i].corepts.push_back(index[j]);
            }
        }
    }
    std::cout << "Num of corepoints: " << corecloud_->points.size() << std::endl;
}

template <typename PointT>
void DBSCAN<PointT>::mergeCorePoints()
{
    assert(!cloud_->points.empty());
    for (int i = 0; i < allcloud_.size(); i++)
    {
        if (allcloud_[i].pointtype == 3)
        {
            std::stack<point*> ps;
            if (allcloud_[i].visited != 1)
            {
                allcloud_[i].cluster = ++num_clusters_;
                ps.push(&allcloud_[i]);
                point* v;
                while (!ps.empty())
                {
                    v = ps.top();
                    v->visited = 1;
                    ps.pop();
                    for (int j = 0; j < v->corepts.size(); j++)
                    {
                        if (allcloud_[v->corepts[j]].visited != 1)
                        {
                            allcloud_[v->corepts[j]].visited = 1;
                            ps.push(&allcloud_[v->corepts[j]]);
                            allcloud_[v->corepts[j]].cluster = allcloud_[i].cluster;
                        }
                    }
                }
            }
        }
    }
    std::cout << "Num of clusters: " << num_clusters_ << std::endl;
}

template <typename PointT>
void DBSCAN<PointT>::assignCluster()
{
    corecloud_->points.clear();
    for (int i = 0; i < allcloud_.size(); i++)
    {
        if(allcloud_[i].pointtype == 3)
        {
            PointT pt;
            pt.x = allcloud_[i].x;
            pt.y = allcloud_[i].y;
            pt.z = allcloud_[i].z;
            pt.cluster = allcloud_[i].cluster;
            corecloud_->points.push_back(pt);
        }
    }
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(corecloud_);
    for (int i = 0; i < allcloud_.size(); i++)
    {
        if (allcloud_[i].pointtype == 3)
        {
            continue;
        }
        else if (allcloud_[i].pointtype == 1)
        {
            allcloud_[i].cluster = 0;
        }
        else
        {
            std::vector<int> index(1);
            std::vector<float> dist(1);
            PointT search_point;
            search_point.x = allcloud_[i].x;
            search_point.y = allcloud_[i].y;
            search_point.z = allcloud_[i].z;
            kdtree.nearestKSearch(search_point, 1, index, dist);
            allcloud_[i].cluster = corecloud_->points[index[0]].cluster;
        }
    }
}

template <typename PointT>
bool DBSCAN<PointT>::cluster(PointTCloudPtr & output)
{
    if(allcloud_.empty())
    {
        LOG(ERROR) << "ERROR: no input data, please check...";
        return false;
    }
    searchCorePoints();
    mergeCorePoints();
    assignCluster();
    if(output == NULL)
    {
        output = boost::make_shared<PointTCloud>(*(new PointTCloud));
    }
    for (int i = 0; i < allcloud_.size(); i++)
    {
        if (allcloud_[i].cluster > 0)
        {
            PointT pt;
            pt.x = allcloud_[i].x;
            pt.y = allcloud_[i].y;
            pt.z = allcloud_[i].z;
            pt.cluster = allcloud_[i].cluster;
            output->points.push_back(pt);
        }
    }
    return true;
}
}

#endif
