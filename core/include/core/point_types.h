#ifndef SEM2VEC_POINT_TYPES_H
#define SEM2VEC_POINT_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace sem2vec
{
struct PointTypeCluster
{
    PCL_ADD_POINT4D;
    int cluster;
} EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT (sem2vec::PointTypeCluster,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (int, cluster, cluster)
)


#endif
