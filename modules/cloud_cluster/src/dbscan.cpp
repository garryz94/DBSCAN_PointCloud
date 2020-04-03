#include "cloud_cluster/impl/dbscan.hpp"

#define INSTANTIATE(PointT) \
template class DBSCAN<PointT>; \

namespace sem2vec
{
INSTANTIATE(PointTypeCluster);
}

#undef INSTANTIATE