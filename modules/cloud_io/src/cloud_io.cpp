#include "cloud_io/impl/cloud_io.hpp"

#define INSTANTIATE(PointT) \
template class CloudIO<PointT>; \

namespace sem2vec
{
INSTANTIATE(PointTypeCluster);
}

#undef INSTANTIATE
