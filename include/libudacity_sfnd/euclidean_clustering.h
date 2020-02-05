#ifndef LIBUDACITY_EUCLIDEAN_CLUSTERING_H_
#define LIBUDACITY_EUCLIDEAN_CLUSTERING_H_

#include "libudacity_sfnd/kdtree.h"

namespace sfnd
{

template <typename PointT>
class EuclideanClustering
{
public:
    EuclideanClustering();

    EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr &cloud, int min_points, int max_points, float distance_tolerance);

    std::unique_ptr<std::vector<typename pcl::PointCloud<PointT>::Ptr>> getClusterClouds();

private:
    int min_points_;
    int max_points_;
    float distance_tolerance_;
    typename pcl::PointCloud<PointT>::Ptr cloud_;
    std::shared_ptr<KDTree<PointT>> kdtree_;

    void clusterHelper_(typename pcl::PointCloud<PointT>::Ptr &cluster, int index, std::vector<bool> &visited);
};
} // namespace sfnd

#endif