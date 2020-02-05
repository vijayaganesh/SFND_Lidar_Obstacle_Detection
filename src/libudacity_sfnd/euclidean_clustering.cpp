#include "libudacity_sfnd/euclidean_clustering.h"

template <typename PointT>
sfnd::EuclideanClustering<PointT>::EuclideanClustering() : cloud_(new typename pcl::PointCloud<PointT>),
                                                           kdtree_(new KDTree<PointT>)
{
}

template <typename PointT>
sfnd::EuclideanClustering<PointT>::EuclideanClustering(typename pcl::PointCloud<PointT>::Ptr &cloud,
                                                       int min_points,
                                                       int max_points,
                                                       float distance_tolerance) : cloud_(cloud),
                                                                                   min_points_(min_points),
                                                                                   max_points_(max_points),
                                                                                   distance_tolerance_(distance_tolerance)
{
    kdtree_ = sfnd::KDTree<PointT>::pointCloudToKDTree(cloud_);
}

template <typename PointT>
std::unique_ptr<std::vector<typename pcl::PointCloud<PointT>::Ptr>> sfnd::EuclideanClustering<PointT>::getClusterClouds()
{

    std::vector<bool> visited(cloud_->points.size(), false);

    std::unique_ptr<std::vector<typename pcl::PointCloud<PointT>::Ptr>> clusters(new std::vector<typename pcl::PointCloud<PointT>::Ptr>);

    for (int i = 0; i < cloud_->points.size(); i++)
    {
        if (!visited[i])
        {
            typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);

            clusterHelper_(cluster, i, visited);

            if (cluster->points.size() >= min_points_ && cluster->points.size() <= max_points_)
                clusters->push_back(cluster);
        }
    }

    return clusters;
}

template <typename PointT>
void sfnd::EuclideanClustering<PointT>::clusterHelper_(typename pcl::PointCloud<PointT>::Ptr &cluster,
                                                       int index,
                                                       std::vector<bool> &visited)
{
    visited[index] = true;
    cluster->points.push_back(cloud_->points[index]);

    auto nearby_points = kdtree_->search(cloud_->points[index], distance_tolerance_);
    for (auto nearby_point : nearby_points)
    {
        if (!visited[nearby_point])
        {
            clusterHelper_(cluster, nearby_point, visited);
        }
    }
}

template class sfnd::EuclideanClustering<pcl::PointXYZ>;
template class sfnd::EuclideanClustering<pcl::PointXYZI>;