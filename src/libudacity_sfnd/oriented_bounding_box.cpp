#include "libudacity_sfnd/oriented_bounding_box.h"

sfnd::OBBox::OBBox()
{
}

sfnd::OBBox::OBBox(Eigen::Vector3f translation_, Eigen::Quaternionf rotation_, float x_size_, float y_size_, float z_size_)
{
}

template <typename PointT>
sfnd::OrientedBoundingBox<PointT>::OrientedBoundingBox() : cloud_(new pcl::PointCloud<PointT>::Ptr)
{
}

template <typename PointT>
sfnd::OrientedBoundingBox<PointT>::OrientedBoundingBox(typename pcl::PointCloud<PointT>::Ptr cloud) : cloud_(cloud)
{
}

template <typename PointT>
sfnd::OBBox sfnd::OrientedBoundingBox<PointT>::findBoundingBox()
{
    // Based on the implementation here    
    // https://stackoverflow.com/a/21652622

    Eigen::Matrix3f covariance_matrix;

    Eigen::Vector3f centroid;

    pcl::compute3DCentroid(cloud_, centroid);

    pcl::computeCovarianceMatrix(cloud_, centroid, covariance_matrix);

    Eigen::EigenSolver<Eigen::Matrix3f>(covariance_matrix, true);

    


}