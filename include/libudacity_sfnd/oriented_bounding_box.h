#ifndef LIBUDACITY_SFND_ORIENTED_BOUNDING_BOX_H_
#define LIBUDACITY_SFND_ORIENTED_BOUNDING_BOX_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "Eigen/Dense"


namespace sfnd
{

struct OBBox{
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
    float x_size;
    float y_size;
    float z_size;

    OBBox();

    OBBox(Eigen::Vector3f translation_, Eigen::Quaternionf rotation_, float x_size_, float y_size_, float z_size_);
};

template <typename PointT>
class OrientedBoundingBox
{
public:
    OrientedBoundingBox();

    OrientedBoundingBox(typename pcl::PointCloud<PointT>::Ptr cloud);

    OBBox findBoundingBox();

private:
    typename pcl::PointCloud<PointT>::Ptr cloud_;
};
} // namespace sfnd

#endif