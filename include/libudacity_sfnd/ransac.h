#ifndef LIB_UDACITY_RANSAC_H_
#define LIB_UDACITY_RANSAC_H_

#include <unordered_set>
#include <random>
#include <chrono>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "pcl/filters/extract_indices.h"


namespace sfnd
{
template<typename PointT>
class RANSAC
{
    public:

        using PointCloudPtr = typename pcl::PointCloud<PointT>::Ptr;

        RANSAC();

        ~RANSAC();

        RANSAC(PointCloudPtr cloud, int max_iterations, float distance_threshold);

        std::pair<PointCloudPtr, PointCloudPtr> segmentPlane();

    private:
        int max_iterations_;
        float distance_threshold_;
        PointCloudPtr cloud_;
        pcl::PointIndices::Ptr findGroundPoints_();
};
} // namespace sfnd

#endif // LIB_UDACITY_RANSAC_H_