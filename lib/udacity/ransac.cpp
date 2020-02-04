#include "ransac.h"

template <typename PointT>
sfnd::RANSAC<PointT>::RANSAC()
{
}

template <typename PointT>
sfnd::RANSAC<PointT>::~RANSAC()
{
}

template <typename PointT>
sfnd::RANSAC<PointT>::RANSAC(typename sfnd::RANSAC<PointT>::PointCloudPtr cloud,
                             int max_iterations,
                             float distance_threshold) : cloud_(cloud),
                                                         max_iterations_(max_iterations),
                                                         distance_threshold_(distance_threshold)
{
}

template <typename PointT>
std::pair<typename sfnd::RANSAC<PointT>::PointCloudPtr, typename sfnd::RANSAC<PointT>::PointCloudPtr> sfnd::RANSAC<PointT>::segmentPlane()
{
    sfnd::RANSAC<PointT>::PointCloudPtr inlier_cloud(new pcl::PointCloud<PointT>());
    sfnd::RANSAC<PointT>::PointCloudPtr outlier_cloud(new pcl::PointCloud<PointT>());
    
    auto inliers = sfnd::RANSAC<PointT>::findGroundPoints_();

    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud_);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*inlier_cloud);

    extract.setNegative(true);
    extract.filter(*outlier_cloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(inlier_cloud, outlier_cloud);
    return segResult;

}

template <typename PointT>
pcl::PointIndices sfnd::RANSAC<PointT>::findGroundPoints_()
{
    int maxIterations = max_iterations_;
    pcl::PointIndices ground_indices;
    std::random_device rd;
    std::mt19937 mt(rd());

    int i = 0, j = 0, k = 0;
    size_t cloud_size = cloud_->size();

    if(cloud_size == 0){
        return ground_indices;
    }

    std::uniform_int_distribution<int> dist(0, cloud_size);
    int best_count = 0, current_count = 0;
    int best_i = 0, best_j = 0, best_k = 0;
    int x1, y1, z1, x2, y2, z2, x3, y3, z3;
    int a = 0, b = 0, c = 0, d = 0;

    while (maxIterations > 0)
    {
        maxIterations--;
        i = dist(mt) % cloud_size;
        j = dist(mt) % cloud_size;
        k = dist(mt) % cloud_size;
        while (i == j)
        {
            j = dist(mt) % cloud_size;
        }
        while (i == k || j == k)
        {
            k = dist(mt) % cloud_size;
        }

        x1 = cloud_->at(i).x;
        x2 = cloud_->at(j).x;
        x3 = cloud_->at(k).x;

        y1 = cloud_->at(i).y;
        y2 = cloud_->at(j).y;
        y3 = cloud_->at(k).y;

        z1 = cloud_->at(i).z;
        z2 = cloud_->at(j).z;
        z3 = cloud_->at(k).z;

        a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        d = -(a * x1 + b * y1 + c * z1);

        for (auto &point : *cloud_)
        {
            float dist = abs(a * point.x + b * point.y + c * point.z + d);
            dist = dist / sqrt(a * a + b * b + c * c);
            if (dist <= distance_threshold_)
            {
                current_count++;
            }
        }

        if (current_count > best_count)
        {
            best_count = current_count;
            best_i = i;
            best_j = j;
            best_k = k;
        }
        current_count = 0;
    }

    x1 = cloud_->at(i).x;
    x2 = cloud_->at(j).x;
    x3 = cloud_->at(k).x;

    y1 = cloud_->at(i).y;
    y2 = cloud_->at(j).y;
    y3 = cloud_->at(k).y;

    z1 = cloud_->at(i).z;
    z2 = cloud_->at(j).z;
    z3 = cloud_->at(k).z;

    a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    d = -(a * x1 + b * y1 + c * z1);

    for (int iter = 0; iter < cloud_size; iter++)
    {
        float dist = abs(a * cloud_->at(iter).x + b * cloud_->at(iter).y + c * cloud_->at(iter).z + d) / sqrt(a * a + b * b + c * c);
        if (dist <= distance_threshold_)
        {
            ground_indices.indices.push_back(iter);
        }
    }
    return ground_indices;
}