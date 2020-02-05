#ifndef LIBUDACITY_SFND_KDTREE_H_
#define LIBUDACITY_SFND_KDTREE_H_

#include <queue>
#include <utility>
#include "pcl/point_cloud.h"
namespace sfnd
{

template <typename PointT>
struct Node
{
    PointT point;
    int index;
    std::shared_ptr<Node<PointT>> left;
    std::shared_ptr<Node<PointT>> right;

    Node();

    Node(PointT new_point, int new_index);
};

template <typename PointT>
struct KDTree
{
    std::shared_ptr<Node<PointT>> root;

    KDTree();

    void insert(PointT point, int index);

    std::vector<int> search(PointT target, float distance_tolerance);

private:
    void insert_helper_(std::shared_ptr<Node<PointT>> &current, PointT point, int index, int level);

    int check_range_(PointT point, int level, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);

    bool is_proximal_(PointT point, PointT target, float distance_tolerance, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
};
} // namespace sfnd

#endif