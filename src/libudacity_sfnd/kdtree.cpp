#include "libudacity_sfnd/kdtree.h"

template <typename PointT>
sfnd::Node<PointT>::Node()
{
}

template <typename PointT>
sfnd::Node<PointT>::Node(PointT new_point, int new_index) : point(new_point), index(new_index)
{
}

template <typename PointT>
sfnd::KDTree<PointT>::KDTree()
{
}

template <typename PointT>
void sfnd::KDTree<PointT>::insert(PointT point, int index)
{
    insert_helper(root, point, index, 0);
}

template <typename PointT>
std::vector<int> sfnd::KDTree<PointT>::search(PointT target, float distance_tolerance)
{
    std::vector<int> indices;
    float x_min = target.x - distance_tolerance;
    float x_max = target.x + distance_tolerance;

    float y_min = target.y - distance_tolerance;
    float y_max = target.y + distance_tolerance;

    float z_min = target.z - distance_tolerance;
    float z_max = target.z + distance_tolerance;

    std::queue < std::pair<std::shared_ptr<Node<PointT>>, int>> to_go_queue;

    while (!to_go_queue.empty())
    {
        auto entry = to_go_queue.front();
        to_go_queue.pop();
        int level = entry.second;
        auto current = entry.first;

        if (current.use_count() == 0)
        {
            continue;
        }

        if (is_proximal_(current->point, target, distance_tolerance, x_min, x_max, y_min, y_max, z_min, z_max))
        {
            indices.push_back(current->id);
        }

        auto range = check_range_(current->point, level, x_min, x_max, y_min, y_max, z_min, z_max);

        if (range == 1)
        {
            to_go_queue.push(std::make_pair(current->left, ++level));
        }
        else if (range == 0)
        {
            to_go_queue.push(std::make_pair(current->left, ++level));
            to_go_queue.push(std::make_pair(current->right, ++level));
        }
        else
        {
            to_go_queue.push(std::make_pair(current->right, ++level));
        }
    }

    return indices;
}

template <typename PointT>
int sfnd::KDTree<PointT>::check_range_(PointT point, int level,
                                       float x_min, float x_max,
                                       float y_min, float y_max,
                                       float z_min, float z_max)
{
    // Return 1 to check only left subtree
    // Return 0 to check both subtree
    // Return -1 to check right subtree
    if (level % 3 == 0)
    {
        if (point.x >= x_min && point.x <= x_max)
        {
            return 0;
        }
        else if (point.x <= x_min)
        {
            return -1;
        }
        else
        {
            return 1;
        }
    }
    else if (level % 3 == 1)
    {
        if (point.y >= y_min && point.y <= y_max)
        {
            return 0;
        }
        else if (point.y <= y_min)
        {
            return -1;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        if (point.z >= z_min && point.z <= z_max)
        {
            return 0;
        }
        else if (point.z <= z_min)
        {
            return -1;
        }
        else
        {
            return 1;
        }
    }
}

template <typename PointT>
bool sfnd::KDTree<PointT>::is_proximal_(PointT point,
                                        PointT target,
                                        float distance_tolerance,
                                        float x_min, float x_max,
                                        float y_min, float y_max,
                                        float z_min, float z_max)
{
    if (point.x >= x_min &&
        point.x <= x_max &&
        point.y >= y_min &&
        point.y <= y_max &&
        point.z >= z_min &&
        point.z <= z_max)
    {
        if (sqrt(pow((point.x - target.x), 2) +
                 pow((point.y - target.y), 2) +
                 pow((point.z - target.z), 2)) <= distance_tolerance)
        {
            return true;
        }
    }
    return false;
}

template <typename PointT>
void sfnd::KDTree<PointT>::insert_helper_(std::shared_ptr<Node<PointT>> &current, PointT point, int index, int level)
{
    if (current.use_count() == 0)
    {
        current.reset(point, index);
    }
    else
    {
        if (level % 3 == 0)
        {
            if (point.x <= current->point.x)
            {
                insert_helper_(current->left, point, index, ++level);
            }
            else
            {
                insert_helper_(current->right, point, index, ++level);
            }
        }
        else if (level % 3 == 1)
        {
            if (point.y <= current->point.y)
            {
                insert_helper_(current->left, point, index, ++level);
            }
            else
            {
                insert_helper_(current->right, point, index, ++level);
            }
        }
        else
        {
            if (point.z <= current->point.z)
            {
                insert_helper_(current->left, point, index, ++level);
            }
            else
            {
                insert_helper_(current->right, point, index, ++level);
            }
        }
    }
}