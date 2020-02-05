/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

template <typename PointT>
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
               std::unique_ptr<ProcessPointClouds<PointT>> &processor,
               typename pcl::PointCloud<PointT>::Ptr &input_cloud)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    auto filtered_cloud = processor->FilterCloud(input_cloud, 0.3, Eigen::Vector4f(-75, -6, -3, 1), Eigen::Vector4f(75, 8, 20, 1));
    auto lidar_chassis = processor->SegmentLidarChassis(filtered_cloud, Eigen::Vector4f(-2, -2, -2, 1), Eigen::Vector4f(3, 2, 0, 1));
    auto clouds = processor->CustomSegmentPlane(filtered_cloud, 50, 0.25);

    auto obstacle_clusters = processor->CustomClustering(clouds.second, 1.2, 10, 1000);
    int cluster_index = 0;
    for (auto &cluster : *obstacle_clusters)
    {
        Box box = processor->BoundingBox(cluster);
        renderBox(viewer, box, cluster_index);
        renderPointCloud(viewer, cluster, std::to_string(cluster_index), Color(dis(gen), dis(gen), dis(gen)));
        cluster_index++;
    }
    // renderPointCloud(viewer,clouds.second,"Obstacles", Color(1,0,1));
    renderPointCloud(viewer, clouds.first, "Road", Color(0, 1, 0));
    renderBox(viewer, lidar_chassis, cluster_index, Color(0.4, 0.3, 0.9));
}

void cityScene(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    std::unique_ptr<ProcessPointClouds<pcl::PointXYZI>> processor(new ProcessPointClouds<pcl::PointXYZI>());
    std::vector<boost::filesystem::path> stream = processor->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
    while (!viewer->wasStopped())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        input_cloud = processor->loadPcd((*streamIterator).string());
        cityBlock<pcl::PointXYZI>(viewer, processor, input_cloud);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    std::unique_ptr<Lidar> lidar(new Lidar(cars, 0));
    // renderRays(viewer, Vect3(0,0,0), lidar->scan());
    // renderPointCloud(viewer, lidar->scan(), "viewer");

    // TODO:: Create point processor
    std::unique_ptr<ProcessPointClouds<pcl::PointXYZ>> processor(new ProcessPointClouds<pcl::PointXYZ>());

    auto clouds = processor->SegmentPlane(lidar->scan(), 1000, 0.2);

    std::vector<Color> colors = {Color(1, 1, 0), Color(1, 0, 1), Color(0, 1, 1)};

    auto clusters = processor->Clustering(clouds.second, 2, 1, 100);

    for (int i = 0; i < clusters.size(); i++)
    {
        Box box = processor->BoundingBox(clusters[i]);
        renderBox(viewer, box, i);
        renderPointCloud(viewer, clusters[i], std::to_string(i), colors[i]);
    }

    renderPointCloud(viewer, clouds.first, "ground", Color(0, 1, 0));
    // renderPointCloud(viewer, clouds.second, "cars", Color(1, 0, 0));
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    cityScene(viewer);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}