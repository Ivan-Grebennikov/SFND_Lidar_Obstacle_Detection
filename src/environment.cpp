/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor 

    std::shared_ptr<Lidar> lidar = std::make_shared<Lidar>(cars, 0);
    auto cloud = lidar->scan();
    renderPointCloud(viewer, cloud, "Point cloud");

    // TODO:: Create point processor

    ProcessPointClouds<pcl::PointXYZ> pointCloudProcessor;

    auto separatedClouds = pointCloudProcessor.Segment(cloud, 100, 0.2);
  
    auto cloudClusters = pointCloudProcessor.Clustering(separatedClouds.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
          std::cout << "cluster size ";
          pointCloudProcessor.numPoints(cluster);

          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

          const Box box = pointCloudProcessor.BoundingBox(cluster);
          renderBox(viewer, box, clusterId);

          ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& _viewer,
               ProcessPointClouds<pcl::PointXYZI>& _pointCloudProcessor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& _inputCloud)
{
    auto filteredCloud = _pointCloudProcessor.FilterCloud(_inputCloud, 0.25, Eigen::Vector4f (-25, -5, -2, 1), Eigen::Vector4f (25, 6, 2, 1));

    auto separatedClouds = _pointCloudProcessor.Segment(filteredCloud, 100, 0.25);

    auto cloudClusters = _pointCloudProcessor.Clustering(separatedClouds.first, 0.5, 10, 500);

    renderPointCloud(_viewer, separatedClouds.second, "roadCloud", Color(0, 1 ,0));

    static const std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for (int clusterId = 0; clusterId < cloudClusters.size(); ++clusterId)
    {
        auto& cluster = cloudClusters[clusterId];

        std::cout << "cluster size ";
        _pointCloudProcessor.numPoints(cluster);

        renderPointCloud(_viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % 3]);

        const Box box = _pointCloudProcessor.BoundingBox(cluster);
        renderBox(_viewer, box, clusterId);
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI> pointCloudProcessor;

    std::vector<boost::filesystem::path> stream = pointCloudProcessor.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    while (!viewer->wasStopped())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        auto inputCloud = pointCloudProcessor.loadPcd(streamIterator->string());
        cityBlock(viewer, pointCloudProcessor, inputCloud);

        if (++streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    } 
}
