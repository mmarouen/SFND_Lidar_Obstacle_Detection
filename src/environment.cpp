/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <algorithm>
#include <random>
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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  
  //filter parameters
  float filterRes=0.3;
  Eigen::Vector4f minVec(-10, -6, -3, 1);
  Eigen::Vector4f maxVec(30, 6, 3, 1);
  //plane segmentation parameters
  int maxIterations=25;
  float distanceThreshold=0.2;
  //clustering parameters
  float clusterTolerance =0.5;
  int minSize=10;
  int maxSize=600;

  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterRes , minVec,maxVec);
  //renderPointCloud(viewer,filterCloud,"filterCloud");
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud=pointProcessorI->SegmentPlane(filterCloud,maxIterations,distanceThreshold);
  //renderPointCloud(viewer,segmentCloud.first,"ObstCloud",Color(1,0,0));
  //renderPointCloud(viewer,segmentCloud.second,"PlanetCloud",Color(0,1,0));
  
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters_filtered=pointProcessorI->Clustering(segmentCloud.first,clusterTolerance,minSize,maxSize);
  int clusterId=0;
  std::vector<Color> colors ={Color(1,0,0),Color(0,1,1),Color(0,0,1)};
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr clus: cloudClusters_filtered)
  {
      std::cout << "Cluster size";
      pointProcessorI->numPoints(clus);
      renderPointCloud(viewer,clus,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
      Box box=pointProcessorI->BoundingBox(clus);
      renderBox(viewer,box,clusterId);
      ++clusterId;
  }
  renderPointCloud(viewer,segmentCloud.second,"PlanetCloud",Color(0,1,0));
  
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
    Lidar* lidar=new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud=lidar->scan();
    //Lidar lidar= Lidar(cars,0.0);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud=lidar.scan();
    //renderRays(viewer,lidar->position,inputCloud);
    renderPointCloud(viewer,inputCloud,"pt cloud");
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud=pointProcessor.SegmentPlane(inputCloud,100,0.2);
    renderPointCloud(viewer,segmentCloud.first,"ObstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"PlanetCloud",Color(0,1,0));
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
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    
    //Single frame processing
    /*
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    cityBlock(viewer,pointProcessorI,inputCloudI);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
    */
    
    //streaming frame processing
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    auto streamIterator = stream.begin();
    while (!viewer->wasStopped ())
    {
        //viewer->spinOnce ();
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
}