
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
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
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
	filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (-15, -5, -2, 1), Eigen::Vector4f ( 30, 7, 4, 1));
	std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RANSAC_SegmentPlane(filterCloud, 50, 0.3);
	renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering_Cluster(segmentCloud.first, 0.3, 10, 750);
	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
	for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
	{
	   std::cout << "cluster size ";
	   pointProcessorI->numPoints(cluster);
	   renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);

	   Box box = pointProcessorI->BoundingBox(cluster);
	   renderBox(viewer,box,clusterId);

	   ++clusterId;
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
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    while (!viewer->wasStopped ())
    {
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);

      streamIterator++;
      if(streamIterator == stream.end())
        streamIterator = stream.begin();

      viewer->spinOnce ();
    }

}
