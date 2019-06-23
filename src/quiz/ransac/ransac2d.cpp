/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include "../cluster/kdtree.h"
#include "../cluster/cluster.cpp"
#include "../../render/box.h"
#include <chrono>
#include <string>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;
  	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D(){
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene(){
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Point2Cloud(std::vector<std::vector<float>> points){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;
  	return cloud;
}

std::vector<std::vector<float>> Cloud2Point(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	std::vector<std::vector<float>> points;
	for(int i = 0; i < cloud->points.size(); i++){
  		std::vector<float> pt;
		pt.push_back(cloud->points[i].x);
		pt.push_back(cloud->points[i].y);
		pt.push_back(cloud->points[i].z);
  		points.push_back(pt);
  	}
  	return points;
}

//template<typename PointT>
pcl::PointIndices::Ptr Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	//std::unordered_set<int> inliersResult;
	pcl::PointIndices::Ptr inliersResult (new pcl::PointIndices);
	std::unordered_set<int> inliersResult0;
	srand(time(NULL));
	
	// TODO: Fill in this function
	while(maxIterations--){
		//select 2 points randomly
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		while(inliers->indices.size() <3){
            //int r = rand() % cloud->points.size();
            std::random_device random_device;
            std::mt19937 engine{random_device()};
            std::uniform_int_distribution<int> dist(0, cloud->points.size() - 1);
            int r = dist(engine);
			inliers->indices.push_back(r);
        }

        auto itr=inliers->indices.begin();
		float x1,x2,x3,y1,y2,y3,z1,z2,z3;
		x1=cloud->points[*itr].x;
		y1=cloud->points[*itr].y;
		z1=cloud->points[*itr].z;
		itr++;
		x2=cloud->points[*itr].x;
		y2=cloud->points[*itr].y;
		z2=cloud->points[*itr].z;
		itr++;
		x3=cloud->points[*itr].x;
		y3=cloud->points[*itr].y;
		z3=cloud->points[*itr].z;
		//build the plane
		float a = ((y2-y1)*(z3-z1)-(z2-z1)*(y3-y1));
		float b = ((z2-z1)*(x3-x1)-(x2-x1)*(z3-z1));
		float c = ((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1));
		float d = (-(a*x1+b*y1+c*z1));
		//number of points within distanceTol to line
		for(int id=0; id < cloud->points.size();id++){

			if(std::count(inliers->indices.begin(),inliers->indices.end(),id)){
				continue;
            }
			pcl::PointXYZ pt=cloud->points[id];
			float x4=pt.x;
			float y4=pt.y;
			float z4=pt.z;
			float d=fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);
			if(d <= distanceTol){
				inliers->indices.push_back(id);
            }
		}
		if(inliers->indices.size() > inliersResult->indices.size()){
            inliersResult->indices.clear();
            inliersResult->indices.insert(inliersResult->indices.end(), std::begin(inliers->indices), std::end(inliers->indices)); 
        }
	}

	return inliersResult;

}

std::unordered_set<int> Ransac2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
	std::unordered_set<int> inliersResult;

	srand(time(NULL));
	
	// TODO: Fill in this function
	while(maxIterations--){
		//select 2 points randomly
		std::unordered_set<int> inliers;
		while(inliers.size()<3)
			inliers.insert(rand() % cloud->points.size());
		auto itr=inliers.begin();
		float x1,x2,x3,y1,y2,y3,z1,z2,z3;
		x1=cloud->points[*itr].x;
		y1=cloud->points[*itr].y;
		z1=cloud->points[*itr].z;
		itr++;
		x2=cloud->points[*itr].x;
		y2=cloud->points[*itr].y;
		z2=cloud->points[*itr].z;
		itr++;
		x3=cloud->points[*itr].x;
		y3=cloud->points[*itr].y;
		z3=cloud->points[*itr].z;

		//build the plane
		float a = ((y2-y1)*(z3-z1)-(z2-z1)*(y3-y1));
		float b = ((z2-z1)*(x3-x1)-(x2-x1)*(z3-z1));
		float c = ((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1));
		float d = (-(a*x1+b*y1+c*z1));

		//number of points within distanceTol to line
		for(int id=0; id < cloud->points.size();id++){
			if(inliers.count(id)>0)
				continue;
			pcl::PointXYZ pt=cloud->points[id];
			float x4=pt.x;
			float y4=pt.y;
			float z4=pt.z;
			float d=fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);
			if(d <= distanceTol)
				inliers.insert(id);
		}
		if(inliers.size() > inliersResult.size()){
			inliersResult=inliers;
		}
	}

	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();
	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// Segment data
	//std::unordered_set<int> inliers = Ransac2(cloud, 30, 0.5);
	pcl::PointIndices::Ptr inliers = Ransac(cloud, 30, 0.5);
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());
	for(int index = 0; index < cloud->points.size(); index++){
		pcl::PointXYZ point = cloud->points[index];
		//if(inliers.count(index))
		if(std::count(inliers->indices.begin(),inliers->indices.end(),index)){
			cloudInliers->points.push_back(point);
		}
		else{
			cloudOutliers->points.push_back(point);
		}
	}

	// Render 2D point cloud with inliers and outliers
	if(inliers->indices.size()){
		//renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		//renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
		std::vector<std::vector<float>> points=Cloud2Point(cloudOutliers);

		KdTree* tree = new KdTree;
  		int it = 0;
    	for (int i=0; i<points.size(); i++) 
    		tree->insert(points[i],i); 

		//std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = euclideanCluster(points, tree, 3.0);
		int clusterId = 0;
		std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
		for(pcl::PointCloud<pcl::PointXYZ>::Ptr clus: clusters)
		{
			renderPointCloud(viewer,clus,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
			++clusterId;
		}
		/*
		for(std::vector<int> cluster : clusters){
			pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
			for(int indice: cluster)
				clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
			renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
			++clusterId;
		}
		*/
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
