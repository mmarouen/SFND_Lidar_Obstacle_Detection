// PCL lib Functions for processing point clouds 

#include <algorithm>
#include <random>
#include "processPointClouds.h"
#include <unordered_set>
//#include "quiz/cluster/kdtree.h"
//#include "quiz/cluster/cluster.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

//downsample + ROI
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_voxels (new pcl::PointCloud<PointT> ());
	// Create the filtering object
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (filterRes, filterRes, filterRes);
	sor.filter (*cloud_voxels);

	typename pcl::PointCloud<PointT>::Ptr cloud_cropped (new pcl::PointCloud<PointT> ());
	pcl::CropBox<PointT> cropBox(true);
	cropBox.setInputCloud(cloud_voxels);
	cropBox.setMin(minPoint);
	cropBox.setMax(maxPoint);
	cropBox.filter(*cloud_cropped);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    return cloud_cropped;
}

//segment into ground + obstacles
template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	pcl::PointIndices::Ptr inliersResult (new pcl::PointIndices);
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
            /*
			if(std::count(inliers->indices.begin(),inliers->indices.end(),id)){
				continue;
            }
            */
			PointT pt=cloud->points[id];
			float x4=pt.x;
			float y4=pt.y;
			float z4=pt.z;
			float dist=fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);
			if(dist <= distanceTol){
				inliers->indices.push_back(id);
                //inliersResult1.insert(id);
            }
		}

		if(inliers->indices.size() > inliersResult->indices.size()){
            inliersResult->indices.clear();
            inliersResult->indices.insert(inliersResult->indices.end(), std::begin(inliers->indices), std::end(inliers->indices)); 
        }
	}
	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obst_cloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planet_cloud (new pcl::PointCloud<PointT> ());
    for (int index : inliers->indices)
        planet_cloud->points.push_back(cloud->points[index]);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    //extract.filter (*cloud_p);
    extract.setNegative (true);
    extract.filter (*obst_cloud);
    //cloud_filtered.swap (cloud_f);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obst_cloud, planet_cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    /*
    //Create the segmentation object
    std::cout <<"cloud point size "<< cloud->points.size() <<std::endl;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    */
    inliers = Ransac(cloud,maxIterations,distanceThreshold);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

//cluster obstacles
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector< PointT, Eigen::aligned_allocator< PointT >> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[indice]=true;
	cluster.push_back(indice);
  	std::vector<float> pt;
	pt.push_back(points[indice].x);
	pt.push_back(points[indice].y);
	pt.push_back(points[indice].z);
	std::vector<int> nearest= tree->search(pt,distanceTol);
	for(int id : nearest){
		if(!processed[id]){
			clusterHelper(id,points,cluster,processed,tree,distanceTol);
		}
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol,int minSize,int maxSize)
{
	std::vector<std::vector<int>> clusters_indices;
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool> processed(cloud->points.size(),false);

	int i=0;
	while(i < cloud->points.size()){
		if(processed[i]){
			i++;
			continue;
		}
		std::vector<int> cluster;
		clusterHelper(i,cloud->points,cluster,processed,tree,distanceTol);
        if(cluster.size()>=minSize && cluster.size()<=maxSize)
		    clusters_indices.push_back(cluster);
		i++;
	}

	for(std::vector<int> cluster : clusters_indices){
		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new typename pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = cluster.begin (); pit != cluster.end (); ++pit){
            clusterCloud->points.push_back (cloud->points[*pit]);
        }
            
        /*
        for(int indice: cluster){
            PointT point;
  		    point.x = points[indice][0];
  		    point.y = points[indice][1];
  		    point.z = points[indice][2];
			//clusterCloud->points.push_back(pcl::PointT(points[indice][0],points[indice][1],points[indice][2],0));
			clusterCloud->points.push_back(point);
        }
        */
		clusterCloud->width = clusterCloud->points.size ();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
		clusters.push_back(clusterCloud);
	}
	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	KdTree* tree = new KdTree;
  	int it = 0;
    for (int i=0; i<cloud->points.size(); i++){
  		std::vector<float> pt;
		pt.push_back(cloud->points[i].x);
		pt.push_back(cloud->points[i].y);
		pt.push_back(cloud->points[i].z);
        tree->insert(pt,i);
    }
    	 
	clusters = euclideanCluster(cloud, tree, clusterTolerance,minSize, maxSize);
    /*
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }
    */
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    return clusters;
}
//General helper functions
template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}