#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

template<typename PointT>
class cluster {
public:

    //constructor
    cluster();
    //deconstructor
    ~cluster();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol);
    void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */