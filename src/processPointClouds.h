#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>

struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template <typename PointT>
struct KdTree
{
	Node* root;
	KdTree()
	: root(NULL)
	{}
	void inserthelper(Node *&node, uint level, PointT point, int id)
	{
	    uint index = level%3;
	
		if(node == NULL)
		{
		 std::vector<float> v_point(point.data, point.data+3);
		 node = new Node(v_point,id);
		}
		else if(point.data[index] < node->point[index])
		{
		inserthelper(node->left,level+1,point,id);
		}
		else
		{
		inserthelper(node->right,level+1,point,id);
		}
	}
	void insert(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		for(uint index = 0; index < cloud->points.size(); index++)
		{
		   inserthelper(root,0,cloud->points[index],index);
		}
	}

	void searchHelper(Node *&node,uint depth,std::vector<int> *ids,PointT target, float distanceTol)
	{
		uint id = depth%3;
		if(node!=NULL)
		{
			if(((node->point[0]<target.data[0]+distanceTol)&&(node->point[0]>target.data[0]-distanceTol))&&
			   ((node->point[1]<target.data[1]+distanceTol)&&(node->point[1]>target.data[1]-distanceTol))&&
			   ((node->point[2]<target.data[2]+distanceTol)&&(node->point[2]>target.data[2]-distanceTol)))
			{
				uint dis=sqrt((node->point[0]-target.data[0])*(node->point[0]-target.data[0])+
						      (node->point[1]-target.data[1])*(node->point[1]-target.data[1])+
					          (node->point[2]-target.data[2])*(node->point[2]-target.data[2]));

				if(dis<distanceTol)
				{
					ids->push_back(node->id);
				}
			}
			if(target.data[id]-distanceTol<node->point[id])
			{
				searchHelper(node->left,depth+1,ids,target,distanceTol);
			}
			if(target.data[id]+distanceTol>node->point[id])
			{
				searchHelper(node->right,depth+1,ids,target,distanceTol);
			}
		}
	}

	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		uint depth =0;
		uint maxdistance=0;

		searchHelper(root,depth,&ids,target,distanceTol);
		return ids;
	}


};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(
		typename pcl::PointCloud<PointT>::Ptr cloud,
		float filterRes,
		Eigen::Vector4f minPoint,
		Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(
		pcl::PointIndices::Ptr inliers,
		typename pcl::PointCloud<PointT>::Ptr cloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RANSAC_SegmentPlane(
		typename pcl::PointCloud<PointT>::Ptr cloud,
		int maxIterations,
		float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering_Cluster(
		typename pcl::PointCloud<PointT>::Ptr cloud,
		float clusterTolerance,
		int minSize,
		int maxSize);

	std::vector<std::vector<int>> euclidean_Cluster(
		typename pcl::PointCloud<PointT>::Ptr cloud,
		typename KdTree<PointT>::KdTree* tree,
		float distanceTol,
		int minSize,
		int maxSize);

	void Proximity(
		typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int> &cluster,
		std::vector<bool> &processed_flag,
		int idx,typename KdTree<PointT>::KdTree* tree,
		float distanceTol,
		int maxSize);

    Box BoundingBox(
		typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(
		typename pcl::PointCloud<PointT>::Ptr cloud,
		std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

};


#endif /* PROCESSPOINTCLOUDS_H_ */
