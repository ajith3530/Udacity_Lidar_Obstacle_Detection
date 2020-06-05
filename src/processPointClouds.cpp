// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

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

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
	typename pcl::PointCloud<PointT>::Ptr cloud,
	float filterRes, Eigen::Vector4f minPoint,
	Eigen::Vector4f maxPoint)
{

    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);
    std::cerr << "Voxeled " << cloud_filtered->points.size () << std::endl;

    pcl::CropBox<PointT> region;
	region.setMin(minPoint);
	region.setMax(maxPoint);
	region.setInputCloud(cloud_filtered);
	region.filter(*cloud_filtered);
    std::cerr << "region " << cloud_filtered->points.size () << std::endl;

    std::vector<int> indices;
    pcl::CropBox<PointT> roof_region(true);
    roof_region.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
	roof_region.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
	roof_region.setInputCloud(cloud_filtered);
	roof_region.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for(int point:indices)
    	inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
	typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
	pcl::PointIndices::Ptr inliers,
	typename pcl::PointCloud<PointT>::Ptr cloud) 
{
	typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

	for(int it : inliers->indices)
	{
		planeCloud->points.push_back(cloud->points[it]);
	}

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RANSAC_SegmentPlane(
	typename pcl::PointCloud<PointT>::Ptr cloud,
	int maxIterations,
	float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	PointT point1;
	PointT point2;
	PointT point3;
	int idx1;
	int idx2;
	int idx3;
	float a,b,c,d,dis,len;

	for(int it=0;it<maxIterations;it++)
	{
		std::unordered_set<int> temp_Indices;
		while(temp_Indices.size()<3)
			temp_Indices.insert((rand() % cloud->points.size()));
		auto iter = temp_Indices.begin();
		idx1 = *iter;
		++iter;
		idx2 = *iter;
		++iter;
		idx3 = *iter;
		point1 = cloud->points[idx1];
		point2 = cloud->points[idx2];
		point3 = cloud->points[idx3];

		a = (((point2.y-point1.y)*(point3.z-point1.z))-((point2.z-point1.z)*(point3.y-point1.y)));
		b = (((point2.z-point1.z)*(point3.x-point1.x))-((point2.x-point1.x)*(point3.z-point1.z)));
		c = (((point2.x-point1.x)*(point3.y-point1.y))-((point2.y-point1.y)*(point3.x-point1.x)));
		d = -(a*point1.x+b*point1.y+c*point1.z);
		len = sqrt(a*a+b*b+c*c);

		for(int point_count=0;point_count<cloud->points.size();point_count++)
		{
			if(point_count!=idx1||point_count!=idx2||point_count!=idx3)
			{
				dis = (fabs(a*cloud->points[point_count].x+b*cloud->points[point_count].y+c*cloud->points[point_count].z+d)/len);
				if(dis<=distanceThreshold)
				{
					temp_Indices.insert(point_count);
				}
			}
		}
		if(temp_Indices.size()>inliersResult.size())
		{
			inliersResult.clear();
			inliersResult = temp_Indices;

		}
	}

	if (inliersResult.size () == 0)
	{
	  std::cerr << "No Inliers found." << std::endl;
	}
	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(
	typename pcl::PointCloud<PointT>::Ptr cloud,
	std::vector<int> &cluster,
	std::vector<bool> &processed_flag,int idx,
	typename KdTree<PointT>::KdTree* tree,
	float distanceTol,
	int maxSize)
{
	if ((processed_flag[idx] == false) &&
			(cluster.size()<maxSize))
	{
		processed_flag[idx] = true;
		cluster.push_back(idx);
		std::vector<int> nearby = tree->search(cloud->points[idx],distanceTol);
		for(int index : nearby)
		{
			if (processed_flag[index] == false)
			{
				Proximity(cloud, cluster, processed_flag, index, tree, distanceTol, maxSize);
			}
		}
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclidean_Cluster(
	typename pcl::PointCloud<PointT>::Ptr cloud,
	typename KdTree<PointT>::KdTree* tree,
	float distanceTol,
	int minSize,
	int maxSize)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed_flag(cloud->points.size(),false);

	for(int idx=0;idx<cloud->points.size();idx++)
	{
		if(processed_flag[idx]==false)
		{
			std::vector<int> cluster;
			Proximity(cloud, cluster,processed_flag,idx,tree,distanceTol,maxSize);
			if((cluster.size()>=minSize)&&cluster.size()<=maxSize)
				clusters.push_back(cluster);
		}
	}

	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_Cluster(
	typename pcl::PointCloud<PointT>::Ptr cloud,
	float clusterTolerance,
	int minSize,
	int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename KdTree<PointT>::KdTree *tree =new KdTree<PointT>;
    tree->insert(cloud);
	std::vector<std::vector<int>> cluster_indices = euclidean_Cluster(cloud, tree,clusterTolerance,minSize,maxSize);

	for (std::vector<std::vector<int>>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
	    for (std::vector<int>::const_iterator pit = it->begin (); pit != it->end (); ++pit)
	      cloud_cluster->points.push_back (cloud->points[*pit]); 
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    clusters.push_back(cloud_cluster);
	  }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "euclidean_Clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
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
    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr<< "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});
    sort(paths.begin(), paths.end());

    return paths;
}
