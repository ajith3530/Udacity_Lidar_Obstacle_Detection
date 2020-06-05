// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


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

typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();

	// TODO:: Fill in the function to do voxel grid point reduction and region based filtering
	pcl::VoxelGrid<PointT> vg;
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);

	vg.setInputCloud(cloud);
	vg.setLeafSize(filterRes, filterRes, filterRes);
	vg.filter(*cloudFiltered);

	typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

	pcl::CropBox<PointT> region(true);
	region.setMin(minPoint);
	region.setMax(maxPoint);
	region.setInputCloud(cloudFiltered);
	region.filter(*cloudRegion);

	std::vector<int> indices;

	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
	roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
	roof.setInputCloud(cloudRegion);
	roof.filter(indices);

	pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
	for (int point : indices)
		inliers->indices.push_back(point);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloudRegion);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloudRegion);



	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

	return cloudRegion;

}


// template<typename PointT>
// std::pair<typename pcl::Point<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
// {
//   // TODO: Create two new point clouds, one cloud with obstacles and other segmented plane

//     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
//     return segResult;
// }


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while (maxIterations--)
	{
		std::unordered_set<int> inliers;
		srand(time(NULL));
		// get three points for making a plane
		while (inliers.size() < 3)
		{	// insert random indexes within the point cloud indexes
			inliers.insert(rand() % (cloud->points.size()));
		}

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
		// get the (x,y) from random indexes
		auto inlier_iterator = inliers.begin();
		x1 = cloud->points[*inlier_iterator].x;
		y1 = cloud->points[*inlier_iterator].y;
		z1 = cloud->points[*inlier_iterator].z;
		inlier_iterator++;
		x2 = cloud->points[*inlier_iterator].x;
		y2 = cloud->points[*inlier_iterator].y;
		z2 = cloud->points[*inlier_iterator].z;
		inlier_iterator++;
		x3 = cloud->points[*inlier_iterator].x;
		y3 = cloud->points[*inlier_iterator].y;
		z3 = cloud->points[*inlier_iterator].z;

		float a = (((y2 - y1)*(z3 - z1)) - ((z2 - z1)*(y3 - y1)));
		float b = (((z2 - z1)*(x3 - x1)) - ((x2 - x1)*(z3 - z1)));
		float c = (((x2 - x1)*(y3 - y1)) - ((y2 - y1)*(x3 - x1)));
		float d = -(a*x1 + b*y1 + c*z1);
		float len = sqrt(a*a + b*b + c*c);

		for (int index = 0; index < cloud->points.size(); index++)
		{
			// if the index of the point match with the line index, ignore it.
			if (inliers.count(index) > 0) continue;

			auto point = cloud->points[index];
			float test_point_x = point.x;
			float test_point_y = point.y;
			float test_point_z = point.z;

			float test_point_distance = fabs(a*test_point_x + b*test_point_y + c*test_point_z + d) / len;

			if (test_point_distance <= distanceThreshold) inliers.insert(index);
		}
		if (inliers.size() > inliersResult.size())
		{
			inliersResult.clear();
			inliersResult = inliers;
		}

	}

	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if (inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentPlaneResult(cloudOutliers, cloudInliers);


	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	return segmentPlaneResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

	// Time clustering process
	auto startTime = std::chrono::steady_clock::now();

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	// TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

	return clusters;
}


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
	pcl::io::savePCDFileASCII(file, *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

	typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file \n");
	}
	std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

	return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

	std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{ dataPath }, boost::filesystem::directory_iterator{});

	// sort files in accending order so playback is chronological
	sort(paths.begin(), paths.end());

	return paths;

}

void clusterHelper(int index, const std::vector<std::vector<float>> points, std::vector<int>& cluster,
	std::vector<bool> &processed, KdTree* tree, float distanceTol)
{
	processed[index] = true;
	cluster.push_back(index);

	auto nearest = tree->search(points[index], distanceTol);
	// std::cout<<"nearest"<<nearest;

	for (int id : nearest)
	{
		if (!processed[id])
			clusterHelper(id, points, cluster, processed, tree, distanceTol);
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);

	int i = 0;
	while (i < points.size())
	{
		if (processed[i])
		{
			i++;
			continue;
		}
		std::vector<int> cluster;
		clusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}
	return clusters;

}
