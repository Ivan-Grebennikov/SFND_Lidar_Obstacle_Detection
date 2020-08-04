// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "helpers.h"
#include <unordered_set>
#include <algorithm>

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

    typename pcl::PointCloud<PointT>::Ptr voxelFiltered(new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> voxelFilter;
    voxelFilter.setInputCloud(cloud);
    voxelFilter.setLeafSize(filterRes, filterRes, filterRes);
    voxelFilter.filter(*voxelFiltered);

    pcl::CropBox<PointT> regionFilter;

    typename pcl::PointCloud<PointT>::Ptr regionFiltered(new pcl::PointCloud<PointT>);

    regionFilter.setInputCloud(voxelFiltered);
    regionFilter.setMin(minPoint);
    regionFilter.setMax(maxPoint);
    regionFilter.filter(*regionFiltered);

    std::vector<int> roofIndices;

    pcl::CropBox<PointT> roofFilter;

    roofFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roofFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roofFilter.setInputCloud(regionFiltered);
    roofFilter.filter(roofIndices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    inliers->indices = roofIndices;

    typename pcl::PointCloud<PointT>::Ptr regionWithoutRoof(new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(regionFiltered);
    extractor.setIndices(inliers);
    extractor.setNegative(true);
    extractor.filter(*regionWithoutRoof);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return regionWithoutRoof;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstaclesCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr roadCloud(new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extractor;

    extractor.setInputCloud(cloud);
    extractor.setIndices(inliers);

    extractor.setNegative(false);
    extractor.filter(*roadCloud);

    extractor.setNegative(true);
    extractor.filter(*obstaclesCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclesCloud, roadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefs(new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg(new pcl::SACSegmentation<PointT>);

    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);

    seg.segment(*inliers, *coefs);

    if (inliers->indices.empty())
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Segment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    srand(time(NULL));

    std::vector<std::unordered_set<int> > planes;

    planes.reserve(maxIterations);

    for (int iteration = 0; iteration < maxIterations; ++iteration)
    {
        planes.emplace_back();

        std::unordered_set<int>& plane = planes.back();

        const int p1 = randomOnInterval(cloud->points.size() - 1);
        const int p2 = randomOnInterval(cloud->points.size() - 1);
        const int p3 = randomOnInterval(cloud->points.size() - 1);

        for (int p = 0; p < cloud->points.size(); ++p)
        {
            if (distanceToPlane(cloud->points[p1], cloud->points[p2], cloud->points[p3], cloud->points[p]) <= distanceThreshold)
            {
                plane.insert(p);
            }
        }
    }

    std::sort(std::begin(planes), std::end(planes),
              [](const std::unordered_set<int>& _rhs, const std::unordered_set<int>& _lhs)
    {
        return _rhs.size() < _lhs.size();
    });

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    const std::unordered_set<int>& inliers = planes.back();

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    cloudInliers->width = cloudInliers->points.size();
    cloudInliers->height = 1;
    cloudInliers->is_dense = true;

    cloudOutliers->width = cloudOutliers->points.size();
    cloudOutliers->height = 1;
    cloudOutliers->is_dense = true;

    return std::make_pair(cloudOutliers, cloudInliers);
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringPCL(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndexes;

    pcl::EuclideanClusterExtraction<PointT> euclideanClustering;
    euclideanClustering.setClusterTolerance(clusterTolerance);
    euclideanClustering.setMinClusterSize(minSize);
    euclideanClustering.setMaxClusterSize(maxSize);
    euclideanClustering.setSearchMethod(tree);
    euclideanClustering.setInputCloud(cloud);
    euclideanClustering.extract(clusterIndexes);

    for (const auto& indexes : clusterIndexes)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>(*cloud, indexes.indices));
        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    KdTree3D tree;

    std::vector<std::vector<float> > points;

    for (int id = 0; id < cloud->points.size(); ++id)
    {
        points.push_back({cloud->points[id].x, cloud->points[id].y, cloud->points[id].z});

        tree.insert(points.back(), id);
    }

    std::vector<std::vector<int> > clustersIxs = euclideanCluster(points, &tree, clusterTolerance);

    auto last = std::remove_if(std::begin(clustersIxs), std::end(clustersIxs), [minSize, maxSize](const std::vector<int>& _indexes)
    {
        return _indexes.size() < minSize || _indexes.size() > maxSize;
    });

    clustersIxs.erase(last, std::end(clustersIxs));

    for (const auto& indexes : clustersIxs)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>(*cloud, indexes));
        clusters.push_back(clusterCloud);
    }

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
