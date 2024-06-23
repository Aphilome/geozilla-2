#include "ZoneSplitter.h"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <algorithm>
#include <thread>

namespace gz::core
{

PointCloud::Ptr CreateHorizontCloud(PointCloud::Ptr cuttedHorizontCloud) {
    pcl::NormalEstimation<Point, pcl::Normal> ne;
    ne.setInputCloud(cuttedHorizontCloud);
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(3.3);
    ne.compute(*cloud_normals);

    pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    PointCloud::Ptr horizontal_plane(new PointCloud);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.9);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.3);
    seg.setInputCloud(cuttedHorizontCloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud(cuttedHorizontCloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*horizontal_plane);

    return horizontal_plane;
}

std::vector<PointCloud::Ptr> CreatePlanes(PointCloud::Ptr horizontCloud) {
    std::vector<PointCloud::Ptr> clouds;

    pcl::search::Search <Point>::Ptr tree(new pcl::search::KdTree<Point>);

    pcl::IndicesPtr indices(new std::vector <int>);
    pcl::removeNaNFromPointCloud(*horizontCloud, *indices);

    pcl::RegionGrowingRGB<Point> reg;
    reg.setInputCloud(horizontCloud);
    reg.setIndices(indices);
    reg.setSearchMethod(tree);

    reg.setDistanceThreshold(10);
    reg.setPointColorThreshold(6); // 6
    reg.setRegionColorThreshold(5);// 5
    reg.setMinClusterSize(100); // 600

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    PointCloud::Ptr colored_cloud = reg.getColoredCloud();
    

    clouds.push_back(colored_cloud);

    
    PointCloud::Ptr filteredCloud(new PointCloud);

    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud(colored_cloud);
    sor.setMeanK(50); // Number of neighbors to analyze for each point
    sor.setStddevMulThresh(1.0); // Standard deviation threshold
    sor.filter(*filteredCloud);

    clouds.push_back(filteredCloud);


    return clouds;
}

std::vector<PointCloud::Ptr> ZoneSplitter::CreateHorizontClouds(PointCloud::Ptr cuttedHorizontCloud) {
    auto horizont = CreateHorizontCloud(cuttedHorizontCloud);
    auto planes = CreatePlanes(horizont);

    return planes;
}

std::vector<Zone> ZoneSplitter::SplitToClouds(PointCloud::Ptr originalCloud) {
    auto zones = std::vector<Zone>();
    zones.push_back(Zone{ "remove me, please", originalCloud });

    
    auto cutting = CreateHorizontSplitting(originalCloud);
    auto cuttedHorizontCloud = cutting[0];
    auto cuttedObjectsCloud = cutting[1];


    auto planes = CreateHorizontClouds(cuttedHorizontCloud);
    for (auto& i : planes)
        zones.push_back({ "plane", i });


    //auto obstacles = CreateObstaclesObjects(cuttedObjectsCloud);
    //for (auto& i : obstacles)
    //    zones.push_back({ "obstacle", i});


    for (auto& z : zones)
        VisualizeCloud(z.cloud, z.type);
        

    return zones;
}

std::vector<PointCloud::Ptr> ZoneSplitter::CreateHorizontSplitting(PointCloud::Ptr cloud) {
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    size_t totalPointsCount = cloud->points.size();
    float deltaHorizont = (maxPt.y - minPt.y) * 5 / 100; // 5%
    float percentOfHorizont = 0.0;
    float currentHorizontLevel = minPt.y;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr horizontal_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    while (percentOfHorizont < 15.0)
    {
        currentHorizontLevel += deltaHorizont;

        horizontal_plane->clear();

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(minPt.y, currentHorizontLevel);
        pass.filter(*horizontal_plane);
        percentOfHorizont = horizontal_plane->points.size() / (float)totalPointsCount * 100.0;

        std::cout
            << "min: " << minPt.y
            << "; corrent level: " << currentHorizontLevel
            << "; percent: " << percentOfHorizont
            << std::endl;
    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr withoutHorizontalCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(minPt.y, currentHorizontLevel);
    pass.setNegative(true);
    pass.filter(*withoutHorizontalCloud);

    return { horizontal_plane, withoutHorizontalCloud };


    /*

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(3.3);
    ne.compute(*cloud_normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr horizontal_planes(new pcl::PointCloud<pcl::PointXYZRGB>);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.9);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.3);
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*horizontal_planes);
    */

    //return horizontal_plane;
}


void ZoneSplitter::VisualizeCloud(PointCloud::Ptr cloud, std::string title) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(title));

    using namespace std::chrono_literals;

    viewer->setBackgroundColor(0, 0, 20);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "1");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

std::vector<PointCloud::Ptr> ZoneSplitter::CreateObstaclesObjects(PointCloud::Ptr cloud) {
    std::vector<PointCloud::Ptr> obstacles;
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance(1.2); // 2 meters
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto& indices : cluster_indices) {
        PointCloud::Ptr cloud_cluster(new PointCloud);
        for (const auto& idx : indices.indices) {
            cloud_cluster->points.push_back(cloud->points[idx]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        obstacles.push_back(cloud_cluster);
    }

    return obstacles;
}

} // namespace gz::core
