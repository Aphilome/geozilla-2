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

    PointCloud::Ptr filteredCloud(new PointCloud);
    
    // filtrate
    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud(horizontCloud);
    sor.setMeanK(50); // Number of neighbors to analyze for each point
    sor.setStddevMulThresh(1.0); // Standard deviation threshold
    sor.filter(*filteredCloud);

    // segmentation
    pcl::search::Search <Point>::Ptr tree(new pcl::search::KdTree<Point>);
    pcl::IndicesPtr indices(new std::vector <int>);
    pcl::removeNaNFromPointCloud(*filteredCloud, *indices);

    pcl::RegionGrowingRGB<Point> reg;
    reg.setInputCloud(filteredCloud);
    reg.setIndices(indices);
    reg.setSearchMethod(tree);

    reg.setDistanceThreshold(10);
    reg.setPointColorThreshold(6); // 6
    reg.setRegionColorThreshold(5);// 5
    reg.setMinClusterSize(100); // 600

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);
    //PointCloud::Ptr colored_cloud = reg.getColoredCloud();

    for (const auto& indices : clusters) {
        PointCloud::Ptr cloud_cluster(new PointCloud);
        for (const auto& idx : indices.indices) {
            cloud_cluster->points.push_back(filteredCloud->points[idx]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clouds.push_back(cloud_cluster);
    }

    return clouds;
}

std::vector<PointCloud::Ptr> ZoneSplitter::CreateHorizontClouds(PointCloud::Ptr cuttedHorizontCloud) {
    auto horizont = CreateHorizontCloud(cuttedHorizontCloud);
    auto planes = CreatePlanes(horizont);

    return planes;
}

std::vector<Zone> ZoneSplitter::SplitToZones(PointCloud::Ptr originalCloud, bool visualize) {
    std::vector<Zone> zones;

    VisualizeCloud(originalCloud, "originalCloud", visualize);

    auto cutting = CreateHorizontCutting(originalCloud);
    auto cuttedHorizontCloud = cutting[0];
    auto cuttedObjectsCloud = cutting[1];

    auto planes = CreateHorizontClouds(cuttedHorizontCloud);
    for (auto& i : planes)
        zones.push_back({ i, _planeType });

    auto objects = CreateObstaclesObjects(cuttedObjectsCloud);
    for (auto& i : objects)
        zones.push_back({ i, _objectType });

    Classificator(zones);

    for (auto& i : zones)
        VisualizeCloud(i.cloud, i.type, visualize);

    return zones;
}

std::vector<PointCloud::Ptr> ZoneSplitter::CreateHorizontCutting(PointCloud::Ptr cloud) {
    Point minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    size_t totalPointsCount = cloud->points.size();
    float deltaHorizont = (maxPt.y - minPt.y) * 5 / 100; // 5%
    float percentOfHorizont = 0.0;
    float currentHorizontLevel = minPt.y;

    PointCloud::Ptr horizontal_plane(new PointCloud);
    while (percentOfHorizont < 15.0)
    {
        currentHorizontLevel += deltaHorizont;

        horizontal_plane->clear();

        pcl::PassThrough<Point> pass;
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


    PointCloud::Ptr withoutHorizontalCloud(new PointCloud);
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(minPt.y, currentHorizontLevel);
    pass.setNegative(true);
    pass.filter(*withoutHorizontalCloud);

    return { horizontal_plane, withoutHorizontalCloud };
}

void ZoneSplitter::VisualizeCloud(PointCloud::Ptr cloud, std::string title, bool visualize) {
    if (!visualize)
        return;

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

Point getAvgColor(PointCloud::Ptr cloud) {
    Point point;

    int r = 0, g = 0, b = 0, total = 0;
    for (auto& p : cloud->points) {
        r += p.r;
        g += p.g;
        b += p.b;
        total++;
    }

    point.r = r / total;
    point.g = g / total;
    point.b = b / total;

    return point;
}

bool isGreen(Point point) {
    return point.g * 0.7 > point.r && point.g * 0.7 > point.b;
}

bool isGray(Point point) {
    // 70 - 230
    return abs(point.g - point.r) < 30
        && abs(point.g - point.b) < 30
        && abs(point.r - point.b) < 30
        && point.r > 70 && point.r < 230
        && point.g > 70 && point.g < 230
        && point.b > 70 && point.b < 230;
}

void ZoneSplitter::Classificator(std::vector<Zone>& zones) {
    for (auto& zone : zones) {
        Point minPt, maxPt;
        pcl::getMinMax3D(*zone.cloud, minPt, maxPt);
        float widht = maxPt.x - minPt.x;
        float length = maxPt.z - minPt.z;
        float height = maxPt.y - minPt.y;
        auto avgColor = getAvgColor(zone.cloud);
        float density = zone.cloud->points.size() / widht * length;

        zone.maxHeight = maxPt.y;

        std::cout
            << "- widht: " << widht
            << "; length: " << length
            << "; height: " << height
            << "; r: " << (int)avgColor.r
            << "; g: " << (int)avgColor.g
            << "; b: " << (int)avgColor.b
            << "; density: " << density
            << std::endl;

        // grass
        // road
        // sidewalk
        if (zone.type == _planeType) {
            if (isGreen(avgColor)) {
                zone.type = "grass";
            } else if (isGray(avgColor)) {
                // - widht: 21.124; length: 20.093; height: 1.31099; r: 108; g: 105; b: 103; density: 182.629
                if (widht > 10.0 && height > 10.0 && density > 150) {
                    zone.type = "road";
                }
                else {
                    zone.type = "sidewalk";
                }
            }
            else if (zone.type == _planeType)
                zone.type = "grass";
        }
        // building
        // obstacles
        else if (zone.type == _objectType) {
            if (widht > 2.0 && length > 2.0 && height > 2.0 && isGray(avgColor)) {
                zone.type = "building";
            }
            else {
                zone.type = "obstacles";
            }
        }
    }
}


} // namespace gz::core
