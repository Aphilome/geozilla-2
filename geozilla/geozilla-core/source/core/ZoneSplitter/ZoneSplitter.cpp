#include "ZoneSplitter.h"

//#include <nlohmann/json.hpp>

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

#include <algorithm>
#include <thread>


std::string ZoneSplitter::GenerateGeoJson(pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud) {
    auto clouds = SplitToClouds(originalCloud);


    return "{}";
}


std::vector<Zone> ZoneSplitter::SplitToClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud) {
    auto zones = std::vector<Zone>();
    zones.push_back(Zone{ "remove me, please", originalCloud });

    
    auto horizontCloud = CreateHorizontCloud(originalCloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr grassCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roadCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& point : *horizontCloud) {
        if (IsGreenMore(point))
            grassCloud->points.push_back(point);
        else
            roadCloud->points.push_back(point);
    }
    zones.push_back(Zone{ "grass", grassCloud });
    zones.push_back(Zone{ "road", roadCloud });
    
    


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr whithoutHorizontCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(originalCloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.0, 10.0);
    //pass.setNegative (true);
    pass.filter(*whithoutHorizontCloud);



    //zones.push_back(Zone{ "test", whithoutHorizontCloud });

    auto obstacles = CreateObstaclesObjects(originalCloud);
    for (auto& o : obstacles)
        zones.push_back(Zone{ "obstacles", o });


    for (auto& z : zones)
        VisualizeCloud(z.cloud, z.type);
        

    return zones;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr ZoneSplitter::CreateHorizontCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
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

    return horizontal_planes;
}


bool ZoneSplitter::IsGreenMore(const pcl::PointXYZRGB& point) {
    return point.g > point.r && point.g > point.b;
}


void ZoneSplitter::VisualizeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string title) {
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


std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ZoneSplitter::CreateObstaclesObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    auto obstacles = std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>();
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(1.2); // 2 meters
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
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

//std::string ZoneSplitter::ClassifyZone(const Zone& sector) {
    //cv::Mat image = ZoneToImage(cloud);
    //cv::imwrite("output_image.png", image);

    //// Вычисление признаков GLCM
    //std::vector<double> features = computeGLCMFeatures(image);
    //double mean = features[0];
    //double variance = features[1];
    //double entropy = features[2];
    //double contrast = features[3];

    //if (entropy < 1.0 && contrast < 0.1) {
    //    return "road";
    //}
    //else if (entropy < 1.0 && contrast >= 0.1) {
    //    return "sidewalk";
    //}
    //else if (entropy >= 1.0 && variance < 0.5) {
    //    return "grass";
    //}
    //else if (variance >= 0.5) {
    //    return "building";
    //}

//  return "unknown";
//}

//std::string ZoneSplitter::CreateGeoJson(const std::vector<Zone>& zones) {
    //json geojson;
    //geojson["type"] = "FeatureCollection";
    //geojson["features"] = json::array();

    //for (const auto& sector : sectors) {
    //    json feature;
    //    feature["type"] = "Feature";
    //    feature["properties"]["type"] = sector.type;
    //
    //    json geometry;
    //    geometry["type"] = "Polygon";
    //    geometry["coordinates"] = json::array();
    //
    //    json coordinates = json::array();
    //    for (const auto& point : sector.points) {
    //        coordinates.push_back({ point.x, point.y, point.z });
    //    }
    //    geometry["coordinates"].push_back(coordinates);
    //
    //    feature["geometry"] = geometry;
    //    geojson["features"].push_back(feature);
    //}

    //return geojson.dump(4);
//	return {};
//}


//cv::Mat zoneToImage(Zone& zone) {
//    int width = 500;
//    int height = 500;
//    cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
//
//    for (const auto& point : zone.points) {
//        int x = static_cast<int>((point.x + 10) * 25);
//        int y = static_cast<int>((point.y + 10) * 25);
//        if (x >= 0 && x < width && y >= 0 && y < height) {
//            image.at<cv::Vec3b>(y, x) = cv::Vec3b(point.b, point.g, point.r);
//        }
//    }
//
//    return image;
//}

//std::vector<double> computeGLCMFeatures(const cv::Mat& image) {
//    cv::Mat grayImage;
//    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
//
//    cv::Mat glcm = cv::Mat::zeros(256, 256, CV_32F);
//    int dx = 1, dy = 0;
//
//    for (int y = 0; y < grayImage.rows - dy; y++) {
//        for (int x = 0; x < grayImage.cols - dx; x++) {
//            int i = grayImage.at<uchar>(y, x);
//            int j = grayImage.at<uchar>(y + dy, x + dx);
//            glcm.at<float>(i, j)++;
//        }
//    }
//
//    glcm /= (grayImage.rows * grayImage.cols);
//
//    std::vector<double> features;
//
//    double mean = 0.0;
//    double variance = 0.0;
//    double entropy = 0.0;
//    double contrast = 0.0;
//
//    for (int i = 0; i < glcm.rows; i++) {
//        for (int j = 0; j < glcm.cols; j++) {
//            double val = glcm.at<float>(i, j);
//            mean += val;
//            variance += (i - mean) * (i - mean) * val;
//            if (val > 0) {
//                entropy -= val * log2(val);
//            }
//            contrast += (i - j) * (i - j) * val;
//        }
//    }
//
//    features.push_back(mean);
//    features.push_back(variance);
//    features.push_back(entropy);
//    features.push_back(contrast);
//
//    return features;
//}
