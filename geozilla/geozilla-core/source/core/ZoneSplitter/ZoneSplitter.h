#pragma once

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointType;

// grass
// road
// sidewalk
// building
// obstacles
// default (in front, all other)
struct Zone {
    std::string type;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
};

class ZoneSplitter {
public:
    std::string GenerateGeoJson(pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud);
private:
    void VisualizeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string title);
    std::vector<Zone> SplitToClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr originalCloud);
    bool IsGreenMore(const pcl::PointXYZRGB& point);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CreateHorizontCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> CreateObstaclesObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

};
