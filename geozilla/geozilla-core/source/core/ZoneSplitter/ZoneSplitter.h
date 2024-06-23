#pragma once

#include "GeoTypes.h"

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//typedef pcl::PointXYZRGB PointType;

namespace gz::core
{

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
    std::vector<Zone> SplitToClouds(PointCloud::Ptr originalCloud);
private:
    void VisualizeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string title);
    bool IsGreenMore(const pcl::PointXYZRGB& point);
    std::vector<PointCloud::Ptr> CreateHorizontClouds(PointCloud::Ptr horizontCloud);
    std::vector<PointCloud::Ptr> CreateHorizontSplitting(PointCloud::Ptr cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> CreateObstaclesObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
};

} // namespace gz::core
