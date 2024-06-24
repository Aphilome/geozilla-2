#pragma once

#include "GeoTypes.h"

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gz::core
{

// grass
// road
// sidewalk
// building
// obstacles
// default (in front, all other)

class ZoneSplitter {
public:
    std::vector<Zone> SplitToZones(PointCloud::Ptr originalCloud, bool visualize);

private:
    void VisualizeCloud(PointCloud::Ptr cloud, std::string title, bool visualize);
    std::vector<PointCloud::Ptr> CreateHorizontClouds(PointCloud::Ptr horizontCloud);
    std::vector<PointCloud::Ptr> CreateHorizontCutting(PointCloud::Ptr cloud);
    std::vector<PointCloud::Ptr> CreateObstaclesObjects(PointCloud::Ptr cloud);
};

} // namespace gz::core
