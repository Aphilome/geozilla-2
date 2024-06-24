#pragma once

#include "GeoTypes.h"

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gz::core
{
class ZoneSplitter {
public:
    std::vector<Zone> SplitToZones(PointCloud::Ptr originalCloud, bool visualize);

private:
    void VisualizeCloud(PointCloud::Ptr cloud, std::string title, bool visualize);
    std::vector<PointCloud::Ptr> CreateHorizontClouds(PointCloud::Ptr horizontCloud);
    std::vector<PointCloud::Ptr> CreateHorizontCutting(PointCloud::Ptr cloud);
    std::vector<PointCloud::Ptr> CreateObstaclesObjects(PointCloud::Ptr cloud);
    void Classificator(std::vector<Zone>& zones);

    std::string _planeType = "plane";
    std::string _objectType = "object";
};

} // namespace gz::core
