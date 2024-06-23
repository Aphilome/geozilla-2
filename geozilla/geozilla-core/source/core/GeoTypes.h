#pragma once

#include <CesiumGltf/Model.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>

#include <nlohmann/json.hpp>

#include <glm/vec3.hpp>

namespace gz::core
{

using GeoModel = CesiumGltf::Model;
using GeoJson = nlohmann::ordered_json;
using Point = pcl::PointXYZRGB;
using ConcaveHull = pcl::ConcaveHull<Point>;
using PointCloud = pcl::PointCloud<Point>;

struct GeoPointCloud
{
    PointCloud::Ptr points;
    glm::dvec3 center;
};

} // namespace gz::core
