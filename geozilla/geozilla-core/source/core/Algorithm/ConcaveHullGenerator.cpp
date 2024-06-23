#include "ConcaveHullGenerator.h"

#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>

namespace gz::core
{

GeoPointCloud ConcaveHullGenerator::Generate(const GeoPointCloud& pointCloud)
{
    Point minPoint;
    Point maxPoint;
    pcl::getMinMax3D(*pointCloud.points, minPoint, maxPoint);

    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    coefficients->values = { 0.0f, 1.0f, 0.0f, -maxPoint.y };

    auto cloudProjected = std::make_shared<PointCloud>();
    auto projection = pcl::ProjectInliers<Point>();
    projection.setModelType(pcl::SACMODEL_PLANE);
    projection.setInputCloud(pointCloud.points);
    projection.setModelCoefficients(coefficients);
    projection.filter(*cloudProjected);

    auto cloudHull = std::make_shared<PointCloud>();
    auto hull = pcl::ConcaveHull<Point>();
    hull.setInputCloud(cloudProjected);
    hull.setAlpha(10.0);
    hull.reconstruct(*cloudHull);

    return GeoPointCloud{
        std::move(cloudHull),
        pointCloud.center,
        pointCloud.geoCoord
    };
}

} // namespace gz::core
