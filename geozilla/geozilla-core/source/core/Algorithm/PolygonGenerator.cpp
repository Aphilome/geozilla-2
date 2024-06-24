#include "PolygonGenerator.h"

#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>

namespace gz::core
{

PointCloud::Ptr PolygonGenerator::Generate(const PointCloud::Ptr& pointCloud)
{
    auto projectedCloud = std::make_shared<PointCloud>();
    for (auto&& p : *pointCloud)
    {
        projectedCloud->emplace_back(p.x, 0.0f, p.z, p.r, p.g, p.b);
    }

    pcl::Indices indices(1);
    std::vector<float> sqrDistances(1);
    pcl::KdTreeFLANN<Point> kdTree;
    kdTree.setInputCloud(projectedCloud);

    auto polygonCloud = ComputePolygon(projectedCloud);
    for (auto&& p : *polygonCloud)
    {
        int count = kdTree.nearestKSearch(p, 1, indices, sqrDistances);
        assert(count != 0);
        if (count != 0)
        {
            p.y = (*pointCloud)[indices[0]].y;
        }
    }
    return polygonCloud;
}

PointCloud::Ptr PolygonGenerator::ComputePolygon(const PointCloud::Ptr& pointCloud)
{
    auto hullCloud = std::make_shared<PointCloud>();

    auto polygons = std::vector<pcl::Vertices>();
    auto concaveHull = pcl::ConcaveHull<Point>();
    concaveHull.setInputCloud(pointCloud);
    concaveHull.setAlpha(10.0);
    concaveHull.reconstruct(*hullCloud, polygons);

    if (polygons.size() != 1)
    {
        hullCloud = std::make_shared<PointCloud>();

        auto convexHull = pcl::ConvexHull<Point>();
        convexHull.setInputCloud(pointCloud);
        convexHull.reconstruct(*hullCloud);
    }

    return hullCloud;
}

} // namespace gz::core
