#include "ConcaveHullGenerator.h"

#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>

namespace gz::core
{

PointCloud::Ptr ConcaveHullGenerator::Generate(const PointCloud::Ptr& pointCloud)
{
    auto cloudProjected = std::make_shared<PointCloud>();
    for (auto&& p : *pointCloud)
    {
        cloudProjected->emplace_back(p.x, 0.0f, p.z, p.r, p.g, p.b);
    }

    auto cloudHull = std::make_shared<PointCloud>();
    auto hull = pcl::ConcaveHull<Point>();
    hull.setInputCloud(cloudProjected);
    hull.setAlpha(10.0);
    hull.reconstruct(*cloudHull);

    pcl::Indices indices(1);
    std::vector<float> sqrDistances(1);
    pcl::KdTreeFLANN<Point> kdTree;
    kdTree.setInputCloud(cloudProjected);

    for (auto&& p : *cloudHull)
    {
        int count = kdTree.nearestKSearch(p, 1, indices, sqrDistances);
        if (count == 0)
        {
            assert(false);
            continue;
        }

        p.y = (*pointCloud)[indices[0]].y;
    }

    return cloudHull;
}

} // namespace gz::core
