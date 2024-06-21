#include "ConcaveHullGenerator.h"

#include <pcl/filters/voxel_grid.h>

namespace gz::core
{

GeoPointCloud ConcaveHullGenerator::Generate(const GeoPointCloud& pointCloud)
{
    auto filteredPointCloud = std::make_shared<PointCloud>();
    auto voxelGrid = pcl::VoxelGrid<Point>();
    voxelGrid.setInputCloud(pointCloud.points);
    voxelGrid.setLeafSize(2.0f, 2.0f, 2.0f);
    voxelGrid.filter(*filteredPointCloud);

    auto hullCloud = std::make_shared<PointCloud>();
    pcl::ConcaveHull<Point> concaveHull;
    concaveHull.setInputCloud(filteredPointCloud);
    concaveHull.setAlpha(0.3);
    concaveHull.setDimension(2);
    concaveHull.reconstruct(*hullCloud);

    return GeoPointCloud{
        std::move(hullCloud),
        pointCloud.center
    };
}

} // namespace gz::core
