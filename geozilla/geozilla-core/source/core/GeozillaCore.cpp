#include "GeozillaCore.h"

#include <Logger/ConsoleLogger.h>
#include <Loader/GeoModelLoader.h>

#include "Algorithm/GltfToPointCloudConverter.h"
#include <Algorithm/ConcaveHullGenerator.h>
#include <Algorithm/GeoJsonGenerator.h>

// remove unused
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/common.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>

#include <thread>
#include <algorithm>

using namespace gz::core;

namespace
{

std::vector<GeoModel> LoadGeoModels(const std::filesystem::path& path)
{
    if (!std::filesystem::exists(path))
        return {};

    auto loader = GeoModelLoader();
#ifdef _DEBUG
    loader.SetLogger(std::make_shared<ConsoleLogger>());
#endif
    return loader.Load(path);
}

GeoPointCloud ConvertToPointCloud(const std::vector<GeoModel>& models)
{
    if (models.empty())
        return {};

    assert((models.size() == 1) && "Multi models is not supported");
    return GltfToPointCloudConverter::Convert(models.front(), true);
}

std::vector<GeoPointCloud> GenerateConcaveHulls(const std::vector<GeoPointCloud>& pointClouds)
{
    std::vector<GeoPointCloud> concaveHulls(pointClouds.size());
    std::transform(std::begin(pointClouds), std::end(pointClouds), std::begin(concaveHulls), [](const GeoPointCloud& pc)
    {
        return ConcaveHullGenerator::Generate(pc);
    });
    return concaveHulls;
}

GeoJson GenerateGeoJson(const std::vector<GeoPointCloud>& pointClouds, const std::filesystem::path& path)
{
    auto fileName = path.filename().string();
    auto dotPos = fileName.find_last_of('.');
    if (dotPos != std::string::npos)
    {
        fileName = fileName.substr(0, dotPos);
    }
    return GeoJsonGenerator::Generate(pointClouds, fileName);
}

} // namespace

std::string GenerateGeoJson(const std::filesystem::path& path)
{
    constexpr auto indent = 4;
    auto models = LoadGeoModels(path);
    auto pointCloud = ConvertToPointCloud(models);
    auto hullClouds = GenerateConcaveHulls({ pointCloud });
    auto geoJson = GenerateGeoJson(hullClouds, path);
    return geoJson.dump(indent);
}

const char* GenerateGeoJsonBuffer(const char* path)
{
    if (!path)
        return nullptr;

    auto geoJson = GenerateGeoJson(path);
    const auto size = geoJson.size();
    auto* buffer = new char[size + 1];
    buffer[size] = '\0';
    std::copy(std::cbegin(geoJson), std::cend(geoJson), buffer);
    return buffer;
}

void FreeBuffer(const char* buffer)
{
    delete[] buffer;
}
