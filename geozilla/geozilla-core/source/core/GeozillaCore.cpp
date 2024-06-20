#include "GeozillaCore.h"
#include "Converter/GltfToPointCloudConverter.h"

#include <Logger/ConsoleLogger.h>
#include <Loader/GeoModelLoader.h>
#include <ZoneSplitter/ZoneSplitter.h>

#include <CesiumGltf/Model.h>

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

namespace
{

std::vector<gz::core::IGeoModelLoader::GeoModel> LoadGeoModels(const char* path)
{
    if (!path)
        return {};

    auto loader = gz::core::GeoModelLoader();
#ifdef _DEBUG
    loader.SetLogger(std::make_shared<gz::core::ConsoleLogger>());
#endif
    return loader.Load(path);
}

gz::core::GltfToPointCloudConverter::Points::Ptr ConvertToPointCloud(const std::vector<gz::core::IGeoModelLoader::GeoModel>& models)
{
    using namespace gz::core;

    auto pointCloud = std::make_shared<GltfToPointCloudConverter::Points>();

    std::for_each(std::begin(models), std::end(models), [&pc = *pointCloud](const auto& model)
    {
        gz::core::GltfToPointCloudConverter::Convert(model, pc);
    });

    return pointCloud;
}

const char* ConvertToRawMemory(const std::string& data)
{
    const auto size = data.size();
    auto* buffer = new char[size + 1];
    buffer[size] = '\0';
    std::copy(std::cbegin(data), std::cend(data), buffer);
    return buffer;
}

} // namespace

const char* GenerateGeoJson(const char* path, float latitude, float longitude)
{
    auto models = LoadGeoModels(path);
    auto originalCloud = ConvertToPointCloud(models);
    auto splitter = ZoneSplitter();
    auto geoJson = splitter.GenerateGeoJson(originalCloud);

    return ConvertToRawMemory(geoJson);
}

void FreeBuffer(const char* buffer)
{
    delete[] buffer;
}
