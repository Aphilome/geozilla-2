#include "GeozillaCore.h"
#include "ZoneSplitter/ZoneSplitter.h"

#include <Logger/ConsoleLogger.h>
#include <Loader/GeoModelLoader.h>

#include "Algorithm/GltfToPointCloudConverter.h"
#include <Algorithm/ConcaveHullGenerator.h>
#include <Algorithm/GeoJsonGenerator.h>

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
    return GltfToPointCloudConverter::Convert(models.front());
}

std::vector<Zone> GenerateConcaveHulls(std::vector<Zone> zones)
{
    for (auto&& zone : zones)
    {
        zone.cloud = ConcaveHullGenerator::Generate(zone.cloud);
    }
    return zones;
}

GeoJson GenerateGeoJson(const std::vector<Zone>& zones, const GeoCoord& geoCoord, const std::filesystem::path& path)
{
    auto fileName = path.filename().string();
    auto dotPos = fileName.find_last_of('.');
    if (dotPos != std::string::npos)
    {
        fileName = fileName.substr(0, dotPos);
    }
    return GeoJsonGenerator::Generate(zones, geoCoord, fileName);
}

} // namespace

std::string GenerateGeoJson(const std::filesystem::path& path, bool visualize)
{
    constexpr auto indent = 4;
    auto models = LoadGeoModels(path);
    auto pointCloud = ConvertToPointCloud(models);
    auto zones = ZoneSplitter{}.SplitToZones(pointCloud.points, visualize);
    auto zoneHulls = GenerateConcaveHulls(std::move(zones));
    auto geoJson = GenerateGeoJson(zoneHulls, pointCloud.geoCoord, path);
    return geoJson.dump(indent);
}

const char* GenerateGeoJsonBuffer(const char* path)
{
    if (!path)
        return nullptr;

    auto geoJson = GenerateGeoJson(path, false);
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
