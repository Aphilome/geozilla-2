#pragma once

#include "GeoTypes.h"

#include <array>

namespace gz::core
{

class GltfToPointCloudConverter
{
public:
    static GeoPointCloud Convert(const GeoModel& model);

private:
    static PointCloud::Ptr ExtractPoints(const GeoModel& model);
    static glm::dvec3 ExtractCenter(const GeoModel& model);
    static CesiumGeospatial::Cartographic ComputeGeoCoord(const PointCloud::Ptr& points, const glm::dvec3& center);
    static PointCloud::Ptr Normalize(const PointCloud::Ptr& points, const CesiumGeospatial::Cartographic& geoCoord);
    static const CesiumGltf::Accessor* GetAccessor(const GeoModel& model, const std::unordered_map<std::string, int32_t>& attributes, const std::string& name);
    static const std::byte* GetAttributeBuffer(const GeoModel& model, const CesiumGltf::Accessor* accessor);
    static const CesiumGltf::TextureInfo* GetTextureInfo(const GeoModel& model, int32_t materialIndex);
    static const CesiumGltf::Image* GetImage(const GeoModel& model, const CesiumGltf::TextureInfo* textureInfo);
    static std::array<std::uint8_t, 3> GetColor(const CesiumGltf::Image* image, float u, float v);
};

} // namespace gz::core
