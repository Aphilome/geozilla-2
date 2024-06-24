#pragma once

#include "GeoTypes.h"

#include <array>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace gz::core
{

class GltfToPointCloudConverter
{
public:
    static GeoPointCloud Convert(const GeoModel& model);

private:
    struct Triangle
    {
        std::array<glm::vec3, 3> positions;
        std::array<glm::vec2, 3> texCoords;
    };

    static void Triangulate(const Triangle& triangle, const CesiumGltf::Image* image, PointCloud& pointCloud);
    static void AddPoint(const glm::vec3& point, const glm::vec2 texCoord, const CesiumGltf::Image* image, PointCloud& pointCloud);
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
