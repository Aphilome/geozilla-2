#include "GltfToPointCloudConverter.h"

#include <CesiumGltf/Model.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGltf/ExtensionCesiumRTC.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <glm/ext/matrix_transform.hpp>

#include <cassert>
#include <numeric>

namespace gz::core
{

GeoPointCloud GltfToPointCloudConverter::Convert(const GeoModel& model, bool normalize)
{
    auto pointCloud = GeoPointCloud{};
    pointCloud.center = ExtractCenter(model);
    pointCloud.points = ExtractPoints(model);
    if (normalize)
    {
        pointCloud.points = Normalize(pointCloud.points, pointCloud.center);
    }
    return pointCloud;
}

PointCloud::Ptr GltfToPointCloudConverter::ExtractPoints(const GeoModel& model)
{
    using CesiumGltf::Model;
    using CesiumGltf::Node;
    using CesiumGltf::Mesh;
    using CesiumGltf::MeshPrimitive;

    auto points = std::make_shared<PointCloud>();

    auto primitiveCallback = [&points](const Model& gltf, const Node& node, const Mesh& mesh, const MeshPrimitive& primitive, const glm::dmat4& transform)
    {
        const auto* positionAccessor = GetAccessor(gltf, primitive.attributes, "POSITION");
        auto positions = reinterpret_cast<const float*>(GetAttributeBuffer(gltf, positionAccessor));
        if (!positions)
        {
            assert(false && "Failed to find positions buffer");
            return;
        }

        const auto* textureInfo = GetTextureInfo(gltf, primitive.material);
        const auto* image = GetImage(gltf, textureInfo);
        const float* texCoords = nullptr;
        if (image)
        {
            const auto attributeName = "TEXCOORD_" + std::to_string(textureInfo->texCoord);
            const auto* texCoordAccessor = GetAccessor(gltf, primitive.attributes, attributeName);
            texCoords = reinterpret_cast<const float*>(GetAttributeBuffer(gltf, texCoordAccessor));
            assert(!texCoordAccessor || (texCoordAccessor->type == CesiumGltf::AccessorSpec::Type::VEC2));
        }

        assert(positionAccessor->type == CesiumGltf::AccessorSpec::Type::VEC3);

        for (int64_t i = 0; i < positionAccessor->count; ++i)
        {
            auto position = glm::dvec4(positions[3 * i], positions[3 * i + 1], positions[3 * i + 2], 1.0);
            position = transform * position;

            auto u = texCoords ? texCoords[2 * i + 0] : 0.0f;
            auto v = texCoords ? texCoords[2 * i + 1] : 0.0f;
            auto [r, g, b] = GetColor(image, u, v);
            points->emplace_back(position.x, position.y, position.z, r, g, b);
        }
    };

    for (size_t i = 0; i < model.scenes.size(); ++i)
    {
        model.forEachPrimitiveInScene(static_cast<int32_t>(i), primitiveCallback);
    }

    return points;
}

glm::dvec3 GltfToPointCloudConverter::ExtractCenter(const GeoModel& model)
{
    auto* cesiumRTC = model.getExtension<CesiumGltf::ExtensionCesiumRTC>();
    if (!cesiumRTC)
        return {};

    return { cesiumRTC->center[0], cesiumRTC->center[1], cesiumRTC->center[2] };
}

PointCloud::Ptr GltfToPointCloudConverter::Normalize(const PointCloud::Ptr& points, const glm::dvec3& center)
{
    if (!points)
        return nullptr;

    Point minPoint;
    Point maxPoint;
    pcl::getMinMax3D(*points, minPoint, maxPoint);

    auto cartesianPoint = center + glm::dvec3(
        std::midpoint(minPoint.x, maxPoint.x),
        std::midpoint(minPoint.y, maxPoint.y),
        std::midpoint(minPoint.z, maxPoint.z)
    );

    auto cartographicPoint = CesiumGeospatial::Ellipsoid::WGS84.cartesianToCartographic(cartesianPoint);
    if (!cartographicPoint)
        return nullptr;

    const auto longitude = static_cast<float>(cartographicPoint->longitude);
    const auto latitude = static_cast<float>(cartographicPoint->latitude);
    auto transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(-longitude, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(glm::pi<float>() + latitude, Eigen::Vector3f::UnitY()));

    auto transformedPointCloud = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*points, *transformedPointCloud, transform);
    return transformedPointCloud;
}

const CesiumGltf::Accessor* GltfToPointCloudConverter::GetAccessor(const GeoModel& model, const std::unordered_map<std::string, int32_t>& attributes, const std::string& name)
{
    auto positionIt = attributes.find(name);
    if (positionIt == std::end(attributes))
        return nullptr;

    auto accessorIndex = positionIt->second;
    if (accessorIndex < 0 || accessorIndex >= model.accessors.size())
        return nullptr;

    return &model.accessors[accessorIndex];
}

const std::byte* GltfToPointCloudConverter::GetAttributeBuffer(const GeoModel& model, const CesiumGltf::Accessor* accessor)
{
    if (!accessor)
        return nullptr;

    const auto bufferViewIndex = accessor->bufferView;
    if (bufferViewIndex < 0 || bufferViewIndex >= model.bufferViews.size())
        return nullptr;

    const auto& bufferView = model.bufferViews[bufferViewIndex];
    const auto bufferIndex = bufferView.buffer;
    if (bufferIndex < 0 || bufferIndex >= model.buffers.size())
        return nullptr;

    const auto& buffer = model.buffers[bufferIndex];
    return buffer.cesium.data.data() + bufferView.byteOffset + accessor->byteOffset;
}

const CesiumGltf::TextureInfo* GltfToPointCloudConverter::GetTextureInfo(const GeoModel& model, int32_t materialIndex)
{
    if (materialIndex < 0 || materialIndex >= model.materials.size())
        return nullptr;

    const auto& material = model.materials[materialIndex];
    if (!material.pbrMetallicRoughness)
        return nullptr;

    const auto& texture = material.pbrMetallicRoughness->baseColorTexture;
    if (!texture.has_value())
        return nullptr;

    return &texture.value();
}

const CesiumGltf::Image* GltfToPointCloudConverter::GetImage(const GeoModel& model, const CesiumGltf::TextureInfo* textureInfo)
{
    if (!textureInfo)
        return nullptr;

    const auto textureIndex = textureInfo->index;
    if (textureIndex < 0 || textureIndex >= model.textures.size())
        return nullptr;

    const auto& texture = model.textures[textureIndex];
    const auto imageIndex = texture.source;
    if (imageIndex < 0 || imageIndex >= model.images.size())
        return nullptr;

    return &model.images[imageIndex];
}

std::array<std::uint8_t, 3> GltfToPointCloudConverter::GetColor(const CesiumGltf::Image* image, float u, float v)
{
    if (!image)
        return {};

    assert(image->cesium.channels && "Unsupported channel count");

    const auto width = image->cesium.width;
    const auto height = image->cesium.height;
    const auto& pixelData = image->cesium.pixelData;
    auto texX = static_cast<int>(u * width) % width;
    auto texY = static_cast<int>(v * height) % height;
    auto colorIndex = (texY * width + texX) * image->cesium.channels;
    auto r = pixelData[colorIndex + 0];
    auto g = pixelData[colorIndex + 1];
    auto b = pixelData[colorIndex + 2];
    return { static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b) };
}

} // namespace gz::core
