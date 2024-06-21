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
        assert(primitive.mode == MeshPrimitive::Mode::TRIANGLES);

        auto positionIt = primitive.attributes.find("POSITION");
        if (positionIt == std::end(primitive.attributes))
            return;

        auto accessorIndex = positionIt->second;
        const auto& accessor = gltf.accessors[accessorIndex];
        assert(accessor.type == CesiumGltf::AccessorSpec::Type::VEC3);

        const auto& bufferView = gltf.bufferViews[accessor.bufferView];
        const auto& buffer = gltf.buffers[bufferView.buffer];
        const auto* positions = reinterpret_cast<const float*>(buffer.cesium.data.data() + bufferView.byteOffset + accessor.byteOffset);

        for (size_t i = 0; i < accessor.count; ++i)
        {
            auto position = glm::dvec4(positions[3 * i + 0], positions[3 * i + 1], positions[3 * i + 2], 1.0);
            position = transform * position;
            points->emplace_back(static_cast<float>(position.x), static_cast<float>(position.y), static_cast<float>(position.z));
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

} // namespace gz::core
