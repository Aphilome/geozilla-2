#include "GltfToPointCloudConverter.h"

#include <CesiumGltf/Model.h>
#include <cassert>

namespace gz::core
{

void GltfToPointCloudConverter::Convert(const IGeoModelLoader::GeoModel& model, Points& points)
{
    using CesiumGltf::Model;
    using CesiumGltf::Node;
    using CesiumGltf::Mesh;
    using CesiumGltf::MeshPrimitive;

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

        // Extract texture coordinates
        std::vector<std::tuple<float, float>> texCoords;
        auto texCoordIt = primitive.attributes.find("TEXCOORD_0");
        if (texCoordIt != std::end(primitive.attributes))
        {
            auto texCoordAccessorIndex = texCoordIt->second;
            const auto& texCoordAccessor = gltf.accessors[texCoordAccessorIndex];
            const auto& texCoordBufferView = gltf.bufferViews[texCoordAccessor.bufferView];
            const auto& texCoordBuffer = gltf.buffers[texCoordBufferView.buffer];
            const auto* texCoordsPtr = reinterpret_cast<const float*>(texCoordBuffer.cesium.data.data() + texCoordBufferView.byteOffset + texCoordAccessor.byteOffset);

            for (size_t i = 0; i < texCoordAccessor.count; ++i)
            {
                float u = texCoordsPtr[2 * i + 0];
                float v = texCoordsPtr[2 * i + 1];
                texCoords.emplace_back(u, v);
            }
        }
        
        // Load texture image
        auto image = gltf.images[0].cesium;
        auto texWidth = image.width;
        auto texHeight = image.height;
        auto texData = image.pixelData;
        auto texChannels = image.channels;

        for (size_t i = 0; i < accessor.count; ++i)
        {
            auto x = positions[3 * i + 0];
            auto y = positions[3 * i + 1];
            auto z = positions[3 * i + 2];

            float r = 100, g = 100, b = 100;
            if (!texCoords.empty() && i < texCoords.size())
            {
                float u, v;
                std::tie(u, v) = texCoords[i];

                // Map UV coordinates to texture pixel
                int texX = static_cast<int>(u * texWidth) % texWidth;
                int texY = static_cast<int>(v * texHeight) % texHeight;
                int texIndex = (texY * texWidth + texX) * texChannels;
                r = (float)texData[texIndex + 0];
                g = (float)texData[texIndex + 1];
                b = (float)texData[texIndex + 2];
            }

            points.emplace_back(x, y, z, r, g, b);
        }
    };

    for (size_t i = 0; i < model.scenes.size(); ++i)
    {
        model.forEachPrimitiveInScene(static_cast<int32_t>(i), primitiveCallback);
    }
}

} // namespace gz::core
