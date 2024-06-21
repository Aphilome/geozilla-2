#include "GeoJsonGenerator.h"

#include <CesiumUtility/Math.h>
#include <CesiumGeospatial/Ellipsoid.h>

#include <cassert>

namespace gz::core
{

GeoJson GeoJsonGenerator::Generate(const std::vector<GeoPointCloud>& pointClouds, const std::string& name)
{
    return {
        {"type", "FeatureCollection"},
        {"name", name},
        {"features", GenerateFeatures(pointClouds)},
    };
}

GeoJson GeoJsonGenerator::GenerateFeatures(const std::vector<GeoPointCloud>& pointClouds)
{
    GeoJson features;
    for (auto&& pointCloud : pointClouds)
    {
        features.push_back({
            {"type", "Feature"},
            {"properties", {
                {"zoneType", "unknown"},
            }},
            {"geometry", {
                {"type", "Polygon"},
                {"coordinates", GenerateCoordinates(pointCloud)},
            }},
        });
    }
    return features;
}

GeoJson GeoJsonGenerator::GenerateCoordinates(const GeoPointCloud& pointCloud)
{
    auto&& pc = pointCloud.points;
    if (!pc)
        return {};

    GeoJson coordinates;

    for (auto&& point : pc->points)
    {
        auto cartesianPoint = glm::dvec3(point.x, point.y, point.z);
        auto cartographicPoint = CesiumGeospatial::Ellipsoid::WGS84.cartesianToCartographic(cartesianPoint);
        if (cartographicPoint.has_value())
        {
            auto [lon, lat, height] = *cartographicPoint;
            coordinates.push_back(GeoJson::array({ ToDegrees(lon), ToDegrees(lat), height }));
        }
    }

    if (!coordinates.empty())
    {
        coordinates.push_back(coordinates.front());
    }

    return GeoJson::array({ coordinates });
}

double GeoJsonGenerator::ToDegrees(double radians)
{
    return CesiumUtility::Math::radiansToDegrees(radians);
}

} // namespace gz::core
