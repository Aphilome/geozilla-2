#include "GeoJsonGenerator.h"

#include <CesiumUtility/Math.h>
#include <CesiumGeospatial/Ellipsoid.h>

#include <glm/gtc/constants.hpp>

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
    using namespace Eigen;

    auto&& pc = pointCloud.points;
    if (!pc)
        return {};

    auto coordinates = GeoJson{};
    auto [cx, cy, cz] = pointCloud.center;
    auto [longitude, latitude, _] = pointCloud.geoCoord;

    for (auto&& point : pc->points)
    {
        auto p = Vector4d(point.x, point.y, point.z, 1.0);
        auto transform = Affine3d::Identity();
        transform.translate(Vector3d(cx, cy, cz));
        transform.rotate(AngleAxisd(longitude, Vector3d::UnitX()));
        transform.rotate(AngleAxisd(-glm::pi<double>() - latitude, Vector3d::UnitY()));
        p = transform * p;

        auto cartographicPoint = CesiumGeospatial::Ellipsoid::WGS84.cartesianToCartographic(glm::dvec3(p.x(), p.y(), p.z()));
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
