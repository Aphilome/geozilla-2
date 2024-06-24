#include "GeoJsonGenerator.h"

#include <CesiumUtility/Math.h>
#include <CesiumGeospatial/Ellipsoid.h>

#include <glm/gtc/constants.hpp>

#include <cassert>

namespace gz::core
{

GeoJson GeoJsonGenerator::Generate(const std::vector<Zone>& zones, const GeoCoord& geoCoord, const std::string& name)
{
    return {
        {"type", "FeatureCollection"},
        {"name", name},
        {"features", GenerateFeatures(zones, geoCoord)},
    };
}

GeoJson GeoJsonGenerator::GenerateFeatures(const std::vector<Zone>& zones, const GeoCoord& geoCoord)
{
    GeoJson features;
    for (auto&& zone : zones)
    {
        features.push_back({
            {"type", "Feature"},
            {"properties", {
                {"zoneType", zone.type},
                {"maxHeight", zone.maxHeight}
            }},
            {"geometry", {
                {"type", "Polygon"},
                {"coordinates", GenerateCoordinates(zone.cloud, geoCoord)},
            }},
        });
    }
    return features;
}

GeoJson GeoJsonGenerator::GenerateCoordinates(const PointCloud::Ptr& pointCloud, const GeoCoord& geoCoord)
{
    using namespace Eigen;

    if (!pointCloud)
        return {};

    auto coordinates = GeoJson{};
    auto [cx, cy, cz] = geoCoord.center;
    auto [longitude, latitude, _] = geoCoord.cartographic;

    for (auto&& point : *pointCloud)
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
