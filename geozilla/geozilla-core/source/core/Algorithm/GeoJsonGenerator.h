#pragma once

#include "GeoTypes.h"

#include <string>
#include <vector>

namespace gz::core
{

class GeoJsonGenerator
{
public:
    static GeoJson Generate(const std::vector<Zone>& zones, const GeoCoord& geoCoord, const std::string& name);

private:
    static GeoJson GenerateFeatures(const std::vector<Zone>& zones, const GeoCoord& geoCoord);
    static GeoJson GenerateCoordinates(const PointCloud::Ptr& pointCloud, const GeoCoord& geoCoord);
    static double ToDegrees(double radians);
};

} // namespace gz::core
