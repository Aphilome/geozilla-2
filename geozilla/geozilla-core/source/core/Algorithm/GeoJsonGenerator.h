#pragma once

#include "GeoTypes.h"

#include <string>
#include <vector>

namespace gz::core
{

class GeoJsonGenerator
{
public:
    static GeoJson Generate(const std::vector<GeoPointCloud>& pointClouds, const std::string& name);

private:
    static GeoJson GenerateFeatures(const std::vector<GeoPointCloud>& pointClouds);
    static GeoJson GenerateCoordinates(const GeoPointCloud& pointCloud);
    static double ToDegrees(double radians);
};

} // namespace gz::core
