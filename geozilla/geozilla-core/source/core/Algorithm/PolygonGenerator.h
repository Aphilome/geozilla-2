#pragma once

#include "GeoTypes.h"

namespace gz::core
{

class PolygonGenerator
{
public:
    static PointCloud::Ptr Generate(const PointCloud::Ptr& pointCloud);

private:
    static PointCloud::Ptr ComputePolygon(const PointCloud::Ptr& pointCloud);
};

} // namespace gz::core
