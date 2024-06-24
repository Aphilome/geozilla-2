#pragma once

#include "GeoTypes.h"

namespace gz::core
{

class ConcaveHullGenerator
{
public:
    static PointCloud::Ptr Generate(const PointCloud::Ptr& pointCloud);
};

} // namespace gz::core
