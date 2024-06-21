#pragma once

#include "GeoTypes.h"

namespace gz::core
{

class ConcaveHullGenerator
{
public:
    static GeoPointCloud Generate(const GeoPointCloud& pointCloud);
};

} // namespace gz::core
