#pragma once

#include "GeoTypes.h"

namespace gz::core
{

class GltfToPointCloudConverter
{
public:
    static GeoPointCloud Convert(const GeoModel& model, bool normalize);

private:
    static PointCloud::Ptr ExtractPoints(const GeoModel& model);
    static glm::dvec3 ExtractCenter(const GeoModel& model);
    static PointCloud::Ptr Normalize(const PointCloud::Ptr& points, const glm::dvec3& center);
};

} // namespace gz::core
