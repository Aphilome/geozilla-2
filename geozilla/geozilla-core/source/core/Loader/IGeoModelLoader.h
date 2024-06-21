#pragma once

#include "GeoTypes.h"

#include <memory>
#include <vector>
#include <filesystem>

namespace gz::core
{

struct ILogger;

struct IGeoModelLoader
{
    virtual ~IGeoModelLoader() = default;

    virtual void SetLogger(std::shared_ptr<ILogger> logger) = 0;
    virtual bool IsFileSupported(const std::filesystem::path& path) const = 0;
    virtual std::vector<GeoModel> Load(const std::filesystem::path& path) = 0;
};

} // namespace gz::core
