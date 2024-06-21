#pragma once

#include <string>
#include <filesystem>

__declspec(dllexport) std::string GenerateGeoJson(const std::filesystem::path& path);
__declspec(dllexport) const char* GenerateGeoJsonBuffer(const char* path);
__declspec(dllexport) void FreeBuffer(const char* buffer);
