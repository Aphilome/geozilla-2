#pragma once

__declspec(dllexport) const char* GenerateGeoJson(const char* path, double latitude, double longitude);
__declspec(dllexport) void FreeBuffer(const char* buffer);
