#pragma once

__declspec(dllexport) const char* GenerateGeoJson(const char* path, float latitude, float longitude);
__declspec(dllexport) void FreeBuffer(const char* buffer);
