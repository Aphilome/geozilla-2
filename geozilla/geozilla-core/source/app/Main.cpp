#include <iostream>

#include "GeozillaCore.h"

int main()
{
    std::cout << "Enter the path to the B3DM file:" << std::endl;

    std::string path;
    std::cin >> path;

    auto* geoJson = GenerateGeoJson(path.c_str());
    if (geoJson)
    {
        std::cout << geoJson << std::endl;
        FreeBuffer(geoJson);
    }

    return 0;
}
