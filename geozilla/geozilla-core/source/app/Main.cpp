#include "GeozillaCore.h"

#include <argparse/argparse.hpp>

#include <fstream>
#include <iostream>

namespace
{

void CreateArgumentsParser(argparse::ArgumentParser& program)
{
    program.add_description("GeoJSON Generation Tool for B3DM");

    program
        .add_argument("-i", "--input")
        .required()
        .help("specify the input B3DM file");

    program
        .add_argument("-o", "--output")
        .help("specify the GeoJSON output path");

    program
        .add_argument("--latitude")
        .help("geographic latitude of the input model");

    program
        .add_argument("--longitude")
        .help("geographic longitude of the input model");
}

struct Arguments
{
    std::string inputPath;
    std::optional<std::string> outputPath;
    double latitude = 0.0;
    double longitude = 0.0;
};

double GetDoubleArgument(argparse::ArgumentParser& program, const std::string& argumentName)
{
    auto argument = program.present(argumentName);
    if (!argument.has_value())
        return 0.0;

    try
    {
        return std::stod(*argument);
    }
    catch (const std::exception& e)
    {
        return 0.0;
    }
}

Arguments ParseArguments(argparse::ArgumentParser& program, int argc, char* argv[])
{
    try
    {
        program.parse_args(argc, argv);
    }
    catch (const std::exception& err)
    {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        std::exit(1);
    }

    Arguments arguments = {};
    arguments.inputPath = program.get("-i");
    arguments.outputPath = program.present("-o");
    arguments.latitude = GetDoubleArgument(program, "--latitude");
    arguments.longitude = GetDoubleArgument(program, "--longitude");
    return arguments;
}

} // namespace

int main(int argc, char* argv[])
{
    auto program = argparse::ArgumentParser("geozilla");
    CreateArgumentsParser(program);

    auto arguments = ParseArguments(program, argc, argv);
    auto* geoJson = GenerateGeoJson(arguments.inputPath.c_str(), arguments.latitude, arguments.longitude);
    if (geoJson)
    {
        if (arguments.outputPath)
        {
            std::ofstream output(*arguments.outputPath);
            output << geoJson;
        }
        else
        {
            std::cout << geoJson << std::endl;
        }
        FreeBuffer(geoJson);
    }

    return 0;
}
