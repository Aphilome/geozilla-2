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
        .add_argument("-visualize", "--visualize")
        .flag()
        .help("Visualize point clouds");
}

struct Arguments
{
    std::string inputPath;
    std::optional<std::string> outputPath;
    bool visualize;
};

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
    arguments.visualize = program.is_used("-visualize");

    return arguments;
}

} // namespace

int main(int argc, char* argv[])
{
    auto program = argparse::ArgumentParser("geozilla");
    CreateArgumentsParser(program);

    auto arguments = ParseArguments(program, argc, argv);
    auto geoJson = GenerateGeoJson(arguments.inputPath, arguments.visualize);
    if (arguments.outputPath)
    {
        std::ofstream output(*arguments.outputPath);
        output << geoJson;
    }
    else
    {
        std::cout << geoJson << std::endl;
    }

    return 0;
}
