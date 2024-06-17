@echo off

if exist "./build" (
    echo Deleting files in `build` folder
    rmdir /S /Q build
)

echo Generating Visual Studio solution
cmake -G "Visual Studio 17 2022" -B build
