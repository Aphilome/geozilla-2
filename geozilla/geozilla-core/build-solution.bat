@echo off

echo Building `Debug` solution
cmake --build build --config Debug

echo Building `Release` solution
cmake --build build --config Release
