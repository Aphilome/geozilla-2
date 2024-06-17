@echo off

echo Updating git submodules
git submodule update --init --recursive

echo Updating vcpkg
call .\external\vcpkg\bootstrap-vcpkg.bat

echo Installing vcpkg libs
call .\external\vcpkg\vcpkg install pcl --triplet=x64-windows
