@echo off

cd geozilla\geozilla-core

call .\download-libs.bat
call .\generate-solution.bat
call .\build-solution.bat
