set(PROJECT_NAME geozilla-core)

set(SOURCE_LIST
    DllMain.cpp

    GeoTypes.h

    GeozillaCore.h
    GeozillaCore.cpp

    # Logger
    Logger/ILogger.h
    Logger/ConsoleLogger.h
    Logger/ConsoleLogger.cpp

    # Loader
    Loader/IGeoModelLoader.h
    Loader/GeoModelLoaderBase.h
    Loader/GeoModelLoaderBase.cpp
    Loader/GeoModelLoader.h
    Loader/GeoModelLoader.cpp
    Loader/B3dmLoader.h
    Loader/B3dmLoader.cpp

    # Algorithm
    Algorithm/GltfToPointCloudConverter.h
    Algorithm/GltfToPointCloudConverter.cpp
    Algorithm/ConcaveHullGenerator.h
    Algorithm/ConcaveHullGenerator.cpp
    Algorithm/GeoJsonGenerator.h
    Algorithm/GeoJsonGenerator.cpp

    # ZoneSplitter
    ZoneSplitter/ZoneSplitter.h
    ZoneSplitter/ZoneSplitter.cpp
)

find_package(PCL REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_library(${PROJECT_NAME} SHARED ${SOURCE_LIST})

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PCL_VISUALIZATION_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        CesiumGltf
        Cesium3DTilesContent
        nlohmann_json
        ${PCL_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 20
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin/$<CONFIG>/"
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_LIST})
