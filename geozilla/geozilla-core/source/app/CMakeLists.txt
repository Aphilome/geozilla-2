set(PROJECT_NAME geozilla-app)

set(SOURCE_LIST Main.cpp)

find_package(argparse REQUIRED)

add_executable(${PROJECT_NAME} ${SOURCE_LIST})

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        geozilla-core
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        "${PROJECT_SOURCE_DIR}/source/core/"
        "${argparse_DIR}/../../include/"
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 20
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin/$<CONFIG>/"
)
