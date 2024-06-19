#include "GeozillaCore.h"
#include "Converter/GltfToPointCloudConverter.h"

#include <Logger/ConsoleLogger.h>
#include <Loader/GeoModelLoader.h>
#include <ZoneSplitter/ZoneSplitter.h>

#include <CesiumGltf/Model.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/region_growing.h>

#include <algorithm>

namespace
{

std::vector<gz::core::IGeoModelLoader::GeoModel> LoadGeoModels(const char* path)
{
    if (!path)
        return {};

    auto loader = gz::core::GeoModelLoader();
#ifdef _DEBUG
    loader.SetLogger(std::make_shared<gz::core::ConsoleLogger>());
#endif
    return loader.Load(path);
}

gz::core::GltfToPointCloudConverter::Points::Ptr ConvertToPointCloud(const std::vector<gz::core::IGeoModelLoader::GeoModel>& models)
{
    using namespace gz::core;

    auto pointCloud = std::make_shared<GltfToPointCloudConverter::Points>();

    std::for_each(std::begin(models), std::end(models), [&pc = *pointCloud](const auto& model)
    {
        gz::core::GltfToPointCloudConverter::Convert(model, pc);
    });

    return pointCloud;
}

const char* ConvertToRawMemory(const std::string& data)
{
    const auto size = data.size();
    auto* buffer = new char[size + 1];
    buffer[size] = '\0';
    std::copy(std::cbegin(data), std::cend(data), buffer);
    return buffer;
}

} // namespace

const char* GenerateGeoJson(const char* path, float latitude, float longitude)
{
    auto models = LoadGeoModels(path);
    auto cloud = ConvertToPointCloud(models);

    {
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(cloud);
        normal_estimator.setKSearch(50);
        normal_estimator.compute(*normals);

        pcl::IndicesPtr indices(new std::vector <int>);
        pcl::removeNaNFromPointCloud(*cloud, *indices);

        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
        reg.setMinClusterSize(50);
        reg.setMaxClusterSize(1000000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(30);
        reg.setInputCloud(cloud);
        reg.setIndices(indices);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold(1.0);

        std::vector <pcl::PointIndices> clusters;
        reg.extract(clusters);

        //pcl::io::savePCDFileASCII("cloud.pcd", *cloud);

        for (size_t i = 0; i < clusters.size(); ++i)
        {
            auto c = std::make_shared<gz::core::GltfToPointCloudConverter::Points>();
            for (auto j : clusters[i].indices)
            {
                c->push_back(cloud->points[j]);
            }

            pcl::visualization::CloudViewer viewer1("Cluster viewer");
            viewer1.showCloud(c);
            while (!viewer1.wasStopped())
            {
            }

            //pcl::io::savePCDFileASCII("cluster" + std::to_string(i) + ".pcd", *c);
        }

        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
        pcl::visualization::CloudViewer viewer("Cluster viewer");
        viewer.showCloud(colored_cloud);
        while (!viewer.wasStopped())
        {
        }

        std::cout << clusters.size() << std::endl;
    }

    auto splitter = ZoneSplitter();
    std::string result = "{}";//splitter.SplitToZones(pointCloud);
    return ConvertToRawMemory(result);
}

void FreeBuffer(const char* buffer)
{
    delete[] buffer;
}
