#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <iostream>
#include <filesystem> // C++17 for file existence check

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    // Check command-line arguments
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> <output.pcd> [voxel_size]" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];
    float voxel_size = 0.01f; // Default voxel size

    if (argc == 4) {
        voxel_size = std::stof(argv[3]);
    }

    // Check if the input file exists
    if (!fs::exists(input_file)) {
        std::cerr << "Error: The file '" << input_file << "' does not exist." << std::endl;
        return -1;
    }

    // Check if the output directory exists
    fs::path output_path(output_file);
    if (!fs::exists(output_path.parent_path())) {
        std::cerr << "Error: The directory '" << output_path.parent_path() << "' does not exist." << std::endl;
        return -1;
    }

    // Load the input point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1) {
        std::cerr << "Error: Could not read the file '" << input_file << "'." << std::endl;
        return -1;
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << input_file << std::endl;

    // Apply a voxel grid filter for downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size); // Set the voxel size
    voxel_filter.filter(*cloud_filtered);

    std::cout << "Point cloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // Save the filtered point cloud to a new file
    if (pcl::io::savePCDFileASCII(output_file, *cloud_filtered) == -1) {
        std::cerr << "Error: Could not save the filtered point cloud to '" << output_file << "'." << std::endl;
        return -1;
    }
    std::cout << "Filtered point cloud saved to " << output_file << std::endl;

    return 0;
}