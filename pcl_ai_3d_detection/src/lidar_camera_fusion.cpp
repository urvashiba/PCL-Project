#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

void fuseSensors(pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cameraCloud) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // Example: Set transformation values (rotation + translation)
    transform(0, 3) = 0.1;  // Translation along X
    transform(1, 3) = 0.2;  // Translation along Y
    transform(2, 3) = 0.3;  // Translation along Z

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCameraCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cameraCloud, *transformedCameraCloud, transform);

    *lidarCloud += *transformedCameraCloud;
}