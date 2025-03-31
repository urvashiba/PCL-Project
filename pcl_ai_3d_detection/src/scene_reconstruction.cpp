#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <iostream>

pcl::PointCloud<pcl::PointXYZ>::Ptr alignPointClouds(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) 
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*final);

    if (icp.hasConverged()) {
        std::cout << "ICP converged. Fitness score: " << icp.getFitnessScore() << std::endl;
    } else {
        std::cerr << "ICP did not converge." << std::endl;
    }

    return final;
}