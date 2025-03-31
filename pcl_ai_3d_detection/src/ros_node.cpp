#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Apply a VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud);

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = msg->header;

    pub.publish(output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_ai_node");
    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::PointCloud2>("/processed_cloud", 1);
    ros::Subscriber sub = nh.subscribe("/lidar_points", 1, pointCloudCallback);
    ros::spin();
}