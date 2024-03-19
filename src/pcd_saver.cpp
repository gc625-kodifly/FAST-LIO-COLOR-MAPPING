#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <csignal>
using namespace std;

typedef pcl::PointXYZRGB PointType;

pcl::PointCloud<PointType>::Ptr pc(new pcl::PointCloud<PointType>);
pcl::VoxelGrid<PointType>::Ptr voxel(new pcl::VoxelGrid<PointType>);

void SigHandle(int sig)
{
    ROS_INFO("Start write pcd");
    voxel->setInputCloud(pc);
    voxel->filter(*pc);
    pcl::io::savePCDFile("/home/gabriel/fast_lio_color_ws/src/FAST-LIO-COLOR-MAPPING/colored_map.pcd", *pc);
    ROS_INFO("Done");
    ros::shutdown();
}

void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg_pc)
{
    pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*msg_pc, *temp);
    *pc += *temp;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "catcher");
    ros::NodeHandle nh;
    signal(SIGINT, SigHandle);
    voxel->setLeafSize(0.01, 0.01, 0.01);
    ros::Subscriber pc_sub = nh.subscribe<sensor_msgs::PointCloud2>("/color_pc", 100, PointCloudCallback);

    ros::spin();

    return 0;
}
