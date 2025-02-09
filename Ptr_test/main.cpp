#include <iostream>
#include <thread>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

int main(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::io::loadPCDFile("/home/jiangyh/pcl_demo/data/bunny.pcd", *cloud);
    pcl::copyPointCloud(*cloud, *rgb_cloud);
    pcl::copyPointCloud(*cloud, *cloud_2);
    std::cout << cloud->points[0].x << std::endl;
    cloud->points[0].x = 10;
    std::cout << cloud->points[0].x << " " << rgb_cloud->points[0].x<< " " << cloud_2->points[0].x << std::endl;
    return 0;
}