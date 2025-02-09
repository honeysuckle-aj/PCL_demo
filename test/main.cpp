#include <iostream>
#include <thread>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>


auto make_z_rotate(float theta, Eigen::Matrix4f transform){
    // Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = cos(theta);
    transform(0, 1) = -sin(theta);
    transform(1, 0) = sin(theta);
    transform(1, 1) = cos(theta);
    return transform;
}

auto make_x_rotate(float theta, Eigen::Matrix4f transform){
    // Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(1, 1) = cos(theta);
    transform(1, 2) = -sin(theta);
    transform(2, 1) = sin(theta);
    transform(2, 2) = cos(theta);
    return transform;
}

auto make_y_rotate(float theta, Eigen::Matrix4f transform){
    // Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0, 0) = cos(theta);
    transform(2, 0) = -sin(theta);
    transform(0, 2) = sin(theta);
    transform(2, 2) = cos(theta);
    return transform;
}

int main(int argc, char **argv) {

    // 创建PointCloud的智能指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 加载pcd文件到cloud
    pcl::PLYReader Reader;
    Reader.read("./data/happyStandRight_0.ply", *cloud);
    // pcl::io::loadPCDFile("./data/happyStandRight_0.ply", *cloud);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud");
    viewer->initCameraParameters ();

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity ();
    // transform = make_z_rotate(M_PI/180, transform);
    // transform = make_x_rotate(M_PI/180, transform);
    transform = make_y_rotate(M_PI/180, transform);
    while (!viewer->wasStopped ())
    {
        pcl::transformPointCloud(*cloud, *cloud, transform);
        viewer->updatePointCloud(cloud, "cloud");
        viewer->spinOnce (10);
        // std::this_thread::sleep_for(100ms);
    }
    // std::cout << "points size: "<< cloud->points.size()<<endl;
    // for (size_t i = 0; i < cloud->points.size (); ++i)
    // std::cout << "    " << cloud->points[i].x
    //           << " "    << cloud->points[i].y
    //           << " "    << cloud->points[i].z << std::endl;
    return 0;
}