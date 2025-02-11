#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
// 新建点云存储对象
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jiangyh/pcl_demo/data/table_scene_lms400.pcd", *cloud) != 0)
    {
        cout << "failed to load pcd" << endl;
        return -1;
    }
    // 滤波对象
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    //建立搜索对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
    filter.setSearchMethod(kdtree);
    //设置搜索邻域的半径为3cm
    filter.setSearchRadius(0.03);
    // Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    // 采样的半径是
    filter.setUpsamplingRadius(0.03);
    // 采样步数的大小
    filter.setUpsamplingStepSize(0.02);
    cout << "start up sampling..." << endl;
    filter.process(*filteredCloud);
    cout << "finish up sampling." << endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (filteredCloud, "cloud");
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (10);
        // std::this_thread::sleep_for(100ms);
    }
}