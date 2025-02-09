#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/search/kdtree.h>
#include<pcl/visualization/pcl_visualizer.h>

#define Cloud pcl::PointCloud<pcl::PointXYZ>::Ptr
#define RGBCloud pcl::PointCloud<pcl::PointXYZRGB>::Ptr

auto copy_rgb_cloud(Cloud cloud){
    RGBCloud rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0;i<cloud->size(); i++){
        pcl::PointXYZRGB p;
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        p.r = 255;
        p.g = 0;
        p.b = 0;
        rgb_cloud->push_back(p);    
    }
    return rgb_cloud;
}
int main(){
    // init point cloud
    Cloud cloud(new pcl::PointCloud<pcl::PointXYZ>);
    RGBCloud rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // load pcd file
    pcl::io::loadPCDFile("/home/jiangyh/pcl_demo/data/bunny.pcd", *cloud);
    // init kd-tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    // set the cloud point to be searched
    // rgb_cloud = copy_rgb_cloud(cloud);
    pcl::copyPointCloud(*cloud, *rgb_cloud);
    kdtree->setInputCloud(cloud);
    // int index, distance and point 0
    std::vector<int> idx100, idx1000;
    std::vector<float> dist100, dist1000;
    pcl::PointXYZ point = cloud->points[0];
    // search the nearest k points of k;
    int k = 100;
    int size = kdtree->nearestKSearch(point, k, idx100, dist100);
    int size1000 = kdtree->nearestKSearch(point, 1000, idx1000, dist1000);
    std::cout << "search point: " << size << std::endl;
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud, 0, 255, 0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(cloud, 0, 0, 255);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZRGB> (rgb_cloud, "cloud");
    viewer->initCameraParameters ();
    int pt_idx = 0;
    while (!viewer->wasStopped ())
    {   if(pt_idx < size1000)
        {
            if(idx1000[pt_idx] < rgb_cloud->size())
            {
                rgb_cloud->points[idx1000[pt_idx]].r=0;
                rgb_cloud->points[idx1000[pt_idx]].g=255;
                pt_idx++;
                viewer->updatePointCloud(rgb_cloud, "cloud");
            }
        }
            
        viewer->spinOnce (10);
        // std::this_thread::sleep_for(100ms);
    }
    return 0;
}