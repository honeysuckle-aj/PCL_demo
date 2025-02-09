#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/common/centroid.h>
#include<pcl/point_cloud.h>
#include<pcl/octree/octree.h>
#include <pcl/visualization/pcl_visualizer.h>

#define Cloud pcl::PointCloud<pcl::PointXYZ>::Ptr
#define RGBCloud pcl::PointCloud<pcl::PointXYZRGB>::Ptr

auto copy_rgb_cloud(Cloud cloud){
    RGBCloud rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0;i<cloud->size(); i++){
        pcl::PointXYZRGB p;
        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;
        p.r = 0;
        p.g = 0;
        p.b = 0;
        rgb_cloud->push_back(p);    
    }
    return rgb_cloud;
}

int main(){
    /* randomly create a cloud*/
    // Cloud cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // // load pcd file
    // pcl::io::loadPCDFile("/home/jiangyh/pcl_demo/data/table_scene_lms400.pcd", *cloud);
    RGBCloud rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    rgb_cloud->width = 10000;
	rgb_cloud->height = 1;
	rgb_cloud->points.resize(rgb_cloud->width * rgb_cloud->height);
    for (size_t i = 0; i < rgb_cloud->size(); ++i)
	{
		rgb_cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		rgb_cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		rgb_cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
        rgb_cloud->points[i].r = rgb_cloud->points[i].g = rgb_cloud->points[i].b = 0;
	}

    // init octree
    float resolution = 128;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution);
    // input cloud
    octree.setInputCloud(rgb_cloud);
    // set up octree
    octree.addPointsFromInputCloud();

    /**  start to search **/ 
    // get the centroid of cloud
    Eigen::Vector4f pt_v;
    pcl::compute3DCentroid(*rgb_cloud, pt_v);
    pcl::PointXYZRGB pt(pcl::PointXYZRGB(pt_v[0], pt_v[1], pt_v[2], 0, 0, 0));
    std::cout << "centroid of the cloud: " << pt << std::endl;
    /* voxel search*/
    // voxel search will output the points that locate in the same voxel with the input point
    std::vector<int> idx_voxel;
    int n_voxel = octree.voxelSearch(rgb_cloud->points[100], idx_voxel);
    if(n_voxel > 0)
    {   
        std::cout << idx_voxel.size() << std::endl; 
        for(int i=0; i<idx_voxel.size(); i++)
        {
            rgb_cloud->points[idx_voxel[i]].g = 255;
        }
    }
    /* KNN */
    int k = 50;
    std::vector<int> idx_knn;
    std::vector<float> dist_knn;
    int n_knn = octree.nearestKSearch(pt, k, idx_knn, dist_knn);
    // draw the knn as red
    if(n_knn > 0)
    {   
        std::cout << n_knn << std::endl; 
        for(int i=0; i<n_knn; i++)
        {
            rgb_cloud->points[idx_knn[i]].r = 255;
            rgb_cloud->points[idx_knn[i]].g = rgb_cloud->points[idx_knn[i]].b = 0;
        }
    }

    /* Radius */
    float radius = 300;
    std::vector<int> idx_r;
    std::vector<float> dist_r;
    int n_radius = octree.radiusSearch(pt, radius, idx_r, dist_r);
    //draw the radius as blue
    if(n_radius > 0)
    {
        std::cout << n_radius << std::endl;
        for(int i=0; i<n_radius; i++)
        {
            rgb_cloud->points[idx_r[i]].b = 125;
            // rgb_cloud->points[idx_knn[i]].g = rgb_cloud->points[idx_knn[i]].r = 0;
        }
    }
    /* visualization */
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    
    viewer->setBackgroundColor (255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZRGB> (rgb_cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (10);
        // std::this_thread::sleep_for(100ms);
    }
    return 0;
    
}