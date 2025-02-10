#include<iostream>
// #include<string>
// #include<map>
#include<pcl/common/common.h>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/approximate_voxel_grid.h>
#include<pcl/filters/uniform_sampling.h>
#include<pcl/filters/random_sample.h>
#include<pcl/filters/normal_space.h>
#include<pcl/features/normal_3d.h>
#include<pcl/search/kdtree.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud;

auto calculate_normals(Cloud cloud, int k=20){
    /* k is the number of points used to calculate the normal of the current point*/
    // set up normals estimator
    pcl::NormalEstimation <pcl::PointXYZ, pcl::Normal> estimator;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // use kd-tree to find knn
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    estimator.setInputCloud(cloud);
    estimator.setSearchMethod(kdtree);
    estimator.setKSearch(k);
    estimator.compute(*normals);

    return normals;


}

auto downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string method){
    Cloud downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(method=="voxel"){
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(0.01f, 0.01f, 0.01f); // 设置体素大小
        voxel.filter(*downsampled_cloud);
    }
    else if (method=="approximate voxel")
    {
        pcl::ApproximateVoxelGrid<pcl::PointXYZ> approx_voxel;
        approx_voxel.setLeafSize(0.01f, 0.01f, 0.01f);
        approx_voxel.setInputCloud(cloud);
        approx_voxel.filter(*downsampled_cloud);
    }
    else if (method=="uniform")
    {
        pcl::UniformSampling<pcl::PointXYZ> uniform;
        uniform.setInputCloud(cloud);
        uniform.setRadiusSearch(0.01); // sophere radius
        uniform.filter(*downsampled_cloud);
    }
    else if (method=="random")
    {
        pcl::RandomSample<pcl::PointXYZ> random_sample;
        random_sample.setInputCloud(cloud);
        random_sample.setSample(20000);
        random_sample.filter(*downsampled_cloud);
    }
    else if (method=="normal")
    {
        //calculate normals
        pcl::PointCloud<pcl::Normal>::Ptr normals = calculate_normals(cloud);
        // sampling
        pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> normal_space;
        normal_space.setInputCloud(cloud);
        normal_space.setNormals(normals); //calculate normals
        normal_space.setBins(10, 10, 10); //set bins number of xyz-direction to control the granularity of normals
        normal_space.setSample(5000);
        normal_space.filter(*downsampled_cloud);

    }
    return downsampled_cloud;
}


int main(){

    Cloud cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::io::loadPCDFile("/home/jiangyh/pcl_demo/data/pairwise/frame_00000.pcd", *cloud);

    /*voxel downsampling*/
    std::map<int, std::string> downsample_method = {{0, "uniform"}, 
                                                    {1, "approximate voxel"}, 
                                                    {2, "voxel"},
                                                    {3, "random"},
                                                    {4, "normal"}
                                                };

    Cloud down_cloud = downsample(cloud, downsample_method[4]);

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(down_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (down_cloud, red, "downsampled_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "downsampled_cloud");
    
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (10);
        // std::this_thread::sleep_for(100ms);
    }

}