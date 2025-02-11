#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud;

/* randomly create a point cloud, containing a sphere and a plane */
auto create_random_cloud(int width, int height=1){
    Cloud cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = width;
    cloud->height = height;
    cloud->points.resize(width * height);
    /* random points */
    for(size_t i=0; i<cloud->size()/2; ++i){
        cloud->points[i].x = (rand() - rand()) / (RAND_MAX + 1.0f);
		cloud->points[i].y = (rand() - rand()) / (RAND_MAX + 1.0f);
		cloud->points[i].z = (rand() - rand()) / (RAND_MAX + 1.0f);
    }
    /* sphere points: x^2+y^2+z^2=1/4 */
    for(size_t i=cloud->size()/2; i<cloud->size()/4*3; ++i){
        cloud->points[i].x = (rand() - rand()) / (RAND_MAX + 1.0f);
		cloud->points[i].y = (rand() - rand()) / (RAND_MAX + 1.0f);
        cloud->points[i].z = sqrt(0.25 
            - (cloud->points[i].x * cloud->points[i].x) 
            - (cloud->points[i].y * cloud->points[i].y));
    }
    /* plane points: x+y+z = 1/2 */
    for(size_t i=cloud->size()/4*3; i<cloud->size(); ++i){
        cloud->points[i].x = (rand() - rand()) / (RAND_MAX + 1.0f);
		cloud->points[i].y = (rand() - rand()) / (RAND_MAX + 1.0f);
        cloud->points[i].z = 0.0f
            - cloud->points[i].x
            - cloud->points[i].y;
    }
    return cloud;
}
/* use RANSAC to sample the cloud*/
auto random_sample(Cloud cloud, std::string method){
    // new cloud
    Cloud sampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // segmentation tools
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // to store inliers
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // to store segmentation params(coefficients)
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    if(method=="sphere"){

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_SPHERE); // 设置模型为球形
        seg.setMethodType(pcl::SAC_RANSAC);    // 使用 RANSAC 算法
        seg.setMaxIterations(10000);            // 最大迭代次数
        seg.setDistanceThreshold(0.005);        // 距离阈值（单位：米）
        seg.setRadiusLimits(0.3, 0.7);         // 球形半径范围（单位：米）
        seg.setInputCloud(cloud);
        cout<<"searching..."<<endl;
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.empty()) {
            std::cerr << "No model found!" << std::endl;
        }
        else{
            // output coefficient 
            std::cout << "Sphere center: (" << coefficients->values[0] << ", "
            << coefficients->values[1] << ", " << coefficients->values[2] << ")" << std::endl;
            std::cout << "Sphere radius: " << coefficients->values[3] << std::endl;
            // extract cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false); // false: normal behavior(get inliers)/ true: inverted behavior
            extract.filter(*sampled_cloud);
        }        
        
    }
    else if (method == "plane")
    {
        //todo
        
    }
    
    return sampled_cloud;
}
int main(){
    Cloud cloud = create_random_cloud(10000);


    Cloud sampled_cloud = random_sample(cloud, "sphere");
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(sampled_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (sampled_cloud, red, "downsampled_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "downsampled_cloud");
    
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (10);
        // std::this_thread::sleep_for(100ms);
    }

}

