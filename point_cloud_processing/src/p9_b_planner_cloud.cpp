#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
int main()
{
    pcl::PointCloud<pcl::PointXYZ> cloud ;

    cloud.push_back(pcl::PointXYZ(1.0 ,2.0 ,3.0));
    cloud.push_back(pcl::PointXYZ(4.0 ,5.0 ,6.0));
    cloud.push_back(pcl::PointXYZ(7.0 ,8.0 ,9.0));
    cloud.push_back(pcl::PointXYZ(10.0,11.0,12.0));
    cloud.push_back(pcl::PointXYZ(12.0,14.0,-15.0));

    std::string pcd_file_name = "planez_.pcd";
	std::filesystem::path ros2_ws_path    = std::filesystem::current_path();
    std::filesystem::path point_cloud_dir = ros2_ws_path/"src/point_cloud_processing/point_clouds/";
    std::string pcd_file_path             = point_cloud_dir/pcd_file_name;

    pcl::io::savePCDFileASCII(pcd_file_path,cloud);
    std::cout<<cloud.size();


    return 0;

}