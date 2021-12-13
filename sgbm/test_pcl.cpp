// #include<iostream>
// #include<pcl/io/pcd_io.h>
// #include<pcl/point_types.h>
// #include <pcl/visualization/cloud_viewer.h>
// void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
//     viewer.setBackgroundColor(1.0f, 0.5f, 1.0f);
// }

// int main(int argc, char** argv) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

//     //*打开点云文件
//     if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/cc/Downloads/rabbit.pcd", *cloud) == -1) {
//         PCL_ERROR("Couldn't read file rabbit.pcd\n");
//         return(-1);
//     }
//     std::cout << cloud->points.size() << std::endl;
//     pcl::visualization::CloudViewer viewer("cloud viewer");
//     viewer.showCloud(cloud);
//     viewer.runOnVisualizationThreadOnce(viewerOneOff);
//     while (!viewer.wasStopped()) {

//     }

//     return 0;
// }



// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>

// int
// main (int argc, char** argv)
// {
// pcl::PointCloud<pcl::PointXYZ> cloud;

// // Fill in the cloud data
// cloud.width = 5;
// cloud.height = 1;
// cloud.is_dense = false;

// for (size_t i = 0; i < cloud.points.size (); ++i)
// {
// cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
// cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
// cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
// }

// pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
// std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

// for (size_t i = 0; i < cloud.points.size (); ++i)
// std::cerr << " " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

// return 0;
// }
