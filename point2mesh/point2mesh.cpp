#include <iostream>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h> //最小二乘 重采样平滑
#include <pcl/surface/poisson.h>  //泊松重建
#include <pcl/geometry/polygon_mesh.h> //MESH
#include <pcl/surface/gp3.h>  //贪心三角形
#include<pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;


void DownSampledPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out);
void OutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out);
void SmoothPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out);
pcl::PointCloud<pcl::Normal>::Ptr CalculateNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in);

/*
贪婪三角化投影曲面重建
计算流程：点云输入 --> 下采样 --> 统计滤波去除离群点 --> mls移动最小二乘法进行平滑处理 --> 对平滑后的点云进行法线估计(有向点云) --> 将法线和平滑后的点云的Fields拼接在一起 --> 贪婪三角化 -->显示输出
*/
int main(){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>); //创建存放读取点云的对象
    pcl::io::loadPCDFile("/home/cc/Documents/3d_reconstruction/sgbm/build/pointcloud.pcd", *cloud_in);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out_obj(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
    DownSampledPoints(cloud_in,cloud_out_obj);
    OutlierRemoval(cloud_out_obj, cloud_out_obj);
    SmoothPointcloud(cloud_out_obj, cloud_out);
    

    // cloud_out->height = 1;
	// cloud_out->width = cloud_out->points.size();
	// cout << "point cloud size = " << cloud_out->points.size() << endl;
	// cloud_out->is_dense = false;
	// pcl::visualization::CloudViewer viewer("viewer");
	// viewer.showCloud(cloud_out);
	// while (!viewer.wasStopped())
	// {

	// }

    pcl::StopWatch time;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    normals = CalculateNormal(cloud_out);
	std::cout << "begin  mesh..." << std::endl;
    // 将点云位姿、颜色、法线信息连接到一起
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*cloud_out, *normals, *cloud_with_normals);

    //定义搜索树对象
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud_with_normals);

    // 三角化
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;   // 定义三角化对象
	pcl::PolygonMesh triangles; //存储最终三角化的网络模型

    // 设置三角化参数
	gp3.setSearchRadius(0.1);  //设置搜索时的半径，也就是KNN的球半径
	gp3.setMu(2.5);  //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100);    //设置样本点最多可搜索的邻域个数，典型值是50-100

	gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到的三角形内角的最小的角度为10°
	gp3.setMaximumAngle(2 * M_PI / 3); // 设置三角化后得到的三角形内角的最大角度为120°

	gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
	gp3.setNormalConsistency(false);  //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

	gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
	gp3.setSearchMethod(tree2);   //设置搜索方式
	gp3.reconstruct(triangles);  //重建提取三角化
	cloud_with_normals->width = cloud_with_normals->height = 0;
	std::cout << "success triangles, time(s) "<< time.getTimeSeconds() << std::endl;

    pcl::io::savePLYFile("./triangles.ply",triangles);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh(triangles,"GPTriangle");
    viewer->addText("GPTriangle",0,0,"GPTriangle");
    viewer->initCameraParameters ();
 
    while (!viewer->wasStopped ())
    {
        //viewer->spinOnce (100);
    }

    return 0;
}


//点云下采样
void DownSampledPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out)
{
    // 下采样，同时保持点云形状特征
    pcl::VoxelGrid<pcl::PointXYZRGB> downSampled;  //创建滤波对象
    downSampled.setInputCloud (cloud_in);            //设置需要过滤的点云给滤波对象
    downSampled.setLeafSize (0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
    downSampled.filter (*cloud_out);           //执行滤波处理，存储输出
    std::cout<<"cloud_downSampled: " << cloud_out->size()<<std::endl;
}

//统计滤波去除离群点
void OutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out)
{
    // 统计滤波
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statisOutlierRemoval;       //创建滤波器对象
    statisOutlierRemoval.setInputCloud (cloud_in);            //设置待滤波的点云
    statisOutlierRemoval.setMeanK (50);                                //设置在进行统计时考虑查询点临近点数
    statisOutlierRemoval.setStddevMulThresh (3.0);                     //设置判断是否为离群点的阀值:均值+1.0*标准差
    statisOutlierRemoval.filter (*cloud_out);                     //滤波结果存储到cloud_filtered
 
    std::cout << "cloud_statical_filtered: " << cloud_out->size()<<std::endl;
}

//重采样平滑点云
void SmoothPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_out)
{
	// 对点云重采样 
	std::cout<<"begin smooth: size " << cloud_in->size() << std::endl;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZRGB>); // 创建用于最近邻搜索的KD-Tree
	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;  // 定义最小二乘实现的对象mls
	mls.setSearchMethod(treeSampling);    // 设置KD-Tree作为搜索方法
	mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线
	mls.setInputCloud(cloud_in);        //设置待处理点云
	mls.setPolynomialOrder(2);             // 拟合2阶多项式拟合
	//mls.setPolynomialFit(true);  // 设置为false可以 加速 smooth
	mls.setSearchRadius(0.05); // 单位m.设置用于拟合的K近邻半径
	mls.process(*cloud_out);        //输出
	std::cout << "success smooth, size: " << cloud_out->size() << std::endl;

}

//计算法线
pcl::PointCloud<pcl::Normal>::Ptr CalculateNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in)
{
	// 法线估计
	pcl::StopWatch time;
	std::cout << "begin compute normal.... " << std::endl;
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;                    //创建法线估计的对象
	normalEstimation.setInputCloud(cloud_in);                                    //输入点云
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);          // 创建用于最近邻搜索的KD-Tree
	normalEstimation.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);     // 定义输出的点云法线
																					 // K近邻确定方法，使用k个最近点，或者确定一个以r为半径的圆内的点集来确定都可以，两者选1即可
	normalEstimation.setKSearch(10);                    // 使用当前点周围最近的10个点
														//normalEstimation.setRadiusSearch(0.03);            //对于每一个点都用半径为3cm的近邻搜索方式
	normalEstimation.compute(*normals);                 //计算法线

	std::cout << "success compute normal，time（s）:  "<< time.getTimeSeconds() << std::endl;
	return normals;
}
