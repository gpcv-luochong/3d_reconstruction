#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <iostream>
#include <vector>
#include<opencv2/core.hpp>
#include <opencv2/opencv.hpp>  
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<pcl/visualization/cloud_viewer.h>


using namespace cv;
using namespace std;
using namespace Eigen;

// 定义点云类型
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// void disp2Depth(cv::Mat dispMap, cv::Mat &depthMap, cv::Mat K);
// void insertDepth32f(cv::Mat depth,cv::Mat &depth1);

int main(){
    Mat image_l=imread("/home/cc/Documents/3d_reconstruction/stereorectify/image1.png");  //此处的..或者是.取决于sgbm.out文件在哪
    Mat image_r=imread("/home/cc/Documents/3d_reconstruction/stereorectify/image2.png");
    //SGBM参数
   
    int numberOfDisparities = ((image_l.size().width / 8) + 15) & -16;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
    sgbm->setPreFilterCap(32);
    int SADWindowSize = 9;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = image_l.channels();
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

    Mat disp,disp8,disp_refined;
    sgbm->compute(image_l, image_r, disp);
    	
    //sgbm直接计算出的disp属于CV_16UC1类型，转换成CV_8UC1
    //disp.convertTo(disp8,CV_8U,255 / (numberOfDisparities*16.));//转8位
    disp.convertTo(disp, CV_32F, 1.0 / 16.0f); //将矩阵disparity_sgbm转换为括号中的格式(32位空间的单精度浮点型矩阵)

    normalize(disp, disp8, 0, 255, NORM_MINMAX, CV_8UC1);
    //namedWindow("disparity",0);
    //imshow("disparity", disp8);
    imwrite("./disparity.png",disp8);

    //计算深度图，由于基线baseline距离未知，只能求归一化的深度值，即设baseline=1.0
    // Mat depth_map,depth_map_8;
    // Mat K = ( Mat_<double> ( 3,3 ) << 2905.88, 0, 1416, 0, 2905.88, 1064, 0, 0, 1 );
    // disp2Depth(disp8, depth_map, K);
    // imwrite("./depthmap.png",depth_map);
    // normalize(depth_map, depth_map_8, 0, 255, NORM_MINMAX, CV_8UC1);
    
    // insertDepth32f(disp8,disp_refined);
    // imwrite("./disp_refined.png",disp_refined);

    //转点云
    //相机坐标系下的点云
	PointCloud::Ptr cloud(new PointCloud);
   
    Mat K = ( Mat_<double> ( 3,3 ) << 2905.88, 0, 1416, 0, 2905.88, 1064, 0, 0, 1 );
    // float fx = K.at<float>(0, 0);
    // float fy = K.at<float>(1, 1);
    // float cx = K.at<float>(0, 2);
    // float cy = K.at<float>(1, 2);
    float fx = 2905.88;
    float fy = 2905.88;
    float cx = 1416;
    float cy = 1064;
    float b = 0.2; //基线距离
    for (int i = 0; i < disp.rows; i++){
		for (int j = 0; j < disp.cols; j++)
        {
            if (disp.at<float>(i,j)<= 36 || disp.at<float>(i,j) >= 256) continue;
            //d 存在值，则向点云增加一个点
			PointT p;
			double x = (j - cx) / fx;      //像素坐标转换为归一化坐标
            double y = (i - cy) / fy;
            double depth = fx * b / (disp.at<float>(i,j)); 
            p.x = x * depth ;
            p.y = y * depth ;
            p.z = depth  ;

            p.y = -p.y;
            p.z = -p.z;
            
            
			p.b = image_l.ptr<uchar>(i)[j * 3];
			p.g = image_l.ptr<uchar>(i)[j * 3 + 1];
			p.r = image_l.ptr<uchar>(i)[j * 3 + 2];
            
            //把p加入到点云中
            cloud->points.push_back(p);
        }    
    }
    cloud->height = 1;
	cloud->width = cloud->points.size();
	cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;

	pcl::visualization::CloudViewer viewer("viewer");
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{

	}
    pcl::io::savePCDFile("./pointcloud.pcd",*cloud);

    waitKey(0);
    return 0;
}


/*
函数作用：视差图转深度图
输入：
　　dispMap ----视差图，8位单通道，CV_8UC1
　　K       ----内参矩阵，float类型
输出：
　　depthMap ----深度图，16位无符号单通道，CV_16UC1
*/
// void disp2Depth(cv::Mat dispMap, cv::Mat &depthMap, cv::Mat K)
// {
//     int type = dispMap.type();
//     float fx = K.at<float>(0, 0);
//     float fy = K.at<float>(1, 1);
//     float cx = K.at<float>(0, 2);
//     float cy = K.at<float>(1, 2);
//     float baseline = 1.0; //基线距离65mm
//     if (type == CV_8UC1)
//     {
//         depthMap.create(dispMap.rows,dispMap.cols,CV_8UC1);
//         cv::Mat depth1 = cv::Mat(dispMap.rows,dispMap.cols,CV_16S);
//         for (int i = 0;i < dispMap.rows;i++)
//         {
//             for (int j = 0;j < dispMap.cols;j++)
//             {
//                 if (!dispMap.ptr<ushort>(i)[j])//防止除0中断
//                     continue;
//                 depth1.ptr<ushort>(i)[j] = fx * baseline / dispMap.ptr<ushort>(i)[j];
//             }
//         }
//         depth1.convertTo(depthMap,CV_8U,1./256);//转8位
//     }
// }

//空洞填充部分
//实际测试效果不好
// void insertDepth32f(cv::Mat depth, cv::Mat &depth1)
// {
//     const int width = depth.cols;
//     const int height = depth.rows;
    
//     cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
//     cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
   
  
//     for (int i = 0; i < height; ++i)
//     {
//         for (int j = 0; j < width; ++j)
//         {
//             if (depth.at<float>(i, j) > 1e-3)
//             {
//                 integralMap.at<float>(i, j) = depth.at<float>(i, j);
//                 ptsMap.at<float>(i, j) = 1;
//             }
           
//         }
//     }
//     //积分区间
//     for (int i = 0; i < height; ++i)
//     {
//         for (int j = 1; j < width; ++j)
//         {
//             integralMap.at<float>(i, j) += integralMap.at<float>(i, j-1);
//             ptsMap.at<float>(i, j) += ptsMap.at<float>(i, j-1);
//         }
//     }
//     for (int i = 1; i < height; ++i)
//     {
//         for (int j = 0; j < width; ++j)
//         {
//             integralMap.at<float>(i, j) += integralMap.at<float>(i-1, j);
//             ptsMap.at<float>(i, j) += ptsMap.at<float>(i-1, j);
//         }
//     }
//     int wnd;
//     double dWnd = 100;
//     while (dWnd > 1)
//     {
//         wnd = int(dWnd);
//         dWnd /= 2;
//         for (int i = 0; i < height; ++i)
//         {
//             for (int j = 0; j < width; ++j)
//             {
//                 int left = j - wnd - 1;
//                 int right = j + wnd;
//                 int top = i - wnd - 1;
//                 int bot = i + wnd;
//                 left = max(0, left);
//                 right = min(right, width - 1);
//                 top = max(0, top);
//                 bot = min(bot, height - 1);
//                 int dx = right - left;
//                 int dy = (bot - top) * width;
//                 int idLeftTop = top * width + left;
//                 int idRightTop = idLeftTop + dx;
//                 int idLeftBot = idLeftTop + dy;
//                 int idRightBot = idLeftBot + dx;
//                 int ptsCnt = integralMap.at<float>(bot, right) + integralMap.at<float>(top, left) - (integralMap.at<float>(bot, left) + integralMap.at<float>(top, right));
//                 double sumGray = ptsMap.at<float>(bot, right) + ptsMap.at<float>(top, left) - (ptsMap.at<float>(bot, left) + ptsMap.at<float>(top, right));
//                 if (ptsCnt <= 0)
//                 {
//                     continue;
//                 }
//                 depth.at<float>(i, j) = float(sumGray / ptsCnt);
//             }
//         }
//         int s = wnd / 2 * 2 + 1;
//         if (s > 201)
//         {
//             s = 201;
//         }
//         cv::GaussianBlur(depth, depth1, cv::Size(s, s), s, s);
//     }
// }


