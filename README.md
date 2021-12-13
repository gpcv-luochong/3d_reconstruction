# 3d_reconstruction
A complete pipeline of 3d reconstruction
一个完整的三维重建流程，从两张随意拍摄的图片开始，利用相机内参矩阵，重建出场景的三维模型。
包括stereorectify（核线校正），sgbm(立体匹配，生成视差图，再转点云模型)， point2mesh（点云模型转基于mesh的模型）
在Ubuntu下安装cmake,opencv,pcl，终端运行mkdir build，cd build, cmake .., make, ./xxx 
