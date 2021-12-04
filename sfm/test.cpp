#include <iostream>
#include <fstream>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

int main(){
    float k[3][3];
    ifstream inf;
    inf.open("./images/K.txt");
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            inf>>k[i][j];
        }
    }
    inf.close();
    Mat K = Mat(3,3,CV_32FC1,k);
    cout << "K = "<< endl << " "  << K << endl << endl;
    getchar();
    return 0;
}