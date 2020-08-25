#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/core.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

//using namespace std;
void generate_random_map(int obs_num,int obs_size) {
    int height = 500, width = 800;
    cv::Mat map_img = cv::Mat::zeros(cv::Size(width, height), CV_8UC1);
    for(int i=0;i<height;i++){
        for(int j=0;j<width;j++){
            map_img.at<uchar>(i,j)=255;
        }
    }

    imshow("binary map", map_img);
    cv::waitKey();

    std::vector<std::vector<int>> obs_pos(obs_num,std::vector<int>(2));
    srand(time(0));
    for(int i=0;i<obs_num;i++){
        obs_pos[i][0]=rand()%500;
        obs_pos[i][1]=rand()%800;
        if(obs_pos[i][0] < 20 || obs_pos[i][1] < 20){
            i-=1;
            continue;
        }
        for(int m=std::max(0,obs_pos[i][0]-obs_size);m<std::min(500,obs_pos[i][0]+obs_size);m++){
            for(int n=std::max(0,obs_pos[i][1]-obs_size);n<std::min(800,obs_pos[i][1]+obs_size);n++){
                map_img.at<uchar>(m,n)=0;
            }
        }
    }

    imshow("binary map", map_img);

    cv::waitKey();
    imwrite("random_map.png", map_img);
}

int main(){
    generate_random_map(70,20);
    return 0;
}