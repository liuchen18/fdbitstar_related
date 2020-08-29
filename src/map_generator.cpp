#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/core.hpp>
#include <vector>
#include <iostream>
#include <cmath>
#include "map_generator.h"
#include <fstream>
#include <algorithm>

//using namespace std;
std::vector<Point2d> generate_random_map(int obs_num,int obs_size) {
    int height = 500, width = 800;
    cv::Mat map_img = cv::Mat::zeros(cv::Size(width, height), CV_8UC1);
    for(int i=0;i<height;i++){
        for(int j=0;j<width;j++){
            map_img.at<uchar>(i,j)=255;
        }
    }

    std::vector<Point2d> obs_pos(obs_num,Point2d());
    srand(time(0));
    for(int i=0;i<obs_num;i++){
        obs_pos[i].set_x(rand()%800);
        obs_pos[i].set_y(rand()%500);
        if(obs_pos[i].get_x() < 20 || obs_pos[i].get_y() < 20){
            i-=1;
            continue;
        }
        for(int m=std::max(0,(int)obs_pos[i].get_x()-obs_size);m<std::min(800,(int)obs_pos[i].get_x()+obs_size);m++){
            for(int n=std::max(0,(int)obs_pos[i].get_y()-obs_size);n<std::min(500,(int)obs_pos[i].get_y()+obs_size);n++){
                map_img.at<uchar>(n,m)=0;
            }
        }
    }

    imshow("binary map", map_img);

    cv::waitKey();
    imwrite("../graph/random_map.png", map_img);

    return obs_pos;
}

int main() {
    int obs_num=70;
    std::vector<Point2d> obs_pos=generate_random_map(obs_num,20);
    std::ofstream my_out("../graph/random_map.txt");
    my_out<<obs_num<<std::endl;
    for(auto obs:obs_pos){
        my_out<<obs.get_x()<<" "<<obs.get_y()<<" "<<40<<" "<<40<<std::endl;
    }
}
