#include "RRT.h"
#include "RRTstar.h"
#include <iostream>
#include <fstream>

/*int main(){
    auto map_img = cv::imread("../graph/random_map.png", 0);

    Planner *planner=new RRT(&map_img);

    planner->make_plan(10,10,500,400);
    delete planner;
    planner= nullptr;

    return 0;
}*/

int main(){
    auto map_img = cv::imread("../graph/random_map.png", 0);
    std::vector<std::vector<int>> obs_list;
    std::ifstream my_read("../graph/random_map.txt");
    int obs_num;
    my_read>>obs_num;
    while(obs_num>0){
        obs_num--;
        std::vector<int> obs(4);
        my_read>>obs[0]>>obs[1]>>obs[2]>>obs[3];
        obs_list.push_back(obs);
    }


    //Planner *planner = new RRT(&map_img,obs_list);
    Planner *planner = new RRT_STAR(&map_img,obs_list);
    std::cout<<"planner initialized!"<<std::endl;
    planner->make_plan(0,0,799,499);

    delete planner;
    planner= nullptr;

    return 0;
}


