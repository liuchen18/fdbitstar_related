#include "RRT.h"

int main(){
    auto map_img = cv::imread("../graph/random_map.png", 0);
    Planner *planner=new RRT(&map_img);

    planner->make_plan(10,10,500,400);
    //planner->test();
    delete planner;
    planner= nullptr;

    return 0;
}


