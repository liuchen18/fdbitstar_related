#include "motion_plan.h"

int main(){
    cv::Mat graph=cv::imread("../graph/random_map.png",0);
    imshow("binary map", graph);
    cv::waitKey();

    cv::Mat *map=&graph;
    std::string planner_name="RRT";
    //Motion_plan motion_plan(planner_name,map);

    //motion_plan.plan()

    return 0;
}