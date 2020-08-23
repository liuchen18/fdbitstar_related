#ifndef _MOTION_PLAN
#define _MOTION_PLAN

#include "planner_core.h"
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <string>

class Motion_plan{
public:
    /**
     * @brief constructor
     */
    Motion_plan(std::string planner_name,cv::Mat *map);


private:
    cv::Mat *map;
    Planner planner;
};

#endif