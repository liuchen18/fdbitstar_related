#ifndef _PLANNER
#define _PLANNER

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include "Point2d.h"

class Planner{
public:
    /**
     * @brief constructor
     */
    Planner(){}

    /**
     * @brief destructor
     */
    virtual ~Planner(){}

    /**
     * @brief make plan
     * @param start
     * @param end
     * @return true if got plan successfully
     */
     virtual bool make_plan(Point2d start, Point2d end)=0;

     /**
      * @brief make plan
      * @param start_x
      * @param start_y
      * @param end_x
      * @param end_y
      * @return true if got plan successfully
      */
     virtual bool make_plan(double start_x,double start_y,double end_x,double end_y){
         Point2d s(start_x,start_y),e(end_x,end_y);
         return make_plan(s,e);
     }

     /**
      * @brief clear the path and others
      * @return true if done
      */
     //virtual bool clear_all()=0;
    /**
     * @brief set map
     * @param map
     * @param obs_list
     */
     virtual void set_map(cv::Mat *map,std::vector<std::vector<int>> obs_list)=0;

    /**
    * @brief show the env and the path
    */
     virtual void show_graph()=0;

};

#endif