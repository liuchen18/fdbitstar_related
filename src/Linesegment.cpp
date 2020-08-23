#include "Linesegment2d.h"
#include <cmath>
#include <iostream>
#include <limits>

    Linesegment2d::Linesegment2d(const Point2d& _start,const Point2d& end)
            :start_(_start),end_(end),direction_unit_(0,0){
        double dx=end_.get_x()-start_.get_x();
        double dy=end_.get_y()-start_.get_y();
        length = std::hypot(dy,dx);

        if(length > kMathEpsilon){
            direction_unit_.set_xy(dx/length,dy/length);
        }

        direction_angle=direction_unit_.get_angle();
    }

    double Linesegment2d::distance_to_Point(const Point2d& given_Point) const{
        double dx = given_Point.get_x()-start_.get_x();
        double dy = given_Point.get_y()-start_.get_y();
        double projection = dx*direction_unit_.get_x()+dy*direction_unit_.get_y();
        if(projection<0){
            return std::hypot(dx,dy);
        }
        if(projection>length){
            return end_.distance_to_point(given_Point);
        }
        double distance = std::abs(dx*direction_unit_.get_y()-dy*direction_unit_.get_x());
        return distance;
    }

    bool Linesegment2d::is_Point_in(const Point2d& given_Point) const{
        if(distance_to_Point(given_Point)<kMathEpsilon){
            return true;
        }
        return false;
    }

    bool Linesegment2d::has_intersect(const Linesegment2d& another_line) const{
        Point2d Point;
        return get_intersect(another_line,&Point);
    }

    bool Linesegment2d::get_intersect(const Linesegment2d& another_line,Point2d* intersect_Point) const{
        if(is_Point_in(another_line.get_start())){
            *intersect_Point = another_line.get_start();
            return true;
        }
        if(is_Point_in(another_line.get_end())){
            *intersect_Point = another_line.get_end();
            return true;
        }
        if(another_line.is_Point_in(start_)){
            *intersect_Point=start_;
            return true;
        }
        if(another_line.is_Point_in(end_)){
            *intersect_Point=end_;
            return true;
        }
        if(length<kMathEpsilon || another_line.length<kMathEpsilon){
            return false;
        }
        const double cc1=(end_-start_).crossproduct(another_line.get_start()-start_);
        const double cc2=(end_-start_).crossproduct(another_line.get_end()-start_);
        if(cc1*cc2>=0){return false;}
        const double cc3=(another_line.get_end()-another_line.get_start()).crossproduct(start_-another_line.get_start());
        const double cc4=(another_line.get_end()-another_line.get_start()).crossproduct(end_-another_line.get_start());
        if(cc3*cc4>=0){return false;}
        double ratio=cc4/(cc4-cc3);

        *intersect_Point=Point2d(start_.get_x()*ratio+end_.get_x()*(1-ratio),start_.get_y()*ratio+end_.get_y()*(1-ratio));
        return true;

    }

    Point2d Linesegment2d::get_intersect(const Linesegment2d& another_line) const{
        Point2d intersect_Point(std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
        if(is_Point_in(another_line.get_start())){
            intersect_Point.set_xy(another_line.get_start());
            return intersect_Point;
        }
        if(is_Point_in(another_line.get_end())){
            intersect_Point.set_xy(another_line.get_end());
            return intersect_Point;
        }
        if(another_line.is_Point_in(start_)){
            intersect_Point.set_xy(start_);
            return intersect_Point;
        }
        if(another_line.is_Point_in(end_)){
            intersect_Point.set_xy(end_);
            return intersect_Point;
        }
        if(length<kMathEpsilon || another_line.length<kMathEpsilon){
            return intersect_Point;
        }
        const double cc1=(end_-start_).crossproduct(another_line.get_start()-start_);
        const double cc2=(end_-start_).crossproduct(another_line.get_end()-start_);
        if(cc1*cc2>=0){return intersect_Point;}
        const double cc3=(another_line.get_end()-another_line.get_start()).crossproduct(start_-another_line.get_start());
        const double cc4=(another_line.get_end()-another_line.get_start()).crossproduct(end_-another_line.get_start());
        if(cc3*cc4>=0){return intersect_Point;}
        double ratio=cc4/(cc4-cc3);

        intersect_Point.set_xy(Point2d(start_.get_x()*ratio+end_.get_x()*(1-ratio),start_.get_y()*ratio+end_.get_y()*(1-ratio)));
        return intersect_Point;
    }
