#include "Point2d.h"
#include <cmath>

    double Point2d::get_angle(){return std::atan2(y_,x_);}

    void Point2d::normalize(){
        double l=get_length();
        if(l>kMathEpsilon){
            x_=x_/l;
            y_=y_/l;
        }
    }

    double Point2d::get_length() const{
        return std::hypot(x_,y_);
    }

    double Point2d::distance_to_point(const Point2d& another) const{
        return std::hypot(x_-another.get_x(),y_-another.get_y());
    }

    double Point2d::crossproduct(const Point2d& another) const{
        return x_*another.get_y()-y_*another.get_x();
    }

    double Point2d::innerproduct(const Point2d& another) const{
        return x_*another.get_x()+y_*another.get_y();
    }

    Point2d Point2d::operator + (const Point2d& another) const{
        return Point2d(x_+another.get_x(),y_+another.get_y());
    }

    Point2d Point2d::operator - (const Point2d& another) const{
        return Point2d(x_-another.get_x(),y_-another.get_y());
    }

    Point2d Point2d::operator * (const double ratio) const{
        return Point2d(x_*ratio,y_*ratio);
    }

    Point2d Point2d::operator / (const double ratio) const{
        return Point2d(x_/ratio,y_/ratio);
    }

    Point2d Point2d::operator += (const Point2d& another){
        x_+=another.get_x();
        y_+=another.get_y();
        return *this;
    }

    Point2d Point2d::operator -= (const Point2d& another){
        x_-=another.get_x();
        y_-=another.get_y();
        return *this;
    }

    Point2d Point2d::operator *= (const double ratio){
        x_*=ratio;
        y_*=ratio;
        return *this;
    }

    Point2d Point2d::operator /=(const double ratio){
        x_/=ratio;
        y_/=ratio;
        return *this;
    }

    bool Point2d::operator == (const Point2d& another) const{
        return std::abs(x_-another.get_x())<kMathEpsilon && std::abs(y_-another.get_y())<kMathEpsilon;
    }

    Point2d operator * (const double ratio,const Point2d& Point){
        return Point2d(ratio*Point.get_x(),ratio*Point.get_y());
    }
