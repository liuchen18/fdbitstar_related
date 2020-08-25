#ifndef _AABB2D
#define _AABB2D

#include "Point2d.h"
#include <vector>

    class AABB2d{
    private:
        Point2d center_;
        double length_,width_; //length: x axis;width: y axis
        Point2d conner1_,conner2_,conner3_,conner4_;

    public:
        /*construction function*/
        AABB2d()=default;
        AABB2d(const Point2d& center, const double length, const double width);
        AABB2d(const Point2d& conner1,const Point2d& conner3);

        /**
         * @brief get center of the box
         * @return
         */
        Point2d get_center() const{return center_;}
        double get_length() const {return length_;}
        double get_width() const {return width_;}
        double get_area() const {return width_*length_;}
        double get_min_x() const{return center_.get_x()-length_/2;}
        double get_max_x() const{return center_.get_x()+length_/2;}
        double get_min_y() const{return center_.get_y()-width_/2;}
        double get_max_y() const{return center_.get_y()+width_/2;}

        /*return 4 conners of the box
        @param conners is the Pointer to the vector which contains the conners*/
        bool get_all_conners(std::vector<Point2d> * conners) const;

        /*decide whether the given Point in the box*/
        bool is_Point_in(const Point2d& given_Point)const;

        /*decide whether the given Point on the boundary of the box*/
        bool is_Point_on_boundary(const Point2d& given_Point) const;

        /*compute distance to given Point*/
        double distance_to_Point(const Point2d& given_Point) const;

        /*compute distance to another box */
        double distance_to_box(const AABB2d& another_box) const;

        /*determine whether there is overlap*/
        bool has_overlap(const AABB2d& another_box) const;

        /*move the box
        @param vector is the vector we wish how to move the box*/
        void move(const Point2d& vector);

        /*merge the box with another box*/
        void merge_with_box(const AABB2d& box);

        /*merge the box with another Point*/
        void merge_with_Point(const Point2d& given_Point);





    };

#endif