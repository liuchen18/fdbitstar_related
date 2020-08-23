#ifndef _OBB2D
#define _OBB2D

#include "Point2d.h"
#include <vector>
#include "Linesegment2d.h"
#include "AABB2d.h"

    class OBB2d
    {
    public:
        /**
         * @brief construction function
        */
        OBB2d()=default;
        virtual ~OBB2d(){};

        /**
        @brief construction function
        @param center center Point of the OBB
        @param direction_unit the vector of the direction
        @param length length along the OBB axis
        @param width length perpendicular to the OBB axis
        */
        OBB2d(const Point2d& center,const Point2d& direstion_unit,const double length,const double width);

        /**
        @brief construction function
        @param center center Point of the OBB
        @param direction_angle the angle to the positive x semi axis
        @param length length along the OBB axis
        @param width length perpendicular to the OBB axis
        */
        OBB2d(const Point2d& center,const double direction_angle,const double length,const double width);

        /**
        @brief construction function
        @param central_lineseg central line segment of the OBB, contains direction and length and center
        @param width length perpendicular to the central line segment
        */
        OBB2d(const Linesegment2d& central_lineseg, const double width);

        /**
        @brief construction function
        @param AABBbox box with the type of AABB2d
        */
        OBB2d(const AABB2d& AABBbox);

        /**
        @brief conpute all conners of the OBB
        */
        void compute_conners();

        /*get center Point*/
        Point2d get_center()const{return center_;}

        /*get angle*/
        double get_angle() const {return angle_;}

        /*get cos angle*/
        double get_cos() const{return cos_angle_;}

        /*get sin angle*/
        double get_sin() const{return sin_angle_;}

        /*get length*/
        double get_length() const{return length_;}

        /*get width*/
        double get_width() const {return width_;}

        /*get direction unit*/
        Point2d get_direction_unit() const{return direction_unit_;}

        /*get min x*/
        double get_min_x() const{return min_x_;}

        /*get max x*/
        double get_max_x() const{return max_x_;}

        /*get min y*/
        double get_min_y() const{return min_y_;}

        /*get min x*/
        double get_max_y() const{return max_y_;}

        /*set length*/
        void set_length(const double length){length_=length;}

        /*set width*/
        void set_width(const double width){width_=width;}

        /**
        @brief get the 4 conner
        @param connersptr the Pointer to save the addrass of the vector of conner
        */
        void get_conners(std::vector<Point2d> *const connersptr) const;

        /**
         * brief get the 4 conners
         * @return conners in vector
        */
        std::vector<Point2d> get_conners() const;

        /**
        @brief determine whether the given Point in the obb box
        @param given_Point
        @return bool
        */
        bool is_Point_in(const Point2d& given_Point) const;

        /**
        @brief determine whether the given Point on the boundary of the box
        #param given_Point
        return bool
        */
        bool is_Point_on_boundary(const Point2d& given_Point) const;

        /**
        @brief compute the distance to given Point from the box
        @param given_Point
        @return double distance
        */
        double distance_to_Point(const Point2d& given_Point) const;

        /**
        @brief determine whether the given line and the box bas overlap
        @param lineseg
        @return bool
        */
        bool has_overlap(const Linesegment2d& lineseg) const;

        /**
        @brief determine whether the given box and the box has overlap
        @param another_box
        @return bool
        */
        bool has_overlap(const OBB2d& another_box) const;

        /**
        @brief compute distance to line segment
        @param lineseg
        return double distance*/
        double distance_to_lineseg(const Linesegment2d& lineseg) const;

        /**
        @brief compute distance to another box
        @param another_box
        @return double distance
        */
        double distance_to_box(const OBB2d& another_box) const;

        /**
        @brief move the box along a vector
        @param vector vector to move along
        */
        void translate(const Point2d& vector);

        /**
        @brief rotate the box from center
        @param angle the angle to rotate
        */
        void rotate(const double angle);

        /**
        @brief move the box to the given center
        @param x double x of the center
        @param y double y of the center
        */
        void translate_to(const double x, const double y);

        /**
        @brief rotate the box to the given angle
        @param angle double the angle to rotate to
        */
        void rotate_to(const double angle);


    private:
        Point2d center_;
        Point2d direction_unit_;
        double cos_angle_,sin_angle_;
        double angle_;
        double length_,width_;
        double max_x_,max_y_,min_x_,min_y_;
        std::vector<Point2d> conners_;
    };







#endif