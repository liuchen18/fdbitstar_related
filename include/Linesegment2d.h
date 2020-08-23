#ifndef _Linesegment2D
#define _Linesegment2D

#include "Point2d.h"

    class Linesegment2d{
    private:
        Point2d start_,end_,direction_unit_;
        double direction_angle;//angle to the positive x semi axis
        double length;

    public:
        /**
         * @brief constructor
         * @param start start Point
         * @param end end Point
         */
        Linesegment2d(const Point2d& start,const Point2d& end);
        
        /**
         * @brief get the start Point
         * @return start Point
         */
        const Point2d& get_start() const{return start_;}
        
        /**
         * @brief get end Point
         * @return  end Point
         */
        const Point2d& get_end() const{return end_;}
        
        /**
         * @brief get direction unit
         * @return direction unit
         */
        const Point2d& get_direction_unit() const{return direction_unit_;}

        /**
         * @brief get the x val of the direction unit
         * @return 
         */
        double get_cos()const{return direction_unit_.get_x();}

        /**
         * @brief get the y val of the direction unit
         * @return 
         */
        double get_sin()const{return direction_unit_.get_y();}

        /**
         * @brief get the length of the linesegment
         * @return 
         */
        double get_length() const{return length;}
        
        /**
         * @brief get the angle to the x axis
         * @return 
         */
        double get_angle() const{return direction_angle;}
        
        /**
         * @brief get the center of the linesegment
         * @return 
         */
        const Point2d get_center() const{return (start_+end_)/2;}
        
        /**
         * @brief compute the distance to the given Point 
         * @param given_Point 
         * @return distance
         */
        double distance_to_Point (const Point2d& given_Point) const;
        
        /**
         * @brief compute the distance to the given position
         * @param x x position
         * @param y y position
         * @return distance
         */
        double distance_to_Point (const double x, const double y) const{
            Point2d p(x,y);
            return distance_to_Point(p);
        }
        
        /**
         * @brief check whether the Point in the linesegment
         * @param given_Point 
         * @return true if in
         */
        bool is_Point_in(const Point2d& given_Point) const;
        
        /**
         * @brief check whether the given position in the linesegment
         * @param x x position
         * @param y y position
         * @return true if in
         */
        bool is_Point_in(const double x, const double y) const{
            Point2d p(x,y);
            return is_Point_in(p);
        }
        
        /**
         * @brief check whether the linesegment has intersect with another linesegment
         * @param another_line 
         * @return true if has
         */
        bool has_intersect(const Linesegment2d& another_line) const;

        /**
        @brief compute the intersect Point if there is any
        @param another_line : ahother line segment
        @param intersect_Point: the Pointer of the computed intersect Point
        @return bool, true if there is the intersect Point
        */
        bool get_intersect(const Linesegment2d& another_line, Point2d* intersect_Point) const;

        /**
        @brief compute the intersect Point if there is any. t
        @param another_line : ahother line segment
        @return Point2d, intersect. if no intersect, return double
        */
        Point2d get_intersect(const Linesegment2d& another_line) const;

    };

#endif