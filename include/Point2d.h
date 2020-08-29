#ifndef _Point2d
#define _Point2d

#include <iostream>

constexpr double kMathEpsilon = 1e-10;

    class Point2d{
    public:
        /**
         * @brief constructor
         */
        Point2d():x_(0),y_(0){};

        /**
         * constructor
         * @param x
         * @param y
         */
        Point2d(const double x,const double y):x_(x),y_(y){};

        /**
         * @brief constructor
         * @param p
         */
        Point2d(const Point2d& p):x_(p.get_x()),y_(p.get_y()){}

        /**
         * @brief get x
         * @return
         */
        double get_x() const{return x_;}

        /**
         * @brief get y
         * @return
         */
        double get_y() const{return y_;}

        /**
         * @brief get x^2
         * @return
         */
        double get_xsquare() const {return x_*x_;}

        /**
         * @brief get y^2
         * @return
         */
        double get_ysquare() const{return y_*y_;}

        /**
         * @brief set the x to given x
         * @param x
         */
        void set_x(const double x){x_=x;}

        /**
         * @brief set the y to the given y
         * @param y
         */
        void set_y(const double y){y_=y;}

        /**
         * @brief set x and y at the same time
         * @param x
         * @param y
         */
        void set_xy(const double x,const double y){x_=x;y_=y;}

        /**
         * @brief set the x and y according to another point
         * @param another_point
         */
        void set_xy(const Point2d& another_point){x_=another_point.get_x();y_=another_point.get_y();}

        /**
         * @brief the angle to the positive x axis
         * @return
         */
        double get_angle();

        /**
         * @brief normalize the point so that the distance to the origin point is 1
         */
        void normalize();

        /**
         * @brief get the distance to the origin point
         * @return
         */
        double get_length() const;

        /**
         * @brief compute the distance to another point
         * @param another another point
         * @return distance
         */
        double distance_to_point(const Point2d& another) const;

        /**
         * @brief compute the cross product to another point
         * @param another another point
         * @return cross product
         */
        double crossproduct(const Point2d& another) const;

        /**
         * @brief compute the inner product of the given point
         * @param another
         * @return
         */
        double innerproduct(const Point2d& another) const;

        /**
         * rewrite the common operations
         * @param another
         * @return
         */
        Point2d operator + (const Point2d& another) const;
        Point2d operator - (const Point2d& another) const;
        Point2d operator * (const double ratio) const;
        Point2d operator / (const double ratio) const;
        Point2d operator += (const Point2d& another);
        Point2d operator -= (const Point2d& another);
        Point2d operator *= (const double ratio);
        Point2d operator /= (const double ratio);
        bool operator == (const Point2d& another) const;



    private:
        double x_,y_;
    };

    /**
     * @brief
     * @param ratio
     * @param point
     * @return
     */
    Point2d operator * (const double ratio,const Point2d& point);




#endif