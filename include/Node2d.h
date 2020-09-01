#ifndef _NODE2D
#define _NODE2D

#include "Point2d.h"

class Node2d{
public:
    /**
     * @brief constructor
     * @param x
     * @param y
     * @param parent
     */
    Node2d(double x,double y, Node2d *parent= nullptr):_point(x,y),_parent(parent),_cost(0){}

    /**
     * @brief constructor
     * @param point
     * @param parent
     */
    Node2d(Point2d point, Node2d *parent= nullptr):_point(point),_parent(parent),_cost(0){}

    /**
     * @brief get x
     * @return
     */
    double get_x() const{return _point.get_x();}

    /**
     * @brief get y
     * @return
     */
    double get_y() const{return _point.get_y();}

    /**
     * @brief get parent
     * @return
     */
    Node2d* get_parent() const{return _parent;}

    /**
     * @brief get position of the node
     * @return
     */
    Point2d get_position()const{return _point;}

    /**
     * @brief set x
     * @param x
     */
    void set_x(const double x){_point.set_x(x);}

    /**
     * @brief set y
     * @param y
     */
    void set_y(const double y){_point.set_y(y);}

    /**
     * @brief set parent
     * @param parent
     */
    void set_parent(Node2d *parent){_parent = parent;}

    /**
     * @brief get the current cost of the node
     * @return
     */
    double get_cost() const{return _cost;}

    /**
     * @brief set the cost to the given cost
     * @param cost
     */
    void set_cost(double cost){_cost=cost;}

private:
    Point2d _point;
    Node2d *_parent;
    double _cost;
};

#endif
