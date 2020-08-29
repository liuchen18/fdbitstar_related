#include "OBB2d.h"
#include <iostream>

int main(){
    Point2d p(0,0);
    OBB2d box(20,20,0,15,15);
    //std::cout<<box.get_center().get_x()<<" center : "<<box.get_center().get_y()<<std::endl;
    std::cout<<box.is_Point_in(p)<<std::endl;
    return 0;
}