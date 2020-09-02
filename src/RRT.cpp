#include "RRT.h"
#include <algorithm>
#include <cmath>
#include <random>

RRT::~RRT() {
    clear_tree();
    clear_obs_list();
}

void RRT::add_node(Node2d *node) {
    _tree_nodes.push_back(node);
    size_t cur_num =kdtree_nodes.kdtree_get_point_count();
    kdtree_nodes.pts.push_back(node);
    kdtree.addPoints(cur_num,kdtree_nodes.kdtree_get_point_count()-1);
}

bool RRT::make_plan(Point2d start, Point2d end) {
    _start=start;
    _goal=end;
    //std::cout<<"obstable num: "<<_obs_list.size()<<std::endl;
    if(!check_validity()){
        return false;
    }

    std::cout<<"start planning"<<std::endl;
    Node2d* start_ptr=new Node2d(start);
    add_node(start_ptr);
    int cur_iter=0;
    while(cur_iter < _max_iter && !_got_path){
        Point2d random_point=generate_random_point();
        Node2d* nearest_node = get_nearest(random_point);
        Node2d* new_node = expand_node(nearest_node,&random_point);
        if(is_collision_free(nearest_node,new_node)){
            //_tree_nodes.push_back(new_node);
            add_node(new_node);
            if(new_node->get_position() == _goal){
                _got_path=true;
            }
        }
        else{
            //std::cout<<"collision!"<<std::endl;
            delete new_node;
            new_node= nullptr;
        }
        if(cur_iter % 50 ==0){
            std::cout<<"current iter is "<< cur_iter<<std::endl;
        }
        cur_iter+=1;
    }
    if(_got_path){
        std::cout<<"RRT has got a path. "<<cur_iter<<" iteration used"<<std::endl;

        if(_show_animation){

            show_graph();
        }
        return true;
    }
    else{
        std::cout<<"RRT can not find a path in given max iter"<<std::endl;
        return false;
    }


}

void RRT::clear_tree() {
    for(auto tree_node : _tree_nodes){
        delete tree_node;
        tree_node= nullptr;
    }
    //for(auto node : kdtree_nodes.pts){
    //   node = nullptr;
    //}
    _tree_nodes.clear();
    kdtree_nodes.pts.clear();
    _path.clear();

}

Node2d* RRT::expand_node(Node2d *nearest_node, Point2d *random_point) {
    Point2d tep(random_point->get_x()-nearest_node->get_x(),random_point->get_y()-nearest_node->get_y());
    double dis =tep.get_length();
    double angle=tep.get_angle();

    dis=std::min(_step_size,dis);

    Node2d* new_node=new Node2d(nearest_node->get_x()+dis*cos(angle),nearest_node->get_y()+dis*sin(angle),nearest_node);
    return new_node;
}

Node2d* RRT::get_nearest(Point2d point) {
    /*
    double min_dis=std::numeric_limits<double>::max();
    Node2d *nearest_node= nullptr;
    for(auto tree_node:_tree_nodes){
        double dis=point.distance_to_point(tree_node->get_position());
        if(dis<min_dis){
            min_dis=dis;
            nearest_node=tree_node;
        }
    }
    return nearest_node;
     */
    double query_pt[2]={point.get_x(),point.get_y()};
    const size_t num_results = 1;
    size_t ret_index;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(num_results);
    resultSet.init(&ret_index, &out_dist_sqr );
    kdtree.findNeighbors(resultSet, query_pt, nanoflann::SearchParams(10));

    return kdtree_nodes.pts[ret_index];


}

Point2d RRT::generate_random_point() {
    std::default_random_engine engine;
    static int seed=time(0)*100;
    engine.seed(seed);
    seed+=1;

    int randx= engine()%(_max_x-_min_x)+_min_x;
    int randy= engine()%(_max_y-_min_y)+_min_y;
    if(randx%100>_goal_sample_rate) {
        return Point2d(randx, randy);
    }
    else{
        return _goal;
    }
}

bool RRT::is_collision_free(Node2d *first_node, Node2d *second_node) {
    Linesegment2d line(first_node->get_position(),second_node->get_position());

    for(auto obs:_obs_list){
        if(obs->has_overlap(line)){
            return false;
        }
    }
    return true;

}

void RRT::get_obstacle_list(std::vector<std::vector<int>> obs_list) {
    for(auto obs:obs_list){
        OBB2d *obs_ptr=new OBB2d(obs[0],obs[1],0.0,obs[2],obs[3]);
        _obs_list.push_back(obs_ptr);
    }

    /*
    clear_obs_list();
    for(int i=0;i<_map->rows;i++){
        for(int j=0;j<_map->cols;j++){
            if(_map->at<uchar>(i,j)==0 && (_map->at<uchar>(i-1,j) !=0 && _map->at<uchar>(i,j-1) !=0)){
                Point2d center(i+20,j+20);
                OBB2d* obs=new OBB2d(center,0,40,40);
                _obs_list.push_back(obs);

            }
        }
    }
    for(int i=_map->rows;i>=0;i--){
        for(int j=_map->cols;j>=0;j--){
            if(_map->at<uchar>(i,j)==0 && (_map->at<uchar>(i+1,j) !=0 && _map->at<uchar>(i,j+1) !=0)){
                Point2d center(i-20,j-20);
                OBB2d* obs=new OBB2d(center,0,40,40);
                for(auto ptr:_obs_list){
                    if(ptr->get_center() == obs->get_center()){
                        delete obs;
                        obs= nullptr;
                        break;
                    }
                }
                if(obs){
                    _obs_list.push_back(obs);
                }
            }
        }
    }
    std::cout<<"obs num: "<<_obs_list.size()<<std::endl;
    for(auto obs:_obs_list){
        std::cout<<"obs x: "<<obs->get_center().get_x()<< " obs y: "<<obs->get_center().get_y()<<std::endl;
    }*/

}

void RRT::clear_obs_list() {
    for(auto obs:_obs_list){
        delete obs;
        obs= nullptr;
    }
    _obs_list.clear();
}

bool RRT::check_validity() {
    for(auto obs : _obs_list){
        if(obs->is_Point_in(_start)){
            std::cerr<<"the start is in the obstacle"<<std::endl;
            return false;
        }
        if(obs->is_Point_in(_goal)) {
            std::cerr<<"the goal is in the obstacle"<<std::endl;
            return false;
        }
        if(_start.get_y()>_max_y || _start.get_y()<_min_y || _start.get_x()>_max_x || _start.get_x()<_min_x){
            std::cerr<<"the start is out of range"<<std::endl;
            return false;
        }
        if(_goal.get_x()>_max_x || _goal.get_x() < _min_x || _goal.get_y() < _min_y || _goal.get_y() > _max_y){
            std::cerr<<"the goal is out of range"<<std::endl;
            return false;
        }
    }
    return true;
}

void RRT::show_graph() {
    cv::Mat new_map= *_map;
    int count=0;
    for(auto tree_node:_tree_nodes){
        count+=1;
        if(!tree_node->get_parent()){
            continue;
        }
        Point2d temp(tree_node->get_parent()->get_position()- tree_node->get_position());
        double angle =temp.get_angle();
        int length=0;
        while(length<temp.get_length()){
            int x=std::max(0,(int)(length*std::cos(angle)+tree_node->get_x()));
            int y=std::max(0,(int)(length*std::sin(angle)+tree_node->get_y()));
            new_map.at<uchar>(y,x)=200;
            //std::cout<<y<<" x: "<<x<<std::endl;
            length+=1;
        }
        //std::cout<<_tree_nodes.size()<<" "<<count<<std::endl;

    }
    cv::imshow("planned map",new_map);
    cv::waitKey();

    Node2d* pointer=_tree_nodes.back();
    while(!(pointer->get_position() == _start)){
        Point2d temp(pointer->get_parent()->get_position()- pointer->get_position());
        double angle =  temp.get_angle();
        int length=1;
        new_map.at<uchar>((int)pointer->get_y(),(int)pointer->get_x())=0;
        while(length<temp.get_length()){
            int x=std::max(0,(int)(length*std::cos(angle)+pointer->get_x()));
            int y=std::max(0,(int)(length*std::sin(angle)+pointer->get_y()));
            new_map.at<uchar>(y,x)=0;
            length+=1;
        }
        pointer=pointer->get_parent();

    }
    cv::imshow("planned map",new_map);
    cv::waitKey();
}


