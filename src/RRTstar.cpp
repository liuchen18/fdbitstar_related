#include "RRTstar.h"
#include <algorithm>
#include <cmath>
#include <random>
#include "limits"

RRT_STAR::~RRT_STAR() {
    clear_tree();
    clear_obs_list();
}

void RRT_STAR::clear_tree() {
    for(auto tree_node : _tree_nodes){
        delete tree_node;
        tree_node= nullptr;
    }
    _tree_nodes.clear();
    kdtree_nodes.pts.clear();
    _path.clear();

}

void RRT_STAR::clear_obs_list() {
    for(auto obs:_obs_list){
        delete obs;
        obs= nullptr;
    }
    _obs_list.clear();
}

Node2d* RRT_STAR::expand_node(Node2d *nearest_node, Point2d *random_point) {
    Point2d tep(random_point->get_x()-nearest_node->get_x(),random_point->get_y()-nearest_node->get_y());
    double dis =tep.get_length();
    double angle=tep.get_angle();

    dis=std::min(_step_size,dis);

    Node2d* new_node=new Node2d(nearest_node->get_x()+dis*cos(angle),nearest_node->get_y()+dis*sin(angle),nearest_node);
    //std::cout<<random_point->get_x()<<" point "<<random_point->get_y()<<std::endl;
    //std::cout<<new_node->get_x()<<" node "<<new_node->get_y()<<std::endl;
    return new_node;
}

Node2d* RRT_STAR::get_nearest(Point2d point) {
    /**
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

Point2d RRT_STAR::generate_random_point() {
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

bool RRT_STAR::is_collision_free(Node2d *first_node, Node2d *second_node) {
    Linesegment2d line(first_node->get_position(),second_node->get_position());

    for(auto obs:_obs_list){
        if(obs->has_overlap(line)){
            return false;
        }
    }
    return true;

}

void RRT_STAR::get_obstacle_list(std::vector<std::vector<int>> obs_list) {
    for (auto obs:obs_list) {
        OBB2d *obs_ptr = new OBB2d(obs[0], obs[1], 0.0, obs[2], obs[3]);
        _obs_list.push_back(obs_ptr);
    }
}

bool RRT_STAR::check_validity() {
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

void RRT_STAR::show_tree() {
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
    cv::imshow("search tree",new_map);
    cv::waitKey(0.05);
}

void RRT_STAR::show_graph() {
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
    cv::imshow("search tree",new_map);
    cv::waitKey();

    Node2d* pointer=_goal_ptr;
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
    cv::imshow("final path",new_map);
    cv::waitKey();
}

double RRT_STAR::get_search_radius() {
    //_search_radius=fmax(500.0*sqrt(log(_tree_nodes.size()+1)/(_tree_nodes.size()+1)),(double)_step_size);
    _search_radius=500.0*sqrt(log(_tree_nodes.size()+1)/(_tree_nodes.size()+1));
    return _search_radius;
}
std::vector<Node2d*> RRT_STAR::get_near_neighbor(Node2d *new_node) {
    std::vector<Node2d*> neighbors;
    double query_pt[2]={new_node->get_position().get_x(),new_node->get_position().get_y()};
    /*
    for(auto node:_tree_nodes){
        double dis=get_distance(node,new_node);
        if(dis<_search_radius){
            neighbors.push_back(node);
        }
    }
     */
    const double radius = _search_radius*_search_radius;
    std::vector<std::pair<size_t, double> > indices_dists;
    nanoflann::RadiusResultSet<double, size_t> resultSet(radius, indices_dists);

    kdtree.findNeighbors(resultSet, query_pt, nanoflann::SearchParams());
    for(auto node:resultSet.m_indices_dists){
        neighbors.push_back(kdtree_nodes.pts[node.first]);
    }
    return neighbors;
}

double RRT_STAR::get_distance(Node2d *start, Node2d *end) {
    return hypot(start->get_x()-end->get_x(),start->get_y()-end->get_y());
}

void RRT_STAR::choose_parent(Node2d *new_node, std::vector<Node2d *> neighbors) {
    double max_double=std::numeric_limits<double>::max();
    new_node->set_cost(max_double);
    Node2d* parent= nullptr;
    for(auto neighbor:neighbors){
        double dis=get_distance(neighbor,new_node);
        if(new_node->get_cost()>dis+neighbor->get_cost() && is_collision_free(neighbor,new_node)){
            new_node->set_cost(dis+neighbor->get_cost());
            parent=neighbor;
        }
    }
    new_node->set_parent(parent);
}

void RRT_STAR::rewire(Node2d *new_node, std::vector<Node2d*> neighbors) {
    for(auto neighbor:neighbors){
        if(neighbor->get_cost() > new_node->get_cost()+get_distance(new_node,neighbor) && is_collision_free(neighbor,new_node)){
            neighbor->set_cost(new_node->get_cost()+get_distance(new_node,neighbor));
            neighbor->set_parent(new_node);
        }
    }

}

bool RRT_STAR::make_plan(Point2d start, Point2d end) {
    _start=start;
    _goal=end;
    //std::cout<<"obstable num: "<<_obs_list.size()<<std::endl;
    if(!check_validity()){
        return false;
    }

    std::cout<<"start planning"<<std::endl;
    Node2d* start_ptr=new Node2d(start);
    //_tree_nodes.push_back(start_ptr);
    add_node(start_ptr);
    int cur_iter=0;
    while(cur_iter < _max_iter){
        Point2d random_point=generate_random_point();
        Node2d* nearest_node = get_nearest(random_point);
        Node2d* new_node = expand_node(nearest_node,&random_point);
        get_search_radius();
        std::vector<Node2d*> neighbors=get_near_neighbor(new_node);
        //std::cout<<"neighbor size: "<<neighbors.size()<<std::endl;
        choose_parent(new_node,neighbors);
        if(new_node->get_parent()){
            rewire(new_node,neighbors);
            //_tree_nodes.push_back(new_node);
            add_node(new_node);
            if(new_node->get_position() == _goal){
                _goal_ptr=new_node;
                _got_path=true;
            }
        }
        else{
            //std::cout<<"collision!"<<std::endl;
            delete new_node;
            new_node= nullptr;
        }
        if(cur_iter % 500 ==0){
            std::cout<<"current radius: "<<_search_radius<<std::endl;
            std::cout<<"current node number: "<<_tree_nodes.size()<<std::endl;
            std::cout<<"current iter is "<< cur_iter<<std::endl;
            //show_tree();
        }
        cur_iter+=1;
    }
    if(_got_path){
        std::cout<<"RRT star has got a path."<<std::endl;

        if(_show_animation){

            show_graph();
        }
        return true;
    }
    else{
        std::cout<<"RRT star can not find a path in given max iter"<<std::endl;
        show_tree();
        return false;
    }


}

void RRT_STAR::add_node(Node2d *node) {
    _tree_nodes.push_back(node);
    size_t cur_num =kdtree_nodes.kdtree_get_point_count();
    kdtree_nodes.pts.push_back(node);
    kdtree.addPoints(cur_num,kdtree_nodes.kdtree_get_point_count()-1);
}
