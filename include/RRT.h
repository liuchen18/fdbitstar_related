#ifndef _RRT
#define _RRT

#include "planner_core.h"
#include "Node2d.h"
#include "OBB2d.h"
#include "kdtree_help.h"

class RRT: public Planner{
private:
    cv::Mat *_map;
    double _step_size;
    int _max_iter;
    int _max_x, _min_x,_max_y,_min_y;
    std::vector<Point2d> _path;
    std::vector<OBB2d*> _obs_list;
    std::vector<Node2d*> _tree_nodes;

    Point2d _start,_goal;

    int _goal_sample_rate;
    bool _got_path;
    bool _show_animation;
    KDTREE_POINTS<double> kdtree_nodes;
    my_kd_tree_t   kdtree;


public:
    /**
     * @brief constructor
     * @param map the environment
     * @param obs_list the vector that contains the obstacle information. x,y,length,width
     */
    RRT(cv::Mat *map,
        std::vector<std::vector<int>> obs_list,
        int step_size=5,
        int max_iter=20000,
        int max_x=800,
        int min_x=0,
        int max_y=500,
        int min_y=0,
        int goal_sample_rate=0):
            _map(map),
            _path(std::vector<Point2d>()),
            _obs_list(std::vector<OBB2d*>()),
            _tree_nodes(std::vector<Node2d*>()),
            _start(Point2d(0,0)),
            _goal(Point2d(0,0)),
            kdtree(2 /*dim*/, kdtree_nodes, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */) ){
        get_obstacle_list(obs_list);
        _step_size=step_size;
        _max_iter=max_iter;
        _max_x=max_x;
        _max_y=max_y;
        _min_x=min_x;
        _min_y=min_y;
        _goal_sample_rate=goal_sample_rate;
        _got_path=false;
        _show_animation=true;


    }

    /**
     * @brief destructor
     */
    virtual ~RRT();

    /**
     * @brief make plan
     * @param start
     * @param end
     * @return true if successful
     */
    virtual bool make_plan(Point2d start, Point2d end);

    /**
      * @brief make plan
      * @param start_x
      * @param start_y
      * @param end_x
      * @param end_y
      * @return true if got plan successfully
      */
    virtual bool make_plan(double start_x,double start_y,double end_x,double end_y){
        Point2d s(start_x,start_y),e(end_x,end_y);
        return make_plan(s,e);
    }


    /**
     * @brief clear the tree and the path
     * @return true if done
     */
    void clear_tree();

    /**
     * @brief generate random node
     * @return random node
     */
    Point2d generate_random_point();

    /**
     * @brief the the nearest node in the tree
     * @param node
     * @return
     */
    Node2d* get_nearest(Point2d point);

    /**
     * @brief expand the tree according to the nearest node and the random node
     * @param nearest_node
     * @param random_node
     * @return the pointer to the new node
     */
    Node2d* expand_node(Node2d *nearest_node, Point2d* random_point);

    /**
     * @brief check whether the Linesegment between the first node and second node is collision free
     * @param first_node
     * @param second_node
     * @return true if collision free
     */
    bool is_collision_free(Node2d* first_node, Node2d* second_node);

    /**
     * @brief get the obstacle list from map
     */
    void get_obstacle_list(std::vector<std::vector<int>> obs_list);

    /**
     * @brief set the map to a new map
     * @param map
     */
    virtual void set_map(cv::Mat *map,std::vector<std::vector<int>> obs_list){
        _map=map;
        get_obstacle_list(obs_list);
    }

    /**
     * @brief clear the obstacle list
     */
    void clear_obs_list();

    /**
     * @brief check the validity of the start and goal
     * @return true if start  and goal are not in obstacle
     */
    bool check_validity();

    /**
     * @brief show the env and the path
     */
    virtual void show_graph();

    /**
     * @brief add node to the rrt tree and kd tree
     * @param node
     */
    void add_node(Node2d* node);



};



#endif