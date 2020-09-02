#ifndef _KDTREE_HELP
#define _KDTREE_HELP

#include "nanoflann.hpp"
#include "Node2d.h"



template <typename T>
struct KDTREE_POINTS
{

    std::vector<Node2d*>  pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0) return pts[idx]->get_x();
        else return pts[idx]->get_y();
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

};

// construct a kd-tree index:
typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
        nanoflann::L2_Simple_Adaptor<double, KDTREE_POINTS<double>>,
        KDTREE_POINTS<double>,
        2 /* dim */
> my_kd_tree_t;


#endif