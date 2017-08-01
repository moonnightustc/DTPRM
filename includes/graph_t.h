

#ifndef graph_t_H_
#define graph_t_H_

#include "MyHeader.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace boost;

struct Node_Point_t{
    typedef vertex_property_tag kind;
};
struct EdgeProperty{
    unsigned int index;
    int dis;
};
typedef property<Node_Point_t, cv::Point > VertexProperty;
typedef adjacency_list < listS, vecS, undirectedS, VertexProperty, property < edge_weight_t, int > > graph_t;
typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
typedef graph_traits < graph_t >::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;
typedef graph_traits<graph_t>::vertex_iterator vertex_iterator;
typedef graph_traits<graph_t>::edge_iterator edge_iterator;
typedef graph_traits<graph_t>::out_edge_iterator out_edge_iterator;


#endif // graph_t_H_