

#ifndef _PRM_GENERAL_OPERATION_H_
#define _PRM_GENERAL_OPERATION_H_

#include "MyHeader.h"
#include "graph_t.h"
#include "prm_DT_based.h"

float distance( const cv::Point q1, const cv::Point q2 );
bool prm_collision_checker( const cv::Point q1, const cv::Point q2, const cv::Mat Map, const int RobotSize);
void prm_local_planner( graph_t &G, const cv::Point newNode, const cv::Mat DTMap, const int RobotSize );
std::vector<vertex_descriptor> prm_GetNeighbors( const cv::Point newNode, const graph_t G, const int Range );

#endif // _PRM_GENERAL_OPERATION_H_
