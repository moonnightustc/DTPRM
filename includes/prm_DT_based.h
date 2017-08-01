
#ifndef _PRM_DT_BASED_H_
#define _PRM_DT_BASED_H_

#include "MyHeader.h"
#include "graph_t.h"
#include "prm_general_operation.h"
#include "prm_sampling.h"

float mapDensity( const cv::Mat DTMap, float &DutyRatio );
int prm_GetSamplingNumbers( const cv::Mat DTMap, int RobotSize );

int AreaDetection( const cv::Mat DTMap, const cv::Point curPt, const int RobotSize );
cv::Point GetMaxNeight( const cv::Mat DTMap, const cv::Point curPt );
cv::Point GetMinNeight( const cv::Mat DTMap, const cv::Point curPt );

void prm_map_thredhold( const cv::Mat src, cv::Mat &dst );

/*****************************************************************************/
// Three core steps of DT-based PRM path planning( Also the APIs)
// Step-1: Calculate the distance map--DTMap
void prm_disanceMap( const cv::Mat srcMap, cv::Mat &DTMap );
// Step-2: Constructe the PRM graph
void prm_constructor( const cv::Mat DTMap, graph_t &G, const int RobotSize );
// Step-3: Query the path according to the init and goal configuration
std::vector<cv::Point> prm_query( const graph_t G, const cv::Point qInit, const cv::Point qGoal, const cv::Mat DTMap, const int RobotSize );
/*****************************************************************************/

#endif //_PRM_DT_BASED_H_
