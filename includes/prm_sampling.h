
/******************************************************************************/
/* 文件信息:
 *     1. filename: prm_sampling.h
 *     2. copyright: moonnight( liminkmf@mail.ustc.edu.cn)
 *     3. sign-off date: 2017-07-28 by moonnight
 *
 * 功能说明：定义了采样点获取函数，包括一个API接口和四个用于不同区域内的采样函数。
 *     1. SamplingNode()为外部借口函数，供PRM构建函数调用；
 *     2. 其余四个SamplingNode_case_X()为用于不同区域的采样策略，供SamplingNode()调用；
 * 输入参数:
 *     1. cv::Mat DTMap   : 距离变换后得到的距离地图；
 *     2. cv::Point curPt : 当前点的地图位置；
 *     3. int RobotSize   : 机器人尺寸--用于确定安全半径、自适应采样范围等；
 * 输出参数:
 *     std::vector<cv::Point> : 基于当前点进行采样得到的新的采样点。
 * 特殊依赖:
 *     1. 需要prm_planner.h中的AreaDetection()函数支持:用于识别curPt位于哪种区域;
 *     2. 需要OpenCV 支持;
 *     3. 需要C++ 11 random lib支持
 */
/******************************************************************************/

#ifndef _PRM_SAMPLING_H_H
#define _PRM_SAMPLING_H_H

#include "MyHeader.h"
#include "prm_DT_based.h"

std::vector<cv::Point> SamplingNode( const cv::Mat DTMap, const cv::Point curPt, const int RobotSize );

// 以下为四个用于不同区域的采样函数
std::vector<cv::Point> SamplingNode_case_F( const cv::Mat DTMap, const cv::Point curPt, const int RobotSize );
std::vector<cv::Point> SamplingNode_case_O( const cv::Mat DTMap, const cv::Point curPt, const int RobotSize );
std::vector<cv::Point> SamplingNode_case_E( const cv::Mat DTMap, const cv::Point curPt, const int RobotSize );
std::vector<cv::Point> SamplingNode_case_N( const cv::Mat DTMap, const cv::Point curPt, const int RobotSize );

#endif // _PRM_SAMPLING_H_H
