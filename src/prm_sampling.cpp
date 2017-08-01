
/******************************************************************************/
/* 文件信息:
 *     1. filename: prm_sampling.cpp
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

#include "prm_sampling.h"


std::vector<cv::Point> SamplingNode( const cv::Mat DTMap, const cv::Point curPt, const int RobotSize )
{
    int res = AreaDetection( DTMap, curPt, RobotSize );
    const int ValueGrid = (int)DTMap.at<uchar>( curPt.x, curPt.y );
    const int MWD = RobotSize / 2;

    std::vector<cv::Point> Nodes_Res;

    if( res == -2 )
    { // Obstacle Area
    }
    else if( res == -1 )
    { // Forbid Area
        Nodes_Res = SamplingNode_case_F( DTMap, curPt, RobotSize);
    }
    else if( res == 0 )
    { // Open Area
        Nodes_Res = SamplingNode_case_O( DTMap, curPt, RobotSize);
    }
    else if( res == 1 )
    { // Edge and Corner
        Nodes_Res = SamplingNode_case_E( DTMap, curPt, RobotSize);
    }
    else if( res == 4 )
    { // Narrow passage
        Nodes_Res = SamplingNode_case_N( DTMap, curPt, RobotSize);
    }

    //std::cout << curPt << "=" << ValueGrid << "--" << Nodes_Res.size()  << std::endl;
    return Nodes_Res;
}

std::vector<cv::Point> SamplingNode_case_F( const cv::Mat DTMap, const cv::Point curPt, const int RobotSize )
{
    //std::cout << "Forbid Entry!" << std::endl;
    const int ValueGrid = (int)DTMap.at<uchar>( curPt.x, curPt.y );
    const int MWD = RobotSize / 2;
    std::vector<cv::Point> Nodes_Res_vec;
    /*
    int maxValue = (int)DTMap.at<uchar>( curPt.x-1, curPt.y-1 );
    cv::Point maxPt = cv::Point(-1, -1);
    for( int i = -1; i <=1; i++ )
    {
        for( int j = -1; j <= 1; j++ )
        {
            if( i==0 && j==0 )
                continue;
            if( (int)DTMap.at<uchar>( curPt.x+i, curPt.y+j ) > maxValue )
            {
                maxValue = (int)DTMap.at<uchar>( curPt.x+i, curPt.y+j );
                maxPt = cv::Point(i, j);
            }
        }
    }
    // Strategy-1: random sampling
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator( seed );
    std::uniform_int_distribution<int> disC( MWD, 6*RobotSize );
    int distance = disC(generator);
    cv::Point NodeRes = cv::Point( curPt.x + maxPt.x*distance, curPt.y + maxPt.y*distance );
    while( (int)DTMap.at<uchar>(NodeRes.x, NodeRes.y) <= MWD  )
    {
        distance = disC(generator);
        cv::Point NodeRes = cv::Point( curPt.x + maxPt.x*distance, curPt.y + maxPt.y*distance );
    }
    */
    // Strategy-2: N-Step Search
    cv::Point NodeRes = cv::Point( curPt.x, curPt.y );
    for( int i = 1; i < 3*RobotSize - ValueGrid ; i++ )
    {
        cv::Point maxPt = GetMaxNeight( DTMap, NodeRes );
        if( NodeRes == maxPt )
        {
            break;
        }
        NodeRes = maxPt;
    }

    Nodes_Res_vec.push_back( NodeRes );
    //std::cout << "Forbid: " << NodeRes << std::endl;
    return Nodes_Res_vec;
}

std::vector<cv::Point> SamplingNode_case_O( const cv::Mat DTMap, const cv::Point curPt, const int RobotSize )
{
    //std::cout << "Open Entry!" << std::endl;
    const int ValueGrid = (int)DTMap.at<uchar>( curPt.x, curPt.y );
    const int MWD = RobotSize / 2;
    std::vector<cv::Point> Nodes_Res_vec;
    cv::Point NodeRes = cv::Point( curPt.x, curPt.y );
    for( int i = 1; i < ValueGrid - RobotSize ; i++ )
    {
        cv::Point maxPt = GetMaxNeight( DTMap, NodeRes );
        
        if( NodeRes == maxPt )
        {
            break;
        }
        NodeRes = maxPt;
    }
    //std::cout << "Open: " << NodeRes << std::endl;
    //Nodes_Res_vec.push_back( NodeRes );

    NodeRes = cv::Point( curPt.x, curPt.y );
    for( int i = 1; i < ValueGrid - RobotSize ; i++ )
    {
        cv::Point minPt = GetMinNeight( DTMap, NodeRes );
        if( NodeRes == minPt )
        {
            break;
        }
        NodeRes = minPt;
    }
    //std::cout << "Open: " << NodeRes << std::endl;
    Nodes_Res_vec.push_back( NodeRes );

    return Nodes_Res_vec;
}

std::vector<cv::Point> SamplingNode_case_E( const cv::Mat DTMap, const cv::Point curPt, const int RobotSize )
{
    //std::cout << "Edge Entry!" << std::endl;
    int ValueGrid = (int)DTMap.at<uchar>( curPt.x, curPt.y );
    int MWD = RobotSize / 2;
    std::vector<cv::Point> Nodes_Res_vec;
    int maxValue = (int)DTMap.at<uchar>( curPt.x-1, curPt.y-1 );
    cv::Point maxPt = cv::Point(-1, -1);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator( seed );
    std::uniform_int_distribution<int> dis_distance( RobotSize*3, 6*ValueGrid );
    std::uniform_int_distribution<int> dis_angle( 0, 360 );
    for(int i = 0; i < 4; i++)
    {
        cv::Point NodeRes;
        int res;
        do
        {
            int distance = dis_distance(generator);
            int angle = dis_angle(generator);
            NodeRes = cv::Point( curPt.x + distance * cos(angle * PI / 180.0), curPt.y + distance * sin(angle * PI / 180.0) );
            res = AreaDetection( DTMap, NodeRes, RobotSize );
            //std::cout << "Edge: " << NodeRes << std::endl;
        }while( !(res == 1 || res == 4) || !(NodeRes.x > 0 && NodeRes.x < DTMap.rows && NodeRes.y > 0 && NodeRes.y < DTMap.cols) || DTMap.at<uchar>(NodeRes.x, NodeRes.y) <= MWD );
        Nodes_Res_vec.push_back( NodeRes );
    }
    if( Nodes_Res_vec.empty() )
    {
        cv::Point NodeRes = cv::Point( curPt.x, curPt.y );
        for( int i = 1; i < 3 * RobotSize ; i++ )
        {
            cv::Point maxPt = GetMaxNeight( DTMap, NodeRes );
            if( NodeRes == maxPt )
            {
                break;
            }
            NodeRes = maxPt;
        }
        //std::cout << "Edge: " << NodeRes << std::endl;
        Nodes_Res_vec.push_back( NodeRes );
    }
    return Nodes_Res_vec;
}

std::vector<cv::Point> SamplingNode_case_N( const cv::Mat DTMap, const cv::Point curPt, const int RobotSize )
{
    //std::cout << "Narrow Entry!" << std::endl;
    int ValueGrid = (int)DTMap.at<uchar>( curPt.x, curPt.y );
    int MWD = RobotSize / 2;
    std::vector<cv::Point> Nodes_Res_vec;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator( seed );
    std::uniform_int_distribution<int> dis_distance( 2*MWD, 4*ValueGrid );
    std::uniform_int_distribution<int> dis_angle( 0, 360 );
    for( int i = 0; i < 8; i++ )
    {
        cv::Point NodeRes;
        do
        {
            int distance = dis_distance(generator);
            int angle = dis_angle(generator);
            NodeRes = cv::Point( curPt.x + distance * cos(angle * PI / 180.0), curPt.y + distance * sin(angle * PI / 180.0) );
        }while( !(NodeRes.x > 0 && NodeRes.x < DTMap.rows && NodeRes.y > 0 && NodeRes.y < DTMap.cols) || DTMap.at<uchar>(NodeRes.x, NodeRes.y) <= MWD  );
        //std::cout << "Narrow: " << NodeRes << std::endl;
        Nodes_Res_vec.push_back( NodeRes );
    }
    return Nodes_Res_vec;
}
