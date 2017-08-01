

#include "prm_general_operation.h"

float distance( const cv::Point q1, const cv::Point q2 )
{
    return sqrt( pow( q1.x - q2.x, 2 ) + pow( q1.y - q2.y, 2 ) );
}

bool prm_collision_checker( const cv::Point q1, const cv::Point q2, const cv::Mat Map, const int RobotSize)
{
    // Input parameters check
    //if( Map.empty() )
    //{
    //    std::cout << "Debug(prm_planner.cpp)::prm_collision_checker: input param error--cv::Mat Map is empty!" << std::endl;
    //    return false;
    //}
    if( q1.x < 0 || q1.x >= Map.rows || q1.y < 0 || q1.y >= Map.cols )
    {
        //std::cout << "Debug(prm_planner.cpp)::prm_collision_checker: input param error--cv::Point q1 is out of range!" << std::endl;
        return false;
    }
    if( q2.x < 0 || q2.x >= Map.rows || q2.y < 0 || q2.y >= Map.cols )
    {
        //std::cout << "Debug(prm_planner.cpp)::prm_collision_checker: input param error--cv::Point q2 is out of range!" << std::endl;
        return false;
    }

    // Set the Search Area
    int Xmin, Xmax, Ymin, Ymax;
    if( q1.x <= q2.x )
    {
        Xmin = q1.x;
        Xmax = q2.x;
    }
    else
    {
        Xmin = q2.x;
        Xmax = q1.x;
    }
    if( q1.y <= q2.y )
    {
        Ymin = q1.y;
        Ymax = q2.y;
    }
    else
    {
        Ymin = q2.y;
        Ymax = q1.y;
    }

    // Collision Check
    cv::Mat SearchMap( Map.rows, Map.cols, CV_8UC1, cv::Scalar::all(255) );
    cv::line( SearchMap, cv::Point(q1.y, q1.x), cv::Point(q2.y, q2.x), cv::Scalar(0), 2 );
    //cv::imshow( "Debug(collision checker)", Map );
    for( int i = Xmin; i <= Xmax; i++ )
    {
        for( int j = Ymin; j <= Ymax; j++ )
        {
            if( SearchMap.at<uchar>(i,j) == 0 )
            {
                if( Map.at<uchar>(i,j) <= RobotSize/2 )
                {
                    return false; // if the straight line meet an obstacle grid, then return false
                }
            }
        }
    }
    return true;
}

void prm_local_planner( graph_t &G, const cv::Point newNode, const cv::Mat DTMap, const int RobotSize )
{
    // Input parameters check
    if( DTMap.empty() )
    {
        //std::cout << "Debug(prm_planner.cpp)::prm_collision_checker: input param error--cv::Mat Map is empty!" << std::endl;
        return ;
    }
    if( newNode.x < 0 || newNode.x >= DTMap.rows || newNode.y < 0 || newNode.y >= DTMap.cols )
    {
        //std::cout << newNode << " = " << (int)DTMap.at<uchar>(newNode.x, newNode.y) << std::endl;
        //std::cout << "Debug(prm_planner.cpp)::prm_local_planner: input param error--cv::Point newNode is out of range!" << std::endl;
        return ;
    }
    const int ValueGrid = DTMap.at<uchar>(newNode.x, newNode.y);
    const int MWD = RobotSize / 2;
    if( ValueGrid <= MWD )
    {
        return;
    }

    static int index = 0;
    index++;
    //std::cout << "Debug(prm_planner.cpp)::prm_local_planner: new Node = " << newNode << std::endl;

    VertexProperty v1 = newNode;
    vertex_descriptor vp1 = add_vertex( v1, G );
    
    int Range = RobotSize ;
    int res = AreaDetection( DTMap, newNode, RobotSize );
    if( res ==4 )
    {
        Range = 4*ValueGrid;
    }
    else if( res == 1 )
    {
        Range = 6*ValueGrid;
    }
    else if( res == 0 )
    {
        Range = ValueGrid - RobotSize;
    }
    else if( res == -1 )  
    {
        Range = 3*RobotSize - ValueGrid;
    }
    else
    {
        Range = 10 * RobotSize;
    }

    std::vector<vertex_descriptor> Neighbors = prm_GetNeighbors( newNode, G, Range );

    //std::cout << "Debug(prm_planner.cpp)::prm_local_planner: Neighbors Number = " << Neighbors.size() << std::endl;

    property_map<graph_t, Node_Point_t>::type NodeMap = boost::get( Node_Point_t(), G );
    for( auto iter = Neighbors.begin(); iter != Neighbors.end(); iter++ )
    {
        cv::Point Pt = get( NodeMap, *iter );
        if( prm_collision_checker( newNode, Pt, DTMap, RobotSize ) )
        {
            //std::cout << "Debug(prm_planner.cpp)::prm_local_planner: Neighbor = " << Pt << std::endl;
            add_edge( vp1, *iter, distance( newNode, Pt ), G );
        }
    }
    //std::cout << "Debug(prm_planner.cpp)::prm_local_planner exit with index = " << index << std::endl;
}

std::vector<vertex_descriptor> prm_GetNeighbors( const cv::Point newNode, const graph_t G, const int Range )
{
    property_map<graph_t, Node_Point_t>::const_type NodeMap = boost::get( Node_Point_t(), G );
    vertex_iterator vi, vend;
    std::vector<vertex_descriptor> vd_res;
    for( boost::tie( vi, vend ) = vertices(G); vi != vend; vi++ )
    {
        cv::Point Node_Pt = get( NodeMap, *vi );
        if( distance( newNode, Node_Pt ) <= Range )
        {
            vd_res.push_back( *vi );
        }
    }
    return vd_res;
}
