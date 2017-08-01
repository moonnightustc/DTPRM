#include "prm_DT_based.h"

float mapDensity( const cv::Mat DTMap, float &DutyRatio )
{
    int TotalGridNum = 0;
    int ValidGridNum = 0;
    int ValidGridValueSum = 0;
    for(int i = 0; i < DTMap.rows; i++)  
    {  
        for(int j = 0; j < DTMap.cols; j++)  
        {  
            TotalGridNum ++;
            if( DTMap.at<uchar>(i,j)>0 )  
            {  
                ValidGridNum ++;
                ValidGridValueSum += DTMap.at<uchar>(i,j);
            }
        }
    }

    float meanValue = (float)ValidGridValueSum / ValidGridNum;

    int L = std::max( DTMap.rows, DTMap.cols );
    int W = std::min( DTMap.rows, DTMap.cols );
    float refValue = ( (0.5*(L-W)*W) + W*W/6 )/L;

    float D = 1 - meanValue / refValue;
    DutyRatio = (float)ValidGridNum / TotalGridNum;

    std::cout << "meanValue = " << meanValue << std::endl;
    std::cout << "refValue = " << refValue << std::endl;
    std::cout << "D = " << D << std::endl;
    return D;
}

int prm_GetSamplingNumbers( const cv::Mat DTMap, int RobotSize )
{
    int NumRef = DTMap.rows * DTMap.cols / ( 4 * RobotSize * RobotSize );
    float Density, DutyRatio;   
    Density = mapDensity( DTMap, DutyRatio );

    int NumRes = NumRef * DutyRatio * Density;

    std::cout << "Density: " << Density << std::endl;
    std::cout << "DutyRatio: " << DutyRatio << std::endl;
    std::cout << "NumRef: " << NumRef << std::endl;
    std::cout << "NumRes: " << NumRes << std::endl;
    return NumRes;
}

cv::Point GetMaxNeight( const cv::Mat DTMap, const cv::Point curPt )
{
    cv::Point maxPt = curPt;
    int maxValue = (int)DTMap.at<uchar>( maxPt.x, maxPt.y );
    for( int i = -1; i <= 1; i++ )
    {
        for( int j = -1; j <= 1; j++ )
        {
            int Value = (int)DTMap.at<uchar>( curPt.x+i, curPt.y+j );
            if( Value > maxValue  )
            {
                maxPt = cv::Point( curPt.x+i, curPt.y+j );
                maxValue = Value;
            }
        }
    }
    return maxPt;
}

cv::Point GetMinNeight( const cv::Mat DTMap, const cv::Point curPt )
{
    cv::Point minPt = curPt;
    int minValue = (int)DTMap.at<uchar>( minPt.x, minPt.y );
    for( int i = -1; i <= 1; i++ )
    {
        for( int j = -1; j <= 1; j++ )
        {
            int Value = (int)DTMap.at<uchar>( curPt.x+i, curPt.y+j );
            if( Value < minValue  )
            {
                minPt = cv::Point( curPt.x+i, curPt.y+j );
                minValue = Value;
            }
        }
    }
    return minPt;
}

int AreaDetection( const cv::Mat DTMap, const cv::Point curPt, const int RobotSize )
{
    int ValueGrid = (int)DTMap.at<uchar>( curPt.x, curPt.y );
    if( ValueGrid == 0 )
        return -2; // Obstacle Area
    if( ValueGrid <= RobotSize / 2 && ValueGrid > 0 )
        return -1; // Forbid Accedd Area
    else if( ValueGrid > 3*RobotSize )
        return 0;  // Open Area

    cv::Point iterPt = curPt;
    //std::cout << "----------------------------" << std::endl;
    for( int i = 1; i < 3*RobotSize -ValueGrid-2 ; i++ )
    {
        cv::Point maxPt = GetMaxNeight( DTMap, iterPt );
        //std::cout << maxPt << std::endl;
        if( iterPt == maxPt )
        {
            return 4;
        }
        iterPt = maxPt;
    }
    return 1;
}

void prm_map_thredhold( const cv::Mat src, cv::Mat &dst )
{
    dst = src.clone();
    for( int i = 0; i < src.rows; i++ )
    {
        for( int j = 0; j < src.cols; j++ )
        {
            if( src.at<uchar>(i,j) <= 177 )
            {
                dst.at<uchar>(i,j) = 0;
            }
            else
            {
                dst.at<uchar>(i,j) = 255;
            }
        }
    }
    /**********************************************
    // For Unit Test: show the src and dst Map
    cv::imshow( "srcMap", src );
    cv::imshow( "dstMap", dst );
    **********************************************/
}

// Step-1: Calculate the distance map--DTMap
void prm_disanceMap( const cv::Mat srcMap, cv::Mat &DTMap )
{
    //boost::progress_timer t;
    //boost::timer t;
    // transform the source map to binary map
    cv::Mat binaryMap;
    prm_map_thredhold( srcMap, binaryMap );
    // Execute the distance transform
    cv::Mat thinMap(srcMap.size(), CV_32FC1 );
    cv::distanceTransform( binaryMap, thinMap, CV_DIST_L2, 3 ); 

	float maxValue=0;  //距离变换矩阵中的最大值
	cv::Point MaxPt(0,0);  
    DTMap = cv::Mat::zeros(srcMap.size(),CV_8UC1);
    for( int i = 0; i < thinMap.rows; i++ )  
    {  
        for( int j = 0; j < thinMap.cols; j++ )  
        {  
            DTMap.at<uchar>(i,j) = thinMap.at<float>(i,j);  
            if( thinMap.at<float>(i,j) > maxValue )  
            {  
                maxValue = thinMap.at<float>(i,j); 
                MaxPt = cv::Point(j,i);  
            }  
        }  
    }

    //std::cout << t.elapsed() << std::endl;

    /*************************************************/
    //cv::Mat distShow;
    //cv::normalize( DTMap, distShow, 0, 255, CV_MINMAX ); //为了显示清晰，做了0~255归一化
    //cv::imshow( "distance image", distShow );
    //cv::imwrite( "distance_image.jpg", distShow );
    /*************************************************/
}
// Step-2: Constructe the PRM graph
void prm_constructor( const cv::Mat DTMap, graph_t &G, const int RobotSize )
{
    //boost::progress_timer Module_Timer;
    // Step-1: Get Total Sampling Nodes Numbers adaptively.
    const int MaxNumber = prm_GetSamplingNumbers( DTMap, RobotSize );
    const int MWD = RobotSize / 2;    
    std::cout << "Debug(prm_planner.cpp)::prm_constructor:Total Sampling Numbers: " << MaxNumber << std::endl;
    
    // Random Generator and Distributor initialization
    // Set the random seed and uniform distribution
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator( seed );
    std::uniform_int_distribution<int> disX( MWD, DTMap.rows-MWD );
    std::uniform_int_distribution<int> disY( MWD, DTMap.cols-MWD );
    
    // Random a seed point
    cv::Point curPt;
    do{
        curPt = cv::Point( disX(generator), disY(generator) );
    } while( (int)DTMap.at<uchar>( curPt.x, curPt.y ) <= MWD );
    
    std::cout << "Debug(prm_planner.cpp)::prm_constructor: Start Node Position: " << curPt << " = " << (int)DTMap.at<uchar>(curPt.x, curPt.y) << std::endl;
    
    std::queue<cv::Point> SamplingNodes;
    SamplingNodes.push( curPt );
    std::vector<cv::Point> newPts;

    int i = 0;
    double AreaDetectionTimer = 0.0;
    int cnt = 0;
    while( num_vertices(G) < MaxNumber )
    {
        if( SamplingNodes.empty() )
        {
            do{
                curPt = cv::Point( disX(generator), disY(generator) );
            } while( (int)DTMap.at<uchar>( curPt.x, curPt.y ) <= MWD );
            SamplingNodes.push( curPt );
        }

        std::queue<cv::Point> SamplingNodesBuffer;
        
        while( !SamplingNodes.empty() )
        {
            cv::Point curPt = SamplingNodes.front();
            SamplingNodes.pop();
            int ValueGrid = (int)DTMap.at<uchar>( curPt.x, curPt.y );
            
            boost::timer t;
            int res = AreaDetection( DTMap, curPt, RobotSize );
            AreaDetectionTimer += t.elapsed();
            cnt++;
            int Range = RobotSize;
            if( res ==4 )
            {
                Range = 2*MWD;
            }
            if( res == 1 )
            {
                Range = RobotSize*3;
            }
            if( prm_GetNeighbors( curPt, G, Range ).size() != 0 )
                continue;

            i++;
            // update the graph according to lcoal planner
            if( num_vertices(G) == 0 )
            {
                add_vertex( curPt, G );
            }
            else
            {
                prm_local_planner( G, curPt, DTMap, RobotSize );
            }

            newPts = SamplingNode( DTMap, curPt, RobotSize );
            for( int j = 0; j < newPts.size(); j++ )
            {
                SamplingNodesBuffer.push( newPts.at(j) );
            }
        }
        std::swap( SamplingNodesBuffer, SamplingNodes );
    }
    std::cout << "AreaDetection time: " << AreaDetectionTimer  << std::endl;
}

// Step-3: Query the path according to the init and goal configuration
std::vector<cv::Point> prm_query( const graph_t G, const cv::Point qInit, const cv::Point qGoal, const cv::Mat DTMap, const int RobotSize )
{
    // TODO
    //boost::timer timer_query;
    std::vector<cv::Point> path_res;
    // Step-1: add init and goal node to the Graph
    graph_t queryGraph = G;
    int ValueGrid = DTMap.at<uchar>(qInit.x, qInit.y);
    int Range = RobotSize ;
    int res = AreaDetection( DTMap, qInit, RobotSize );
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
    std::vector<vertex_descriptor> neightborsInitNode = prm_GetNeighbors( qInit, queryGraph, 1.5*Range );    
    property_map<graph_t, Node_Point_t>::type NodeMap = boost::get( Node_Point_t(), queryGraph );
    std::vector<vertex_descriptor>::iterator iter;
    vertex_descriptor vp1 = add_vertex( qInit, queryGraph );
    for( iter = neightborsInitNode.begin(); iter != neightborsInitNode.end(); iter++ )
    {
        cv::Point Pt = get( NodeMap, *iter );
        if( prm_collision_checker( qInit, Pt, DTMap, RobotSize ) )
        {
            //std::cout << "Debug(prm_planner.cpp)::prm_local_planner: Neighbor = " << Pt << std::endl;
            add_edge( vp1, *iter, distance( qInit, Pt ), queryGraph );
        }
    }

    ValueGrid = DTMap.at<uchar>(qGoal.x, qGoal.y);
    Range = RobotSize ;
    res = AreaDetection( DTMap, qGoal, RobotSize );
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
    std::vector<vertex_descriptor> neightborsGoalNode = prm_GetNeighbors( qGoal, queryGraph, 1.5*Range );
    NodeMap = boost::get( Node_Point_t(), queryGraph );
    vertex_descriptor vp2 = add_vertex( qGoal, queryGraph );
    for( iter = neightborsGoalNode.begin(); iter != neightborsGoalNode.end(); iter++ )
    {
        cv::Point Pt = get( NodeMap, *iter );
        if( prm_collision_checker( qGoal, Pt, DTMap, RobotSize ) )
        {
            //std::cout << "Debug(prm_planner.cpp)::prm_local_planner: Neighbor = " << Pt << std::endl;
            add_edge( vp2, *iter, distance( qGoal, Pt ), queryGraph );
        }
    }

    // Step-2: D path planning
	property_map<graph_t, edge_weight_t>::type weightmap = boost::get(edge_weight, queryGraph);
	std::vector<vertex_descriptor> p(num_vertices(queryGraph));
	std::vector<int> d(num_vertices(queryGraph));
	property_map<graph_t, vertex_index_t>::type indexmap = boost::get(vertex_index, queryGraph);

	//boost::timer timer_dijkstra;
	dijkstra_shortest_paths(queryGraph, vp1, predecessor_map(&p[0]).distance_map(&d[0]));
    //double time_dijkstra = timer_dijkstra.elapsed();
    //std::cout << "time_dijkstra = " << time_dijkstra << std::endl;
    
    std::vector<vertex_descriptor> path, path_tmp;
    vertex_descriptor v_iter = vp2;
    while( v_iter != vp1 )
	{
		//cout << v_iter << " <-- ";
		path_tmp.push_back(v_iter);
		v_iter = vertex( p[v_iter], queryGraph );
	}
    while( !path_tmp.empty() )
	{
        cv::Point iterPt = get( NodeMap, path_tmp.back() );
        //std::cout << iterPt << "-->";
		path_res.push_back( iterPt );
		path_tmp.pop_back();
	}
    //double time_query = timer_query.elapsed();
    //std::cout << "time_query = " << time_query << std::endl;
    return path_res;
}
