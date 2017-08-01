
#include "MyHeader.h"
#include "prm_DT_based.h"

int main( int argc, char const *argv[] )
{
    // PRM Algorithm test program
    //////////////////////////////////////////////////////////////////////////////////////
    if( argc != 2 )
    {
		std::cout << "Debug(Error)::unit_test_myprm.cpp: no map file input!" << std::endl;
		std::cout << "Usage: unit_test_myprm [map_file_path]" << std::endl;
		return -1;
    }
    // Check whether the input file path exist
	std::string srcmap_str = argv[1];
	if( (access( srcmap_str.c_str(), F_OK || R_OK )) == -1 )
	{
		std::cout << "Debug(Error)::unit_test_myprm.cpp: " << srcmap_str << " do not exist or you has no right to read it!" << std::endl;
		return -2;
	}
	// Read the source map data as Mat
	cv::Mat srcmap_Mat = cv::imread( srcmap_str, 0 );
	if( srcmap_Mat.empty() )
	{
		std::cout << "Debug(Error)::unit_test_myprm.cpp: Source Map data is empty!" << std::endl;
		return -3;
	}
	// show the source map image in the Source image window
	const std::string winNameSrc_str = "Source Map Image";
	cv::namedWindow( winNameSrc_str, CV_WINDOW_AUTOSIZE );
	cv::imshow( winNameSrc_str, srcmap_Mat );
    //////////////////////////////////////////////////////////////////////////////////////
    // define the RobotSize, default is 8
    const int RobotSize = 8;
    const int MWD = RobotSize / 2;
    
    /////////////////////////////////////////////////////////////////////////////////////
    // Test-one: distance map calculate time: execute 10 times
    cv::Mat DTMap; // DTMap will be initialized in the function prm_distanceMap()
    
    double time_total_dist = 0.0;
    for( int i = 0; i < 10; i++ )
    {
        boost::timer timer_distanceMap;
        prm_disanceMap( srcmap_Mat, DTMap );
        double time_dist = timer_distanceMap.elapsed();
        time_total_dist += time_dist;
        std::cout << "distance map index " << i << " : " << time_dist << "s." << std::endl;
    }
    
    // map pre-process::prm_disanceMap(Calculate the distance transform map)
    // transfer the source image to binary image is done inner the prm_distanceMap.
    prm_disanceMap( srcmap_Mat, DTMap );
    // Debug--show the distance map image in the image window
	const std::string winNameDTMap_str = "Distance Map Image";
	cv::namedWindow( winNameDTMap_str, CV_WINDOW_AUTOSIZE );
	cv::imshow( winNameDTMap_str, DTMap );


    /////////////////////////////////////////////////////////////////////////////////////
    // Test-two: prm construct time: execute 10 times
    double time_total_prm = 0.0;
    for( int i = 0; i < 10; i++ )
    {
        graph_t G;
        boost::timer prm_timer; // Used for program time statistics
        prm_constructor( DTMap, G, RobotSize );
        double time_prm = prm_timer.elapsed();
        time_total_prm += time_prm;
        std::cout << "prm constructor index " << i << " : " << time_prm << "s." << std::endl;
    }
    
    graph_t G;
    // algorithm core::prm_constructor( construct the PRM Graph )
    prm_constructor( DTMap, G, RobotSize );
    //prm_uss_constructor( DTMap, G, RobotSize );
    //double time_constructor = sys_time.elapsed();
	//std::cout << "INFO(main.cpp)::unit_test_myprm: PRM prm_constructor time usage: " << time_constructor << "s." << std::endl;

    // Test three: query time, execute 50 times
    // Random Generator and Distributor initialization
    // Set the random seed and uniform distribution
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator( seed );
    std::uniform_int_distribution<int> disX( MWD, DTMap.rows-MWD );
    std::uniform_int_distribution<int> disY( MWD, DTMap.cols-MWD );   
    cv::Point qInit, qGoal;
    double time_total_query = 0.0;
    for( int i = 0; i < 50; i++ )
    {
        // Random a seed point
        do{
            qInit = cv::Point( disX(generator), disY(generator) );
        } while( (int)DTMap.at<uchar>( qInit.x, qInit.y ) <= RobotSize );
        do{
            qGoal = cv::Point( disX(generator), disY(generator) );
        } while( (int)DTMap.at<uchar>( qGoal.x, qGoal.y ) <= RobotSize );

        boost::timer timer_query; // Used for program time statistics
        prm_query( G, qInit, qGoal, DTMap, RobotSize );
        double time_query = timer_query.elapsed();
        time_total_query += time_query;
        std::cout << "query index " << i << " : " << time_query << "s." << std::endl;
        //std::cout << "-------------------------------" << std::endl;
    }
    std::cout << "Average Distance Map Calculate time (10 times): " << time_total_dist / 10 << "s." << std::endl;
    std::cout << "Average PRM Construct time (10 times): " << time_total_prm / 10 << "s." << std::endl;
    std::cout << "Average Path Query time (50 times): " << time_total_query/50 << "s." << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////
}