cmake_minimum_required(VERSION 2.8)

project( DTPRM )
set(Demo_VERSION_MAJOR 1)
set(Demo_VERSION_MINOR 0)

# find Boost dependence
find_package( Boost REQUIRED )
if( NOT Boost_FOUND )
	message( FATAL_ERROR " Boost Not Found! ")
else()
	message( "Boost Version: ${Boost_VERSION}" )
	message( "Boost Libraries: ${Boost_LIBRARIES}" )
	message( "Boost Include Path: ${Boost_INCLUDE_DIRS}" )
	include_directories( ${Boost_INCLUDE_DIRS} )
endif()

# find OpenCV dependence
find_package( OpenCV REQUIRED )
if( NOT OpenCV_FOUND )
	message( FATAL_ERROR "OpenCV not found!" )
else()
	message( "OpenCV Version: ${OpenCV_VERSION}" )
	message( "OpenCV include path: ${OpenCV_INCLUDE_DIRS}" )
	message( "OpenCV libraries: ${OpenCV_LIBS}" )
	message( "OpenCV path: ${OpenCV_INSTALL_PATH}" )
	include_directories( ${OpenCV_INCLUDE_DIRS} )
endif()

#add_definitions(-std=c++11)
set( CMAKE_CXX_FLAGS "-std=c++11 -O2" )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib )

aux_source_directory(. DIR_SRCS)

include_directories( ./includes )

# Add the sources file directory
add_subdirectory(src)

# Target