/*==============================================================================
    Copyright (c) 2017 limin(liminkmf@mail.ustc.edu.cn)

==============================================================================*/

#ifndef MyHeader_H_
#define MyHeader_H_
// Include standard C++ components
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <chrono>
#include <random>
#include <cmath>
#include <functional>

// Include Operating System related components
#ifdef WIN32
    #include <windows.h>
    #include <io.h>
#else
    #include <unistd.h>
    #include <fcntl.h>
    #include <pthread.h>
#endif

// Include Boost components
#include <boost/config.hpp>
#include <boost/timer.hpp>
#include <boost/progress.hpp>

// Include OpenCV components
#include <opencv2/opencv.hpp>


#define PI 3.14159265

#endif//MyHeader_H_