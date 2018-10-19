/*****************************************************************************

 Copyright (c) 2018 Institute of Robotics and Mechatronics, DLR

 

****************************************************************************//**

 \file opencv_display.h
 \brief your brief file description is missing
 
*****************************************************************************/
#ifndef OPENCV_DISPLAY_H_
#define OPENCV_DISPLAY_H_

///////////////////////////////////////////////////////////////
// DISPLAY
///////////////////////////////////////////////////////////////

#ifdef _USE_OPENCV_DISPLAY

#include <opencv2/core/core.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace DLRtracker
{
void opencv_createWindow(const std::string& title, int resize);

void opencv_showImage(const std::string& title, unsigned char * img, int w, int h, int c);

char opencv_waitKey(int time);

void opencv_destroyWindow(const std::string& title);

void opencv_destroyAllWindows(void);
}

#endif

#endif /* OPENCV_DISPLAY_H_ */
