/*****************************************************************************

 Copyright (c) 2018 Institute of Robotics and Mechatronics, DLR

****************************************************************************//**

 \file FileParser.h
 
*****************************************************************************/
#ifndef PARSER3_H_
#define PARSER3_H_

#include "common.h"
namespace DLRtracker
{
class  FileParser
{
	public:
		FileParser(void);
		~FileParser();
		int parseAllFiles(const char * path_specifications, const char * fname_camera, const char * fname_tracker, const char * fname_model, int& setup_global_array_counter, double * setup_global_array);
		
		void createGlobalArrayHeaderFile(const char * path_specifications, const int setup_global_array_counter, const double * setup_global_array);

	private:
		int parseCameraFile(const char * path_specifications, const char * fname_camera, int& setup_global_array_counter, double * setup_global_array);
		int parseTrackerFile(const char * path_specifications, const char * fname_tracker, int& setup_global_array_counter, double * setup_global_array);
		int parseModelFile(const char * path_specifications, const char * fname_model, int& setup_global_array_counter, double * setup_global_array);
		int parsePolyhedron(const char * fname, int& setup_global_array_counter, double * setup_global_array);
		int parsePolyline(const char * fname, int& setup_global_array_counter, double * setup_global_array);
};

}

#endif /* PARSER3_H_ */
