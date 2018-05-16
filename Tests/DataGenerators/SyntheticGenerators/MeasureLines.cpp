/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DetectOutliers.cpp
 * @date 15/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * This is the main program for manual outliers detection.
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "ShapesProfiler.hpp"
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>

using namespace DataGenerators;

const std::string USAGE =
" \n \
This program requires two inputs: the input cloud file path in ply format, and the output xml file path. \n \n \
Example usage: ./detect_outliers ../tests/Data/PointClouds/Road.ply ../tests/Data/PointClouds/ShapesProfile.xml \n \n";


const std::string INSTRUCTIONS = 
	"press keys Ctrl+ A-W-X-D-E-Z to navigate into the cloud. \n \
	press keys Ctrl+ O-P to increase of decrase the visible area in the cloud \n \
	press SHIFT + left mouse button to mark a point down \n \
	press Ctrl+N button to unselect a point or a line \n \
	press Ctrl+Y to switch between selection of single points (to input their distance to the camera) and the selection of lines of an object(to input their length) \n \
	press Ctrl+K to move to a previous object, Ctrl+L to move to a successive object or create a new one (available only in lines selection mode) \n \
	press Ctrl+J to input the length of a line or (point - to camera) distance in the standard input \n \
	press Ctrl+M to save the outliers to file \n \
	press Ctrl+Z to switch in and out of inspection mode \n \
	press Ctrl+F to input the coordinates of a camera (for measuring its distance from the selected points in point-to-camera mode)\n \
	press Ctrl+Q or E to quit \n \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 3, USAGE);
	
	PRINT_TO_LOG("", INSTRUCTIONS);

	ShapesProfiler profiler(argv[1], argv[2]);
	profiler.Run();

	return 0;
	}

/** @} */
