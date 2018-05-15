/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DetectOutliers.cpp
 * @date 14/05/2018
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
#include "OutliersDetector.hpp"
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>

using namespace DataGenerators;

const std::string USAGE =
" \n \
This program requires two inputs: the input cloud file path in ply format, and the output xml file path. \n \n \
Example usage: ./detect_outliers ../tests/Data/PointClouds/Road.ply ../tests/Data/PointClouds/RoadOutliers.xml \n \n";


const std::string INSTRUCTIONS = 
	"press keys Ctrl+ A-W-X-D-E-Z to navigate into the cloud. \n \
	press keys Ctrl+ O-P to increase of decrase the visible area in the cloud \n \
	press SHIFT + left mouse button to mark an outlier \n \
	press Ctrl+N button to unselect a keypoint \n \
	press Ctrl+M to save the outliers to file \n \
	press Ctrl+Q or E to quit \n \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 3, USAGE);
	
	PRINT_TO_LOG("", INSTRUCTIONS);

	OutliersDetector detector(argv[1], argv[2]);
	detector.Run();

	return 0;
	}

/** @} */
