/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatchImages.cpp
 * @date 07/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * This is the main program for images matcher.
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
#include "ImagesMatcher.hpp"
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>

using namespace DataGenerators;

const std::string USAGE =
" \n \
This program requires three inputs: the input source image file path, the input sink image file path, and the output xml file path. \n \n \
Example usage: ./match_images ../tests/Data/Images/DevonIslandRoadLeft.ppm ../tests/Data/Images/DevonIslandRoadRight.ppm ../tests/Data/Images/DevonIslandRoadMatches.xml \n \n";


const std::string INSTRUCTIONS = 
	"press keys A-W-S-D to navigate into the source image, F-T-G-H to navigate into the sink image \n \
	press keys O-P to zoom in or out in the source image, K-L to zoom in or out in the sink image \n \
	press the left mouse button to mark a keypoint down (once in the sink and once in the source image) \n \
	press the right mouse button to unselect a keypoint or delete a previous match \n \
	press m to save the image to file \n \
	press q to quit \n \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 4, USAGE);
	
	PRINT_TO_LOG("", INSTRUCTIONS);

	ImagesMatcher matcher(argv[1], argv[2], argv[3]);
	matcher.Run();

	return 0;
	}

/** @} */
