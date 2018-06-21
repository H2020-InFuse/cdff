/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatchMultipleImages.cpp
 * @date 19/06/2018
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
#include "MultipleImagesMatcher.hpp"
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>

using namespace DataGenerators;

const std::string USAGE =
" \n \
This program requires n+1 inputs: the input image file paths, and the output xml file path. \n \n \
Example usage: ./match_images ../tests/Data/Images/DevonIslandRoadLeft.ppm ../tests/Data/Images/DevonIslandRoadRight.ppm ../tests/Data/Images/DevonIslandRoadMatches.xml \n \n";


const std::string INSTRUCTIONS = 
	"At any given moment there will be a source image and a sink image \n \
	press Z-X to move between source images, press B-N to move between sink images \n \
	note that you can select a source-sink pair only if the sink comes after source in the input images list \n \
	press keys A-W-S-D to navigate into the source image, F-T-G-H to navigate into the sink image \n \
	press keys O-P to zoom in or out in the source image, K-L to zoom in or out in the sink image \n \
	press the left mouse button to mark a keypoint down (once in the sink and once in the source image) \n \
	press the right mouse button to unselect a keypoint or delete a previous match \n \
	press m to save the image to file \n \
	press q to quit \n \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 4, USAGE);
	
	PRINT_TO_LOG("", INSTRUCTIONS);

	std::vector<std::string> imagePathList(argc-2);
	for(int imageIndex = 0; imageIndex < argc - 2; imageIndex++)
		{
		imagePathList.at(imageIndex) = argv[imageIndex+1];
		}

	MultipleImagesMatcher matcher(imagePathList, argv[argc-1]);
	matcher.Run();

	return 0;
	}

/** @} */
