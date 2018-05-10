/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatchClouds.cpp
 * @date 10/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * This is the main program for clouds matcher.
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
#include "CloudsMatcher.hpp"
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>

using namespace DataGenerators;

const std::string USAGE =
" \n \
This program requires three inputs: the input source cloud file path in ply format, the input sink cloud file path ini ply format, and the output xml file path. \n \n \
Example usage: ./match_images ../tests/Data/Images/DevonIslandRoadOne.ply ../tests/Data/Images/DevonIslandRoadTwo.ply ../tests/Data/Images/DevonIslandRoadCloudMatches.xml \n \n";


const std::string INSTRUCTIONS = 
	"press keys Ctrl+ A-W-X-D-E-Z to navigate into the source cloud, Ctrl + F-T-B-H-Y-V to navigate into the sink cloud \n \
	press keys Ctrl+ O-P to increase of decrase the visible area in the source cloud, Ctrl+ K-L to increase or decrease the visible area in the sink cloud \n \
	press SHIFT + left mouse button to mark a keypoint down (once in the sink and once in the source image) \n \
	press Ctrl+N button to unselect a keypoint or delete a previous match \n \
	press Ctrl+M to save the image to file \n \
	press Ctrl+Q or E to quit \n \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 4, USAGE);
	
	PRINT_TO_LOG("", INSTRUCTIONS);

	CloudsMatcher matcher(argv[1], argv[2], argv[3]);
	matcher.Run();

	return 0;
	}

/** @} */
