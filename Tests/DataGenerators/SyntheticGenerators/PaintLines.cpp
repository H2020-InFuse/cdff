/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PaintLines.cpp
 * @date 09/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * This is the main program for lines painter.
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
#include "LinesPainter.hpp"
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>

using namespace DataGenerators;

const std::string USAGE =
" \n \
This program requires two inputs: the input image file path, and the output lines file path in xml format. \n \n \
Example usage: ./paint_lines ../tests/Data/Images/DevonIslandRoadLeft.ppm ../tests/Data/Images/DevonIslandRoadLeftLines.xml \n \n";

const std::string INSTRUCTIONS = 
	"press keys A-W-S-D to navigate into the image \n \
	press keys O-P to zoom in or out \n \
	press the left mouse button to mark a keypoint down \n \
	press the right mouse button to unmark a keypoint \n \
	press the middle mouse button to close a line \n \
	press m to save the image to file \n \
	press q to quit \n \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 3, USAGE);
	
	PRINT_TO_LOG("", INSTRUCTIONS);

	LinesPainter painter(argv[1], argv[2]);
	painter.Run();

	return 0;
	}

/** @} */
