/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PaintImage.cpp
 * @date 04/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * This is the main program for image painter.
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
#include "ImagePainter.hpp"
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>

using namespace DataGenerators;

const std::string INSTRUCTIONS = 
	"press keys A-W-S-D to navigate into the image \n \
	press keys O-P to zoom in or out \n \
	press the left mouse button to mark a keypoint down \n \
	press the right mouse button to unmark a keypoint \n \
	press m to save the image to file \n \
	press q to quit \n \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 3, "This method requires two inputs: the input image file path, and the output image file path");
	
	PRINT_TO_LOG("", INSTRUCTIONS);

	ImagePainter painter(argv[1], argv[2]);
	painter.Run();

	return 0;
	}

/** @} */
