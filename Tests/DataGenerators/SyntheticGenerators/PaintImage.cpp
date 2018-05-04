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

int main(int argc, char** argv)
	{
	ASSERT(argc >= 3, "This method requires the input image file path, and the output image file path");
	
	ImagePainter painter(argv[1], argv[2]);
	painter.Run();

	return 0;
	}

/** @} */
