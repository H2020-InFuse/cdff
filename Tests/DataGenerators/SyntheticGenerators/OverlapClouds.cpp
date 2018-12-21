/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OverlapClouds.cpp
 * @date 13/11/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * This program is used to manually find the right transform between the scene and model.
 * Keyboard commands allow you to move the model cloud, the current applied transform is in the console output, a video show the position of the model with respect to the scene.
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "CloudsOverlapper.hpp"
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>

using namespace DataGenerators;

const std::string USAGE =
" \n \
This program requires two inputs: the input scene cloud file path in ply format, the input model cloud file path ini ply format \n \n \
Example usage: ./overlap_clouds ../tests/Data/Clouds/road.ply ../tests/Data/Clouds/car.ply \n \n";


const std::string INSTRUCTIONS = 
	"press keys Ctrl+ A-W-X-D-E-Z to move the model cloud, Ctrl + F-T-B-H-Y-V to rotate the model cloud \n \
	press keys Ctrl+ O-P to increase/decrease translation resolution, Ctrl+ K-L to increase/decrease rotation resolution \n \
	press Shift+LeftMouseButton to select a point, press Ctrl+N to cancel previously selected points \n \
	press Ctrl+j once you select 4 pairs of matching points to compute the transform \n \
	Note: make sure that the first point pair (x0, y0) is an exact match, \n \
	for the other three matches (xi, yi) it is sufficient that the lines (x0->xi) and (y0->yi) match. Do not choose 4 coplanar points. \n \
	press Ctrl+Q or E to quit \n \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 3, USAGE);
	
	PRINT_TO_LOG("", INSTRUCTIONS);

	CloudsOverlapper overlapper(argv[1], argv[2]);
	overlapper.Run();

	return 0;
	}

/** @} */
