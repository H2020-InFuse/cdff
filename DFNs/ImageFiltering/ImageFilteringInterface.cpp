/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ImageFilteringInterface.cpp
 * @date 19/02/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the ImageFilteringInterface class
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
#include "ImageFilteringInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfn_ci {

using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ImageFilteringInterface::ImageFilteringInterface()
	{
	
	}

ImageFilteringInterface::~ImageFilteringInterface()
	{

	}

void ImageFilteringInterface::imageInput(FrameConstPtr data) 
	{
    	inImage = data;
	}

FrameConstPtr ImageFilteringInterface::filteredImageOutput() 
	{
    	return outFilteredImage;
	}




}

/** @} */

