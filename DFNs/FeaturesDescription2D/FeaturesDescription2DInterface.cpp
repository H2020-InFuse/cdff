/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesDescription2DInterface.cpp
 * @date 21/02/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the FeaturesDescription2DInterface class
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
#include "FeaturesDescription2DInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfn_ci {

using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FeaturesDescription2DInterface::FeaturesDescription2DInterface()
	{
	
	}

FeaturesDescription2DInterface::~FeaturesDescription2DInterface()
	{

	}

void FeaturesDescription2DInterface::imageInput(FrameConstPtr data) 
	{
    	inImage = data;
	}

void FeaturesDescription2DInterface::featuresSetInput(VisualPointFeatureVector2DConstPtr data)
	{
	inFeaturesSet = data;
	}

VisualPointFeatureVector2DConstPtr FeaturesDescription2DInterface::featuresSetWithDescriptorsOutput() 
	{
    	return outFeaturesSetWithDescriptors;
	}




}

/** @} */

