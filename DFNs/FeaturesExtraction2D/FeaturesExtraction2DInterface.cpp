/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesExtraction2DInterface.cpp
 * @date 15/11/2017
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the FeaturesExtraction2DInterface class
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
#include "FeaturesExtraction2DInterface.hpp"


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FeaturesExtraction2DInterface::FeaturesExtraction2DInterface()
	{
	}

FeaturesExtraction2DInterface::~FeaturesExtraction2DInterface()
	{
	}

void FeaturesExtraction2DInterface::imageInput(ImageType* data) 
	{
    	inImage = data;
	}

VisualPointFeatureVector2D* FeaturesExtraction2DInterface::featuresSetOutput() 
	{
    	return outFeaturesSet;
	}




}

/** @} */

