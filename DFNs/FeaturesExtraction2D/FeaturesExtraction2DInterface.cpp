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

using namespace CppTypes;

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

void FeaturesExtraction2DInterface::imageInput(Frame::ConstPtr data) 
	{
    	inImage = data;
	}

VisualPointFeatureVector2D::ConstPtr FeaturesExtraction2DInterface::featuresSetOutput() 
	{
    	return outFeaturesSet;
	}




}

/** @} */

