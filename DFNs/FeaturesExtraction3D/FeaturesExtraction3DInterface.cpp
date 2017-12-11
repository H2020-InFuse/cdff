/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesExtraction3DInterface.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the FeaturesExtraction3DInterface class
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
#include "FeaturesExtraction3DInterface.hpp"


namespace dfn_ci {

using namespace CppTypes;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FeaturesExtraction3DInterface::FeaturesExtraction3DInterface()
	{
	}

FeaturesExtraction3DInterface::~FeaturesExtraction3DInterface()
	{
	}

void FeaturesExtraction3DInterface::pointCloudInput(PointCloud::ConstPtr data) 
	{
    	inPointCloud = data;
	}

VisualPointFeatureVector3D::ConstPtr FeaturesExtraction3DInterface::featuresSetOutput() 
	{
    	return outFeaturesSet;
	}




}

/** @} */

