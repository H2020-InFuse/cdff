/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesMatching3DInterface.cpp
 * @date 17/01/2017
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the FeaturesMatching3DInterface class
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
#include "FeaturesMatching3DInterface.hpp"


namespace dfn_ci {

using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FeaturesMatching3DInterface::FeaturesMatching3DInterface()
	{
	}

FeaturesMatching3DInterface::~FeaturesMatching3DInterface()
	{
	}

void FeaturesMatching3DInterface::sourceFeaturesVectorInput(VisualPointFeatureVector3DConstPtr data) 
	{
    	inSourceFeaturesVector = data;
	}

void FeaturesMatching3DInterface::sinkFeaturesVectorInput(VisualPointFeatureVector3DConstPtr data) 
	{
    	inSinkFeaturesVector = data;
	}

Transform3DConstPtr FeaturesMatching3DInterface::transformOutput() 
	{
    	return outTransform;
	}

bool FeaturesMatching3DInterface::successOutput()
	{
	return outSuccess;
	}




}

/** @} */

