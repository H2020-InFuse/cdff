/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESMATCHING2D_EXECUTOR_HPP
#define FEATURESMATCHING2D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <Types/CPP/CorrespondenceMap2D.hpp>

namespace CDFF
{
namespace DFN
{
namespace Executors
{
/**
* All the methods in this class execute the DFN for the computation of matches between feature vectors.
* A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputSourceVector: input vector of keypoints of the source image;
* @param inputSinkVector: input vector of keypoints of the sink image;
* @param outputMatches: output vector of matches between source and sink features.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/

void Execute(FeaturesMatching2DInterface* dfn, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputSourceVector,
	VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputSinkVector, CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr& outputMatches);

void Execute(FeaturesMatching2DInterface* dfn, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputSourceVector,
	VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputSinkVector, CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr outputMatches);

void Execute(FeaturesMatching2DInterface* dfn, const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputSourceVector, const
	VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputSinkVector, CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr& outputMatches);

void Execute(FeaturesMatching2DInterface* dfn, const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputSourceVector, const
	VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputSinkVector, CorrespondenceMap2DWrapper::CorrespondenceMap2D& outputMatches);

}
}
}

#endif // FEATURESMATCHING2D_EXECUTOR_HPP

/** @} */
