/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESDESCRIPTION2D_EXECUTOR_HPP
#define FEATURESDESCRIPTION2D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "FeaturesDescription2DInterface.hpp"
#include <VisualPointFeatureVector2D.h>
#include <Frame.h>
#include <Types/CPP/Frame.hpp>
#include <Types/CPP/VisualPointFeatureVector2D.hpp>

namespace CDFF
{
namespace DFN
{
/**
* All the methods in this class execute the DFN for thecomputation of feature descriptors. A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputFrame: input image;
* @param inputVector: input vector of keypoints;
* @param outputVector: output vector of keypoints with associated descriptors.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/
    class FeaturesDescription2DExecutor
    {
        public:

            FeaturesDescription2DExecutor(FeaturesDescription2DInterface* dfn);
            ~FeaturesDescription2DExecutor();

	    void Execute(FrameWrapper::FrameConstPtr inputFrame, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputVector, 
			VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr& outputVector);

	    void Execute(FrameWrapper::FrameConstPtr inputFrame, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputVector, 
			VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr outputVector);

	    void Execute(const FrameWrapper::Frame& inputFrame, const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputVector, 
			VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr& outputVector);

	    void Execute(const FrameWrapper::Frame& inputFrame, const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputVector, 
			VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& outputVector);

        private:

            FeaturesDescription2DInterface* dfn;
    };
}
}

#endif // FEATURESDESCRIPTION2D_EXECUTOR_HPP

/** @} */
