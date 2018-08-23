/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESEXTRACTION3D_EXECUTOR_HPP
#define FEATURESEXTRACTION3D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "FeaturesExtraction3DInterface.hpp"
#include <PointCloud.hpp>
#include <VisualPointFeatureVector3D.hpp>

namespace CDFF
{
namespace DFN
{
/**
* All the methods in this class execute the DFN for the computation of 3d features. A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputCloud: input point cloud;
* @param outputVector: output vector of keypoints.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/
    class FeaturesExtraction3DExecutor
    {
        public:

            FeaturesExtraction3DExecutor(FeaturesExtraction3DInterface* dfn);
            ~FeaturesExtraction3DExecutor();

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputVector);
	    void Execute(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr outputVector);
	    void Execute(const PointCloudWrapper::PointCloud& inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputVector);
	    void Execute(const PointCloudWrapper::PointCloud& inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& outputVector);

        private:

            FeaturesExtraction3DInterface* dfn;
    };
}
}

#endif // FEATURESEXTRACTION3D_EXECUTOR_HPP

/** @} */
