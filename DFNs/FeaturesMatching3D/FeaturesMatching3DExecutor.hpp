/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESMATCHING3D_EXECUTOR_HPP
#define FEATURESMATCHING3D_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "FeaturesMatching3DInterface.hpp"
#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Types/CPP/Pose.hpp>

namespace CDFF
{
namespace DFN
{
/**
* All the methods in this class execute the DFN for the computation of the pose of the sink point cloud in the reference of the source point cloud. 
* A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputSourceVector: input vector of keypoints of the source cloud;
* @param inputSinkVector: input vector of keypoints of the sink cloud;
* @param outputTransform: output estimated pose of the source cloud in the reference frame of the sink cloud.
* @param success: output boolean telling whether the computation was successfull.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/
    class FeaturesMatching3DExecutor
    {
        public:

            FeaturesMatching3DExecutor(FeaturesMatching3DInterface* dfn);
            ~FeaturesMatching3DExecutor();

	    void Execute(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputSourceVector, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputSinkVector, 
			PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

	    void Execute(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputSourceVector, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inputSinkVector, 
			PoseWrapper::Pose3DPtr outputTransform, bool& success);

	    void Execute(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputSourceVector, const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputSinkVector, 
			PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

	    void Execute(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputSourceVector, const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& inputSinkVector, 
			PoseWrapper::Pose3D& outputTransform, bool& success);

        private:

            FeaturesMatching3DInterface* dfn;
    };
}
}

#endif // FEATURESMATCHING3D_EXECUTOR_HPP

/** @} */
