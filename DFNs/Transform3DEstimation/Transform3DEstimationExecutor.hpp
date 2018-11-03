/**
 * @addtogroup DFNs
 * @{
 */

#ifndef TRANSFORM3DESTIMATION_EXECUTOR_HPP
#define TRANSFORM3DESTIMATION_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "Transform3DEstimationInterface.hpp"
#include <Types/CPP/CorrespondenceMaps3DSequence.hpp>
#include <Types/CPP/PosesSequence.hpp>

namespace CDFF
{
namespace DFN
{
/**
* All the methods in this class execute the DFN for the computation of a point cloud for a pair of stereo images. 
* A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputMatches: input matches between 3d features;
* @param outputTransforms: output camera poses of all the cameras (except the first) in the reference frame of the first camera;
* @param success: boolean telling whether the estimation was successful;
* @param error: estimation of the error.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/
    class Transform3DEstimationExecutor
    {
        public:

            Transform3DEstimationExecutor(Transform3DEstimationInterface* dfn);
            ~Transform3DEstimationExecutor();

	    void Execute(CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequenceConstPtr inputMatches, PoseWrapper::Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error);

	    void Execute(CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequenceConstPtr inputMatches, PoseWrapper::Poses3DSequencePtr outputTransforms, bool& success, float& error);

	    void Execute(const CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequence& inputMatches, PoseWrapper::Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error);

	    void Execute(const CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequence& inputMatches, PoseWrapper::Poses3DSequence& outputTransforms, bool& success, float& error);

        private:

            Transform3DEstimationInterface* dfn;
    };
}
}

#endif // TRANSFORM3DESTIMATION_EXECUTOR_HPP

/** @} */
