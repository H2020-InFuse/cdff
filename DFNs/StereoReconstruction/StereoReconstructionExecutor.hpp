/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREORECONSTRUCTION_EXECUTOR_HPP
#define STEREORECONSTRUCTION_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "StereoReconstructionInterface.hpp"
#include <PointCloud.hpp>
#include <Frame.hpp>

namespace CDFF
{
namespace DFN
{
/**
* All the methods in this class execute the DFN for the computation of a point cloud for a pair of stereo images. 
* A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param leftInputFrame: left camera image;
* @param rightInputFrame: right camera image;
* @param outputCloud: output point cloud.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/
    class StereoReconstructionExecutor
    {
        public:

            StereoReconstructionExecutor(StereoReconstructionInterface* dfn);
            ~StereoReconstructionExecutor();

	    void Execute(FrameWrapper::FrameConstPtr leftInputFrame, FrameWrapper::FrameConstPtr rightInputFrame, PointCloudWrapper::PointCloudConstPtr& outputCloud);

	    void Execute(FrameWrapper::FrameConstPtr inputFrame, FrameWrapper::FrameConstPtr rightInputFrame, PointCloudWrapper::PointCloudPtr outputCloud);

	    void Execute(const FrameWrapper::Frame& leftInputFrame, const FrameWrapper::Frame& rightInputFrame, PointCloudWrapper::PointCloudConstPtr& outputCloud);

	    void Execute(const FrameWrapper::Frame& leftInputFrame, const FrameWrapper::Frame& rightInputFrame, PointCloudWrapper::PointCloud& outputCloud);

        private:

            StereoReconstructionInterface* dfn;
    };
}
}

#endif // STEREORECONSTRUCTION_EXECUTOR_HPP

/** @} */
