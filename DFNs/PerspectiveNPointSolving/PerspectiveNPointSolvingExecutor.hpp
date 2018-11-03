/**
 * @addtogroup DFNs
 * @{
 */

#ifndef PERSPECTIVENPOINTSOLVING_EXECUTOR_HPP
#define PERSPECTIVENPOINTSOLVING_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "PerspectiveNPointSolvingInterface.hpp"
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <Types/CPP/PointCloud.hpp>

namespace CDFF
{
namespace DFN
{
/**
* All the methods in this class execute the DFN for the computation of a camera pose from 3d points and matching 2d points. 
* A DFN instance has to be passed in the constructor of these class. Each method takes the following parameters:
* @param inputMatches: input matches between 2d features of images taken by two cameras;
* @param inputPose: pose of the second camera in the reference system of the first camera;
* @param outputCloud: output point cloud.
*
* The main difference between the four methods are input and output types:
* Methods (i) and (ii) have the constant pointer as input, Methods (iii)  and (iv) have a constant reference as input;
* Methods (i) and (iii) are non-creation methods, they give constant pointers as output, the output is just the output reference in the DFN;
* When using creation methods, the output has to be initialized to NULL.
* Methods (ii) and (iv) are creation methods, they copy the output of the DFN in the referenced output variable. Method (ii) takes a pointer, method (iv) takes a reference.
*/
    class PerspectiveNPointSolvingExecutor
    {
        public:

            PerspectiveNPointSolvingExecutor(PerspectiveNPointSolvingInterface* dfn);
            ~PerspectiveNPointSolvingExecutor();

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputKeypoints, 
		PoseWrapper::Pose3DConstPtr& outputPose, bool& success);

	    void Execute(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inputKeypoints, 
		PoseWrapper::Pose3DPtr outputPose, bool& success);

	    void Execute(const PointCloudWrapper::PointCloud& inputCloud, const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputKeypoints, 
		PoseWrapper::Pose3DConstPtr& outputPose, bool& success);

	    void Execute(const PointCloudWrapper::PointCloud& inputCloud, const VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2D& inputKeypoints, 
		PoseWrapper::Pose3D& outputPose, bool& success);

        private:

            PerspectiveNPointSolvingInterface* dfn;
    };
}
}
#endif // PERSPECTIVENPOINTSOLVING_EXECUTOR_HPP

/** @} */
