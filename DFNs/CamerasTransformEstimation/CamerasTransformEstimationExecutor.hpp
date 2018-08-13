/**
 * @addtogroup DFNs
 * @{
 */

#ifndef CAMERASTRANSFORESTIMATION_EXECUTOR_HPP
#define CAMERASTRANSFORESTIMATION_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "CamerasTransformEstimationInterface.hpp"
#include <VisualPointFeatureVector2D.h>
#include <Matrix.hpp>
#include <CorrespondenceMap2D.hpp>
#include <Pose.hpp>

namespace dfn_ci
{

    class CamerasTransformEstimationExecutor
    {
        public:

            CamerasTransformEstimationExecutor(CamerasTransformEstimationInterface* dfn);
            ~CamerasTransformEstimationExecutor();

	    void Execute(MatrixWrapper::Matrix3dConstPtr inputMatrix, CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputMatches, 
			PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

	    void Execute(MatrixWrapper::Matrix3dConstPtr inputMatrix, CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputMatches, 
			PoseWrapper::Pose3DPtr outputTransform, bool& success);

	    void Execute(const MatrixWrapper::Matrix3d& inputMatrix, const CorrespondenceMap2DWrapper::CorrespondenceMap2D& inputMatches, 
			PoseWrapper::Pose3DConstPtr& outputTransform, bool& success);

	    void Execute(const MatrixWrapper::Matrix3d& inputMatrix, const CorrespondenceMap2DWrapper::CorrespondenceMap2D& inputMatches, 
			PoseWrapper::Pose3D& outputTransform, bool& success);

        private:

            CamerasTransformEstimationInterface* dfn;
    };
}

#endif // CAMERASTRANSFORESTIMATION_EXECUTOR_HPP

/** @} */
