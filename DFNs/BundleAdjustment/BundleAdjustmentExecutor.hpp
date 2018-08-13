/**
 * @addtogroup DFNs
 * @{
 */

#ifndef BUNDLEADJUSTMENT_EXECUTOR_HPP
#define BUNDLEADJUSTMENT_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "BundleAdjustmentInterface.hpp"
#include <CorrespondenceMaps2DSequence.hpp>
#include <PosesSequence.hpp>
#include <PointCloud.hpp>

namespace dfn_ci
{

    class BundleAdjustmentExecutor
    {
        public:

            BundleAdjustmentExecutor(BundleAdjustmentInterface* dfn);
            ~BundleAdjustmentExecutor();

	    void Execute(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr inputMatches, PoseWrapper::Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error);

	    void Execute(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr inputMatches, PoseWrapper::Poses3DSequencePtr outputTransforms, bool& success, float& error);

	    void Execute(const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& inputMatches, PoseWrapper::Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error);

	    void Execute(const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& inputMatches, PoseWrapper::Poses3DSequence& outputTransforms, bool& success, float& error);

	    void Execute(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr inputMatches, PoseWrapper::Poses3DSequenceConstPtr poseGuess, PointCloudWrapper::PointCloudConstPtr cloudGuess, 
			PoseWrapper::Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error);

	    void Execute(CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr inputMatches, PoseWrapper::Poses3DSequenceConstPtr poseGuess, PointCloudWrapper::PointCloudConstPtr cloudGuess, 
			PoseWrapper::Poses3DSequencePtr outputTransforms, bool& success, float& error);

            void Execute(const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& inputMatches, const PoseWrapper::Poses3DSequence& poseGuess, const PointCloudWrapper::PointCloud& cloudGuess, 
			PoseWrapper::Poses3DSequenceConstPtr& outputTransforms, bool& success, float& error);

	    void Execute(const CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequence& inputMatches, const PoseWrapper::Poses3DSequence& poseGuess, const PointCloudWrapper::PointCloud& cloudGuess, 
			PoseWrapper::Poses3DSequence& outputTransforms, bool& success, float& error);

        private:

            BundleAdjustmentInterface* dfn;
    };
}

#endif // BUNDLEADJUSTMENT_EXECUTOR_HPP

/** @} */
