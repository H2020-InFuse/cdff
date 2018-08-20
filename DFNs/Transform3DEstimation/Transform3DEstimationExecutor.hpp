/**
 * @addtogroup DFNs
 * @{
 */

#ifndef TRANSFORM3DESTIMATION_EXECUTOR_HPP
#define TRANSFORM3DESTIMATION_EXECUTOR_HPP

#include "DFNCommonInterface.hpp"
#include "Transform3DEstimationInterface.hpp"
#include <CorrespondenceMaps3DSequence.hpp>
#include <PosesSequence.hpp>

namespace CDFF
{
namespace DFN
{
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
