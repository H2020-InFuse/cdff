/**
 * @addtogroup DFNs
 * @{
 */

#ifndef CAMERASTRANSFORMESTIMATION_INTERFACE_HPP
#define CAMERASTRANSFORMESTIMATION_INTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Pose.h>
#include <CorrespondenceMap2D.h>
#include <Eigen.h>

namespace dfn_ci
{
    /**
     * DFN that estimates the geometric transformation between two cameras
     * based on matching keypoints found in a pair of images that they captured
     */
    class CamerasTransformEstimationInterface : public DFNCommonInterface
    {
        public:

            CamerasTransformEstimationInterface();
            virtual ~CamerasTransformEstimationInterface();

            /**
             * Send value to input port "fundamentalMatrix"
             * @param fundamentalMatrix: fundamental matrix of a camera pair
             *        that captured two images A and B
             */
            virtual void fundamentalMatrixInput(const asn1SccMatrix3d& data);
            /**
             * Send value to input port "matches"
             * @param matches: keypoint matches between images A and B. This
             *        is a list of pairs of 2D coordinates: ((x_A1, y_A1),
             *        (x_B1, y_B1)), ((x_A2, y_A2), (x_B2, y_B2)), ...
             *        The geometric transformation between the two camera
             *        frames is deduced from these points, so the matches
             *        should be as reliable as possible.
             */
            virtual void matchesInput(const asn1SccCorrespondenceMap2D& data);

            /**
             * Query value from output port "transform"
             * @return transform: pose of the coordinate frame of the camera
             *         that captured image B relative to the coordinate frame
             *         of the camera that captured image A
             */
            virtual const asn1SccPose& transformOutput() const;
            /**
             * Query value from output port "success"
             * @return success: boolean flag indicating successful computation
             *         of the geometric transformation between the camera
             *         frames. Computation may fail if the matches are not good
             *         enough; in that case, the returned geometric
             *         transformation is meaningless.
             */
            virtual bool successOutput() const;

        protected:

            asn1SccMatrix3d inFundamentalMatrix;
            asn1SccCorrespondenceMap2D inMatches;
            asn1SccPose outTransform;
            bool outSuccess;
    };
}

#endif // CAMERASTRANSFORMESTIMATION_INTERFACE_HPP

/** @} */
