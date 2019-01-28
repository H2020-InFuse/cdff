/**
 * @addtogroup DFNs
 * @{
 */

#ifndef CAMERASTRANSFORMESTIMATION_CAMERASTRANSFORMESTIMATIONINTERFACE_HPP
#define CAMERASTRANSFORMESTIMATION_CAMERASTRANSFORMESTIMATIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Pose.h>
#include <Types/C/CorrespondenceMap2D.h>
#include <Types/C/Eigen.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that estimates the geometric transformation between two cameras
     * based on pairs of 2D matching keypoints found in two images that the
     * cameras captured
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
             * @param matches: keypoint matches between the images A and B.
             *        This is a list of pairs of 2D coordinates: ((x^A_1,
             *        y^A_1), (x^B_1, y^B_1)), ..., ((x^A_n, y^A_n), (x^B_n,
             *        y^B_n)). Each pair contains the coordinates of the same
             *        keypoint (hopefully, the same physical point) in each
             *        image.
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
            bool outSuccess = false;
    };
}
}

#endif // CAMERASTRANSFORMESTIMATION_CAMERASTRANSFORMESTIMATIONINTERFACE_HPP

/** @} */
