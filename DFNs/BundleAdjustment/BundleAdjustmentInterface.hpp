/**
 * @addtogroup DFNs
 * @{
 */

#ifndef BUNDLEADJUSTMENT_INTERFACE_HPP
#define BUNDLEADJUSTMENT_INTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <CorrespondenceMaps2DSequence.hpp>
#include <PosesSequence.hpp>
#include <PointCloud.hpp>

namespace dfn_ci
{
    /**
     * DFN that determines the poses of a camera from the images taken at those poses.
     */
    class BundleAdjustmentInterface : public DFNCommonInterface
    {
        public:

            BundleAdjustmentInterface();
            virtual ~BundleAdjustmentInterface();

            /**
             * Send value to input port "framesSequence"
             * @param framesSequence: sequence of correspondence maps between pairs of features sets.
             * the correspondence maps sequence is obtained from n features sets
             * extracted from n images taken by a stereo camera pair.
             * The sequence contains n*(n-1) correspondence maps between all the 
             * possible pairs of feature sets.
             * If we order the features set as L1, R1, L2, R2, ..., Lm, Rm, with m = n/2 then
             * the correspondence maps are ordered as (L1-R1), (L1-L2), (L1-R2), ..., 
             * (L1-Lm), (L1-Rm), (R1,L2), (R1,R2), ..., (R1, Lm), (R1,Rm), ..., (Lm, Rm).
             * the maximum number of features set is n = 8, for a maximum number of 28 maps.
             */
            virtual void correspondenceMapsSequenceInput(const asn1SccCorrespondenceMaps2DSequence& data);

            /**
             * Send value to input port "guessedPosesSequence"
             * @param guessedPosesSequence: This is an initial estimation of the camera poses, except the first camera. This input is optional,
             * as some algorithm may not need it. Set to an empty sequence if the input is
             * not provided.
             */
            virtual void guessedPosesSequenceInput(const asn1SccPosesSequence& data);

            /**
             * Send value to input port "guessedPointCloud"
             * @param guessedPointCloud: This is an initial estimation of the 3d position of the features detected by the
             * first camera in the reference frame of the first camera. 3d points are listed in
             * the order in which the features appear in the first correspondence map (L1-R1).
             * This input is optional, as some algorirthms may not need it. Set to empty cloud
             * if the input is not provided.
             */
            virtual void guessedPointCloudInput(const asn1SccPointcloud& data);

            /**
             * Query value from output port "posesSequence"
             * @return posesSequence: poses of the camera in the coordinate system relative
             * to the pose of the camera in the first picture.
             */
            virtual const asn1SccPosesSequence& posesSequenceOutput() const;

            /**
             * Query value from output port "success"
             * @return success: whether the estimation of the poses was successful. 
             * If this variable is false the poses sequence is meaningless.
             */
            virtual bool successOutput() const;

            /**
             * Query value from output port "error"
             * @return error: the squared estimation error.
             */
            virtual float errorOutput() const;

        protected:

            asn1SccCorrespondenceMaps2DSequence inCorrespondenceMapsSequence;
            asn1SccPosesSequence inGuessedPosesSequence;
	    asn1SccPointcloud inGuessedPointCloud;
            asn1SccPosesSequence outPosesSequence;
            bool outSuccess;
	    float outError;
    };
}

#endif // BUNDLEADJUSTMENT_INTERFACE_HPP

/** @} */
