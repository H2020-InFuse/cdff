/**
 * @addtogroup DFNs
 * @{
 */

#ifndef STEREOMOTIONESTIMATION_STEREOMOTIONESTIMATIONEDRES_HPP
#define STEREOMOTIONESTIMATION_STEREOMOTIONESTIMATIONEDRES_HPP

#include "StereoMotionEstimationInterface.hpp"
#include "edres-wrapper/EdresVO.h"
#include "Helpers/ParametersListHelper.hpp"

namespace CDFF
{
namespace DFN
{
namespace StereoMotionEstimation
{
    /**
     * @brief Implementation of the visual motion estimation algorithms provided by EDRES library
     */
    class StereoMotionEstimationEdres : public StereoMotionEstimationInterface
    {
        public:

            StereoMotionEstimationEdres();
            virtual ~StereoMotionEstimationEdres();

            virtual void configure();
            virtual void process();

            struct StereoMotionEstimationEdresParams
            {
              /**
               * @brief Visual Odometry Method
               * 0 = INCREMENTAL: Execute the incremental optimization
               * 1 = GLOBAL: Execute a global optimization
               */
              int method;

              /**
               * @brief Match process for refinement iteration
               * 0 = MATCH_ALL: Don't use confidence values
               * 1 = MATCH_QUANTIZED: Compute covariance matrix trace of 3D points
               * 2 = MATCH_2DANGLES: Compute the correlation between two points
               * 3 = MATCH_FROMTRUTH_3D: Use the confidence value provided by Harris modules
               * 4 = MATCH_2DANGLES_ORTHO: Product of 3DCOV by CORR
               * 5 = MATCH_FROMTRUTH_2D: Confidence value computed from reconstruction error covariance
               * 6 = MATCH_FASTORTHO: Don't use confidence values
               * 7 = MATCH_BYPACK: Compute covariance matrix trace of 3D points
               * 8 = MATCH_BYCORR: Compute the correlation between two points
               * 9 = MATCH_NOTUSED1: Use the confidence value provided by Harris modules
               * 10 = ZNCC_THRESHOLD: Product of 3DCOV by CORR
               * 11 = ZNCC_N_BEST: Confidence value computed from reconstruction error covariance
               * 12 = DEFORM_GLOBAL: Don't use confidence values
               * 13 = DEFORM_LOCAL: Compute covariance matrix trace of 3D points
               * 14 = MANUAL: Compute the correlation between two points
               * 15 = ZNCC_HRTRACK: Use the confidence value provided by Harris modules
               * 16 = HOMO_HRTRACK: Product of 3DCOV by CORR
               * 17 = ZNCC_HRSTRACK: Confidence value computed from reconstruction error covariance
               * 18 = MATCH_BY_CORRELATION: Don't use confidence values
               * 19 = MATCH_BY_LOCAL_HOMOGRAPHY: Compute covariance matrix trace of 3D points
               * 20 = TRACK_KLD: Compute the correlation between two points
               * 21 = MATCH_SURF: Use the confidence value provided by Harris modules
               * 22 = MATCH_CVSURFFLANN: Product of 3DCOV by CORR
               * 23 = MATCH_CVBRISKFLANN: Confidence value computed from reconstruction error covariance
               * 21 = MATCH_CVBRIEFFLANN: Use the confidence value provided by Harris modules
               * 22 = MATCH_CVORBFLANN: Product of 3DCOV by CORR
               * 23 = MATCH_BRIEFFLANN: Confidence value computed from reconstruction error covariance
               */
              int motionEstimationMatchingAlgo;

              /**
               * @brief Matching algorithm to use
               * 0 = MATCH_ALL: Don't use confidence values
               * 1 = MATCH_QUANTIZED: Compute covariance matrix trace of 3D points
               * 2 = MATCH_2DANGLES: Compute the correlation between two points
               * 3 = MATCH_FROMTRUTH_3D: Use the confidence value provided by Harris modules
               * 4 = MATCH_2DANGLES_ORTHO: Product of 3DCOV by CORR
               * 5 = MATCH_FROMTRUTH_2D: Confidence value computed from reconstruction error covariance
               * 6 = MATCH_FASTORTHO: Don't use confidence values
               * 7 = MATCH_BYPACK: Compute covariance matrix trace of 3D points
               * 8 = MATCH_BYCORR: Compute the correlation between two points
               * 9 = MATCH_NOTUSED1: Use the confidence value provided by Harris modules
               * 10 = ZNCC_THRESHOLD: Product of 3DCOV by CORR
               * 11 = ZNCC_N_BEST: Confidence value computed from reconstruction error covariance
               * 12 = DEFORM_GLOBAL: Don't use confidence values
               * 13 = DEFORM_LOCAL: Compute covariance matrix trace of 3D points
               * 14 = MANUAL: Compute the correlation between two points
               * 15 = ZNCC_HRTRACK: Use the confidence value provided by Harris modules
               * 16 = HOMO_HRTRACK: Product of 3DCOV by CORR
               * 17 = ZNCC_HRSTRACK: Confidence value computed from reconstruction error covariance
               * 18 = MATCH_BY_CORRELATION: Don't use confidence values
               * 19 = MATCH_BY_LOCAL_HOMOGRAPHY: Compute covariance matrix trace of 3D points
               * 20 = TRACK_KLD: Compute the correlation between two points
               * 21 = MATCH_SURF: Use the confidence value provided by Harris modules
               * 22 = MATCH_CVSURFFLANN: Product of 3DCOV by CORR
               * 23 = MATCH_CVBRISKFLANN: Confidence value computed from reconstruction error covariance
               * 21 = MATCH_CVBRIEFFLANN: Use the confidence value provided by Harris modules
               * 22 = MATCH_CVORBFLANN: Product of 3DCOV by CORR
               * 23 = MATCH_BRIEFFLANN: Confidence value computed from reconstruction error covariance
               */
              int featureMatchingAlgo;

              /**
               * @brief Method used for bundle adjustment
               * 0 = NOBA: No bundle adjustment
               * 1 = SBA: Sparse bundle adjustment
               * 2 = GSLBA: Bundle adjustment with gsl library
               */
              int bundleAdjustmentType;

              /**
               * @brief True if the motion estimation process should not take into account the last estimate nor any input information on movement
               */
              bool openLoop;

              /**
               * @brief Maximum theshold to consider the input stereo pair and disparity image synchronized (in microseconds)
               */
              int theshold_us;
            };

            Helpers::ParametersListHelper parametersHelper;
            StereoMotionEstimationEdresParams parameters;
            static const StereoMotionEstimationEdresParams DEFAULT_PARAMETERS;
            void ValidateParameters();

        private:
            Edres::VisualOdometry *_vo;
    };
}
}
}

#endif // STEREOMOTIONESTIMATION_STEREOMOTIONESTIMATIONEDRES_HPP

/** @} */
