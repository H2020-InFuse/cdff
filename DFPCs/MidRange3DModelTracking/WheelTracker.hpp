/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef WHEELTRACKER_HPP
#define WHEELTRACKER_HPP

#include "MidRange3DModelTrackingInterface.hpp"
#include <PrimitiveFinder/BasicPrimitiveFinder.hpp>
#include <PoseEstimator/PrimitivesPoseEstimator.hpp>
#include <WeightingExpert/PoseWeighting.hpp>
#include <ImageFiltering/BackgroundSubtractorMOG2.hpp>

namespace CDFF
{
namespace DFPC
{
    /**
     * WheelTracker, DFPC implementation that tracks a wheel using the 2D image and depth information.
     */
    class WheelTracker : public MidRange3DModelTrackingInterface
    {
        public:
            WheelTracker();
            ~WheelTracker() = default;

            virtual void setup();
            virtual void run();

        private:
        std::unique_ptr<CDFF::DFN::ImageFilteringInterface> m_background_subtractor;
        std::unique_ptr<CDFF::DFN::PrimitiveFinderInterface> m_ellipse_finder;
        std::unique_ptr<CDFF::DFN::PrimitiveFinderInterface> m_circle_finder;
        std::unique_ptr<CDFF::DFN::PoseEstimatorInterface> m_circle_pose_estimator;
        std::unique_ptr<CDFF::DFN::PoseEstimatorInterface> m_ellipse_pose_estimator;
        std::unique_ptr<CDFF::DFN::WeightingExpertInterface> m_weighting_expert;
    };
}
}

#endif // WHEELTRACKER_HPP

/** @} */
