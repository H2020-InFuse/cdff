/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef WHEELEDROBOT_TRACKER_HPP
#define WHEELEDROBOT_TRACKER_HPP

#include "ModelBasedTrackerInterface.hpp"
#include <PrimitiveFinder/BasicPrimitiveFinder.hpp>
#include <PoseEstimator/PrimitivesPoseEstimator.hpp>
#include <WeightingExpert/PoseWeighting.hpp>
#include <ImageFiltering/BackgroundSubtractorMOG2.hpp>
#include <PoseEstimator/WheeledRobotPoseEstimator.hpp>

namespace CDFF
{
namespace DFPC
{
    /**
     * WheeledRobotTracker, DFPC implementation that tracks wheeled robots using the 2D image and depth information.
     */
    class WheeledRobotTracker : public ModelBasedTrackerInterface
    {
        public:
            WheeledRobotTracker();
            ~WheeledRobotTracker() = default;

            virtual void setup();
            virtual void run();

        private:

            std::string getRobotConfigurationFileName();

            std::unique_ptr<CDFF::DFN::ImageFilteringInterface> m_background_subtractor;
            std::unique_ptr<CDFF::DFN::PrimitiveFinderInterface> m_circle_finder;
            std::unique_ptr<CDFF::DFN::PoseEstimatorInterface> m_pose_estimator;
            std::string m_robot;
    };
}
}

#endif // WHEELEDROBOT_TRACKER_HPP

/** @} */
