/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef LIDARPOSEGRAPHSLAM_HPP
#define LIDARPOSEGRAPHSLAM_HPP

#include "LIDARPoseGraphSlamInterface.hpp"

namespace dfpc_ci
{
    /**
     * TODO Class documentation
     */
    class LIDARPoseGraphSlam : public LIDARPoseGraphSlamInterface
    {
        public:

            LIDARPoseGraphSlam();
            virtual ~LIDARPoseGraphSlam();

            virtual void setup();
            virtual void run();

    };
}

#endif // LIDARPOSEGRAPHSLAM_HPP

/** @} */
