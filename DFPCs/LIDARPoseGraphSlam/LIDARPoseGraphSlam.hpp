/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef LIDARPOSEGRAPHSLAM_HPP
#define LIDARPOSEGRAPHSLAM_HPP

#include "LIDARPoseGraphSlamInterface.hpp"

namespace CDFF
{
namespace DFPC
{
    /**
     * TODO Class documentation
     */
    class LIDARPoseGraphSlam : public LIDARPoseGraphSlamInterface
    {
        public:

            LIDARPoseGraphSlam();
            virtual ~LIDARPoseGraphSlam();

            virtual void setup() override; 
            virtual void run() override;

    };
}
}

#endif // LIDARPOSEGRAPHSLAM_HPP

/** @} */
