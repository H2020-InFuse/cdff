/**
 * @addtogroup DFNs
 * @{
 */

#ifndef LIDARBASEDTRACKING_LIDARBASEDTRACKINGINTERFACE_HPP
#define LIDARBASEDTRACKING_LIDARBASEDTRACKINGINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/RigidBodyState.h>
#include <Types/C/Pointcloud.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that tracks a model in a point cloud
     */
    class LidarBasedTrackingInterface : public DFNCommonInterface
    {
        public:

            LidarBasedTrackingInterface();
            virtual ~LidarBasedTrackingInterface();

            /**
             * Send value to input port "sourceCloud"
             * @param sourceCloud
             *     point cloud in which we want to track the model
             */
            virtual void sourceCloudInput(const asn1SccPointcloud& data);

            /**
             * Query value from output port "state"
             * @return state
             *     state vector containing the pose and the velocities of the model in the point cloud
             */
            virtual const asn1SccRigidBodyState& stateOutput() const;

        protected:

            asn1SccPointcloud inSourceCloud;
            asn1SccRigidBodyState outState;
    };
}
}

#endif // LIDARBASEDTRACKING_LIDARBASEDTRACKINGINTERFACE_HPP

/** @} */
