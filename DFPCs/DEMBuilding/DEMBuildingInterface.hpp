/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef DEMBUILDING_INTERFACE_HPP
#define DEMBUILDING_INTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <Types/C/Pointcloud.h>
#include <Types/C/TransformWithCovariance.h>

namespace CDFF
{
namespace DFPC
{
    /**
     * Digital elevation map building that replies on LIDAR or stereo Pointcloud and the estimated pose of the rover.
     */
    class DEMBuildingInterface : public DFPCCommonInterface
    {
        public:

            DEMBuildingInterface();
            virtual ~DEMBuildingInterface();

            /**
             * Send value to input port "lPC"
             * @param lPC: Point cloud for rover DEM generation
             */
            virtual void lPCInput(const asn1SccPointcloud& data);
            /**
             * Send value to input port "estimatedPose"
             * @param estimatedPose: Estimated pose from localization
             */
            virtual void estimatedPoseInput(const asn1SccTransformWithCovariance& data);

            /**
             * Query value from output port "updatedMap"
             * @return updatedMap: Updated DEM
             */
            //virtual const asn1SccMap& updatedMapOutput() const;


        protected:

            asn1SccPointcloud inLPC;
            asn1SccTransformWithCovariance inEstimatedPose;
            //asn1SccMap outUpdatedMap;

    };
}
}

#endif //  DEMBUILDING_INTERFACE_HPP

/** @} */
