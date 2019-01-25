/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDTRANSFORM_POINTCLOUDTRANSFORMINTERFACE_HPP
#define POINTCLOUDTRANSFORM_POINTCLOUDTRANSFORMINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Pointcloud.h>
#include <Types/C/Pose.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that transforms the coordinates of a point cloud from its reference system to an external system E.
     */
    class PointCloudTransformInterface : public DFNCommonInterface
    {
        public:

            PointCloudTransformInterface();
            virtual ~PointCloudTransformInterface();

            /**
             * Send value to input port "pointCloud"
             * @param pointCloud: input point cloud
             */
            virtual void pointCloudInput(const asn1SccPointcloud& data);

            /**
             * Send value to input port "pose"
             * @param pose: pose of the reference system of the point cloud, as viewed from an external system E
             */
            virtual void poseInput(const asn1SccPose& data);

            /**
             * Query value from output port "transformedPointCloud"
             * @return transformedPointCloud: input cloud as viewed from the external system E
             */
            virtual const asn1SccPointcloud& transformedPointCloudOutput() const;

        protected:

            asn1SccPointcloud inPointCloud;
	    asn1SccPose inPose;
            asn1SccPointcloud outTransformedPointCloud;
    };
}
}

#endif // POINTCLOUDTRANSFORM_POINTCLOUDTRANSFORMINTERFACE_HPP

/** @} */
