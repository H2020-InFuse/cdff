/**
 * @addtogroup DFNs
 * @{
 */

#ifndef REGISTRATION3D_INTERFACE_HPP
#define REGISTRATION3D_INTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Pointcloud.h>
#include <Pose.h>

namespace dfn_ci
{
    /**
     * DFN that registers a source point cloud on a sink point cloud
     */
    class Registration3DInterface : public DFNCommonInterface
    {
        public:

            Registration3DInterface();
            virtual ~Registration3DInterface();

            /**
             * Send value to input port "sourceCloud"
             * @param sourceCloud: 3D pointcloud captured by a 3D sensor
             *        or reconstructed from other perceptions
             */
            virtual void sourceCloudInput(const asn1SccPointcloud& data);
            /**
             * Send value to input port "sinkCloud"
             * @param sinkCloud: 3D pointcloud captured by a 3D sensor
             *        or reconstructed from other perceptions
             */
            virtual void sinkCloudInput(const asn1SccPointcloud& data);

            /**
             * Query value from output port "transform"
             * @return transform: pose of the source point cloud relatively to the coordinate 
             *         frame the sink point cloud
             */
            virtual const asn1SccPose& transformOutput() const;
            /**
             * Query value from output port "success"
             * @return success: boolean flag indicating successful computation of the geometric
             *         transformation between the camera frames. Computation may fail if
             *         the point clouds are not similar enough; in this case the
             *         transform is meaningless.
             */
            virtual bool successOutput() const;

        protected:

            asn1SccPointcloud inSourceCloud;
            asn1SccPointcloud inSinkCloud;
            asn1SccPose outTransform;
            bool outSuccess;
    };
}

#endif // REGISTRATION3D_INTERFACE_HPP

/** @} */
