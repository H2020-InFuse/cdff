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
             * Send value to input port "transformGuess"
             * @param transformGuess: this is the initial transform estimation of the source point
             *        cloud relatively to the coordinate frame of the sink point cloud.
             */
            virtual void transformGuessInput(const asn1SccPose& data);
            /**
             * Send value to input port "useGuess"
             * @param useGuess: this determines whether the transform guess should be used or not.
             *        If it is not used and the implementation requires an estimation,
             *        the identity transform will be used as an initial guess. If the guess
             *        is used and the implementation does not require an estimation,
             *        an error is thrown.
             */
            virtual void useGuessInput(const bool& data);

            /**
             * Query value from output port "transform"
             * @return transform: pose of the source point cloud relatively to the coordinate 
             *         frame of the sink point cloud
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
            asn1SccPose inTransformGuess;
            bool inUseGuess;
            asn1SccPose outTransform;
            bool outSuccess;
    };
}

#endif // REGISTRATION3D_INTERFACE_HPP

/** @} */
