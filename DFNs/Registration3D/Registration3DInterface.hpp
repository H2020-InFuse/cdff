/**
 * @addtogroup DFNs
 * @{
 */

#ifndef REGISTRATION3D_REGISTRATION3DINTERFACE_HPP
#define REGISTRATION3D_REGISTRATION3DINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Pointcloud.h>
#include <Types/C/Pose.h>

namespace CDFF
{
namespace DFN
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
             * @param transformGuess: initial pose estimate of the coordinate
             *        frame of the source pointcloud relative to the coordinate
             *        frame of the sink pointcloud
             */
            virtual void transformGuessInput(const asn1SccPose& data);
            /**
             * Send value to input port "useGuess"
             * @param useGuess: boolean flag indicating whether the initial
             *        estimate of the geometric transformation between the
             *        pointclouds should be used. If it is set to not be used,
             *        and a particular implementation of this DFN requires an
             *        initial estimate, that implementation shall use the
             *        identity transformation as the estimate. If it is set to
             *        be used, and a particular implementation of this DFN does
             *        not require an initial estimate, that implementation must
             *        throw an error.
             */
            virtual void useGuessInput(const bool& data);

            /**
             * Query value from output port "transform"
             * @return transform: pose of the coordinate frame of the source
             *         pointcloud relative to the coordinate frame of the sink
             *         pointcloud
             */
            virtual const asn1SccPose& transformOutput() const;
            /**
             * Query value from output port "success"
             * @return success: boolean flag indicating successful computation
             *         of the geometric transformation between the pointcloud
             *         frames. Computation may fail if the pointclouds are not
             *         similar enough; in that case, the returned geometric
             *         transformation is meaningless.
             */
            virtual bool successOutput() const;

        protected:

            asn1SccPointcloud inSourceCloud;
            asn1SccPointcloud inSinkCloud;
            asn1SccPose inTransformGuess;
            bool inUseGuess = false;
            asn1SccPose outTransform;
            bool outSuccess = false;
    };
}
}

#endif // REGISTRATION3D_REGISTRATION3DINTERFACE_HPP

/** @} */
