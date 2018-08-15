/**
 * @addtogroup DFNs
 * @{
 */

#ifndef PERSPECTIVENPOINTSOLVING_INTERFACE_HPP
#define PERSPECTIVENPOINTSOLVING_INTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Pose.h>
#include <Pointcloud.h>
#include <VisualPointFeatureVector2D.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that solves the Perspective-n-Point problem
     */
    class PerspectiveNPointSolvingInterface : public CDFF::DFN::DFNCommonInterface
    {
        public:

            PerspectiveNPointSolvingInterface();
            virtual ~PerspectiveNPointSolvingInterface();

            /**
             * Send value to input port "points"
             * @param points: a set of n 3D points in a given reference frame
             *        (cartesian coordinates)
             */
            virtual void pointsInput(const asn1SccPointcloud& data);
            /**
             * Send value to input port "projections"
             * @param projections: their corresponding 2D projections in an
             *        image captured by a camera (image coordinates)
             */
            virtual void projectionsInput(const asn1SccVisualPointFeatureVector2D& data);

            /**
             * Query value from output port "camera"
             * @return camera: the pose of the camera in the same reference frame
             */
            virtual const asn1SccPose& cameraOutput() const;
            /**
             * Query value from output port "success"
             * @return success: boolean flag indicating successful estimation
             *        of a valid pose for the camera. If false, the returned
             *        pose is meaningless.
             */
            virtual bool successOutput() const;

        protected:

            asn1SccPointcloud inPoints;
            asn1SccVisualPointFeatureVector2D inProjections;
            asn1SccPose outCamera;
            bool outSuccess;
    };
}
}

#endif // PERSPECTIVENPOINTSOLVING_INTERFACE_HPP

/** @} */
