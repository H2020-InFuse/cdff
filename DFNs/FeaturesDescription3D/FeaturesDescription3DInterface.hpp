/*
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESDESCRIPTION3D_INTERFACE_HPP
#define FEATURESDESCRIPTION3D_INTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <VisualPointFeatureVector3D.h>
#include <Pointcloud.h>

namespace dfn_ci
{
    /**
     * DFN that computes descriptors for 3D keypoints
     */
    class FeaturesDescription3DInterface : public DFNCommonInterface
    {
        public:

            FeaturesDescription3DInterface();
            virtual ~FeaturesDescription3DInterface();

            /**
             * Send value to input port "pointcloud"
             * @param pointcloud: 3D pointcloud captured by a 3D sensor
             *        or reconstructed from other perceptions
             */
            virtual void pointcloudInput(const asn1SccPointcloud& data);
            /**
             * Send value to input port "features"
             * @param features: keypoints extracted from the pointcloud,
             *        without any descriptors, provided as their index
             *        into the input pointcloud
             */
            virtual void featuresInput(const asn1SccVisualPointFeatureVector3D& data);
            /**
             * Send value to input port "normals"
             * @param normals: normals to the surface described by the input
             *        pointcloud, provided as a pointcloud of the same size.
             *        This input is unnecessary for implementations of this DFN
             *        that compute surface normals, and necessary for
             *        implementations that expect surface normals to be
             *        provided. Other implementations may be able to accommodate
             *        both cases: for instance, the ShotDescriptor3D
             *        implementation of this DFN exposes the following
             *        parameters to let its user control the estimation of
             *        normals:
             *        * if ForceNormalEstimation is true:
             *          surface normal estimates are computed regardless of the
             *          provided normals
             *        * otherwise:
             *          * if EnableNormalEstimation is false:
             *            the provided normals are used, unless they're grossly
             *            wrong (e.g. less normals than points), in which case
             *            an exception is thrown
             *          * if EnableNormalEstimation is true:
             *            the provided normals are used, unless they're grossly
             *            wrong (e.g. less normals than points), in which case
             *            better estimates are computed"
             */
            virtual void normalsInput(const asn1SccPointcloud& data);

            /**
             * Query value from output port "features"
             * @return features: same keypoints, with added descriptors,
             *         returned in coordinates or as their index into the input
             *         pointcloud
             */
            virtual const asn1SccVisualPointFeatureVector3D& featuresOutput() const;

        protected:

            asn1SccPointcloud inPointcloud;
            asn1SccVisualPointFeatureVector3D inFeatures;
            asn1SccPointcloud inNormals;
            asn1SccVisualPointFeatureVector3D outFeatures;
    };
}

#endif // FEATURESDESCRIPTION3D_INTERFACE_HPP

/** @} */
