/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESMATCHING3D_FEATURESMATCHING3DINTERFACE_HPP
#define FEATURESMATCHING3D_FEATURESMATCHING3DINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <VisualPointFeatureVector3D.h>
#include <Pose.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that matches 3D keypoints
     */
    class FeaturesMatching3DInterface : public DFNCommonInterface
    {
        public:

            FeaturesMatching3DInterface();
            virtual ~FeaturesMatching3DInterface();

            /**
             * Send value to input port "sourceFeatures"
             * @param sourceFeatures: keypoints extracted from the source 3D
             *        pointcloud (a "model" whose keypoints we would like to
             *        find in the other pointcloud)
             */
            virtual void sourceFeaturesInput(const asn1SccVisualPointFeatureVector3D& data);
            /**
             * Send value to input port "sinkFeatures"
             * @param sinkFeatures: keypoints extracted from the sink 3D
             *        pointcloud (a "scene" in which we are looking for the
             *        model)
             */
            virtual void sinkFeaturesInput(const asn1SccVisualPointFeatureVector3D& data);

            /**
             * Query value from output port "transform"
             * @return transform: best geometric transformation that matches
             *        the source keypoints to the sink keypoints
             */
            virtual const asn1SccPose& transformOutput() const;
            /**
             * Query value from output port "success"
             * @return success: boolean flag indicating successful computation
             *        of the geometric transformation between the keypoints.
             *        Computation may fail if the matches are not good enough;
             *        in that case, the returned geometric transformation is
             *        meaningless.
             */
            virtual bool successOutput() const;

        protected:

            asn1SccVisualPointFeatureVector3D inSourceFeatures;
            asn1SccVisualPointFeatureVector3D inSinkFeatures;
            asn1SccPose outTransform;
            bool outSuccess;
    };
}
}

#endif // FEATURESMATCHING3D_FEATURESMATCHING3DINTERFACE_HPP

/** @} */
