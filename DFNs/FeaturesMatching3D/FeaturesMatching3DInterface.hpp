/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file FeaturesMatching3DInterface.hpp
 * @date 17/01/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that finds the best correspondence between two point clouds.    
 *
 * @{
 */
#ifndef FEATURES_MATCHING_3D_INTERFACE_HPP
#define FEATURES_MATCHING_3D_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <VisualPointFeatureVector3D.hpp>
#include <Pose.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class FeaturesMatching3DInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            FeaturesMatching3DInterface();
            virtual ~FeaturesMatching3DInterface();
            /**
            * Send value to input port sourceFeaturesVector
            * @param sourceFeaturesVector, these are the extracted features of the 3D point cloud model we would like to discover in the other point cloud.
            */
            virtual void sourceFeaturesVectorInput(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr data);

            /**
            * Send value to input port sinkFeaturesVector
            * @param sinkFeaturesVector, these are the extracted features of the 3D point cloud into which we are looking for the model.
            */
            virtual void sinkFeaturesVectorInput(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr data);

            /**
            * Receive value from output port transform
            * @param transform, This is the best 3D trasform correspondence that matches source features to sink features.
            */
            virtual PoseWrapper::Transform3DConstPtr transformOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inSourceFeaturesVector;
            VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inSinkFeaturesVector;
            PoseWrapper::Transform3DConstPtr outTransform;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* FeaturesMatching3DInterface.hpp */
/** @} */
