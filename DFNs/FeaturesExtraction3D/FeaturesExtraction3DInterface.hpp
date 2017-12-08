/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file FeaturesExtraction3DInterface.hpp
 * @date 01/12/2017
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that extract 3D keypoints from a 3D point clouds.    
 *
 * @{
 */
#ifndef FEATURES_EXTRACTION_3D_INTERFACE_HPP
#define FEATURES_EXTRACTION_3D_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <PointCloud3D.h>
#include <VisualPointFeatureVector3D.hpp>
#include <PointCloud3DToPclPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class FeaturesExtraction3DInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            FeaturesExtraction3DInterface();
            virtual ~FeaturesExtraction3DInterface();
            /**
            * Send value to input port image
            * @param pointCloud, a 3D point cloud taken from a 3D sensor or from 3D reconstruction algorithms
            */
            virtual void pointCloudInput(PointCloud3D* data);

            /**
            * Receive value from output port featuresSet
            * @param featuresSet, This is the set of the points extracted from the point cloud, no descriptor is provided yet
            */
            virtual CppTypes::VisualPointFeatureVector3D::ConstPtr featuresSetOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            PointCloud3D* inPointCloud;
            CppTypes::VisualPointFeatureVector3D::ConstPtr outFeaturesSet;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* FeaturesExtraction3DInterface.hpp */
/** @} */
