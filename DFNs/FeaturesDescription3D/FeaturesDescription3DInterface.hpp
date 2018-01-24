/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file FeaturesDescription3DInterface.hpp
 * @date 24/01/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that computes 3D descriptors for a set of features 
 *  points previously extract from the input point cloud.   
 *
 * @{
 */
#ifndef FEATURES_DESCRIPTION_3D_INTERFACE_HPP
#define FEATURES_DESCRIPTION_3D_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <PointCloud.hpp>
#include <VisualPointFeatureVector3D.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class FeaturesDescription3DInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            FeaturesDescription3DInterface();
            virtual ~FeaturesDescription3DInterface();
            /**
            * Send value to input port image
            * @param pointCloud, a 3D point cloud taken from a 3D sensor or from 3D reconstruction algorithms
            */
            virtual void pointCloudInput(PointCloudWrapper::PointCloudConstPtr data);

            /**
            * Send value to input port image
            * @param featuresSet, this is the list of Visual Point detected by a previous processing step. Their descriptors are empty. The points are defined by a reference to the input point cloud.
            */
            virtual void featuresSetInput(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr data);

            /**
            * Send value to input port image
            * @param normalsCloud, This is the optional cloud of estimated normals to the surface represented by the first point cloud. This input is optional. If this input is empty, normals are
	    * estimated. This input is ignored if ForceNormalEstimation parameter is set.
            */
            virtual void normalsCloudInput(PointCloudWrapper::PointCloudConstPtr data);

            /**
            * Receive value from output port featuresSet
            * @param featuresSet, 
	    * This is the optional cloud of estimated normals to the surface represented by the first point cloud. This input may be optional depending on DFNs, as normals may be estimated in this DFN.
	    * As a suggestion for the implementation of different DFNs, one may use two parameters ForceNormalEstimation and EnableNormalEstimation:
            * If parameter ForceNormalEstimation is set to true, this input will be ignored; otherwise:
	    * If parameter EnableNormalEstimation is set to false, an empty input or invalid input will cause an exception,
	    * If parameter EnableNormalEstimation is set to true, normals will be estimated when this input is empty or not valid.
            */
            virtual VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr featuresSetWithDescriptorsOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            PointCloudWrapper::PointCloudConstPtr inPointCloud;
            VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr inFeaturesSet;
            PointCloudWrapper::PointCloudConstPtr inNormalsCloud;
            VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr outFeaturesSetWithDescriptors;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* FeaturesDescription3DInterface.hpp */
/** @} */
