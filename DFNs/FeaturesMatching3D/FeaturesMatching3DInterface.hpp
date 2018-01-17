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
#include <PointCloud.hpp>
#include <CorrespondenceMap3D.hpp>
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
            * Send value to input port image
            * @param pointCloud, a 3D point cloud taken from a 3D sensor or from 3D reconstruction algorithms
            */
            virtual void sourceCloudInput(PointCloudWrapper::PointCloudConstPtr data);

            /**
            * Send value to input port image
            * @param pointCloud, a 3D point cloud taken from a 3D sensor or from 3D reconstruction algorithms
            */
            virtual void sinkCloudInput(PointCloudWrapper::PointCloudConstPtr data);

            /**
            * Receive value from output port featuresSet
            * @param featuresSet, This is the set of the points extracted from the point cloud, no descriptor is provided yet
            */
            virtual CorrespondenceMap3DWrapper::CorrespondenceMap3DConstPtr correspondenceMapOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            PointCloudWrapper::PointCloudConstPtr inSourceCloud;
            PointCloudWrapper::PointCloudConstPtr inSinkCloud;
            CorrespondenceMap3DWrapper::CorrespondenceMap3DConstPtr outCorrespondenceMap;

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
