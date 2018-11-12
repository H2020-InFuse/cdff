/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector3DToPclPointCloudConverter.hpp
 * @date 19/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * This is a mock for the converter from VisualPointFeatureVector3D to pcl point cloud
 * 
 * 
 * @{
 */

#ifndef MOCKS_VISUAL_POINT_FEATURE_VECTOR_3D_TO_PCL_POINT_CLOUD_CONVERTER_HPP
#define MOCKS_VISUAL_POINT_FEATURE_VECTOR_3D_TO_PCL_POINT_CLOUD_CONVERTER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mocks/Mock.hpp"
#include <Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class VisualPointFeatureVector3DToPclPointCloudConverter : public Mock, public Converters::VisualPointFeatureVector3DToPclPointCloudConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		virtual ~VisualPointFeatureVector3DToPclPointCloudConverter();
		const Converters::SupportTypes::PointCloudWithFeatures Convert(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& featureVector);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:

	};


}

#endif

/* VisualPointFeatureVector3DToPclPointCloudConverter.hpp */
/** @} */
