/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PclPointCloudToVisualPointFeatureVector3DConverter.hpp
 * @date 19/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from PclPointCloud to VisualPointFeatureVector3D.
 *  
 *
 * @{
 */

#ifndef PCL_POINT_CLOUD_TO_VISUAL_POINT_FEATURE_3D_VECTOR_CONVERTER_HPP
#define PCL_POINT_CLOUD_TO_VISUAL_POINT_FEATURE_3D_VECTOR_CONVERTER_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "SupportTypes.hpp"


namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PclPointCloudToVisualPointFeatureVector3DConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		template <unsigned SIZE>
		const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr Convert(const SupportTypes::PointCloudWithFeatures<pcl::Histogram<SIZE> >& featuresCloud)
			{
			VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr conversion = VisualPointFeatureVector3DWrapper::NewVisualPointFeatureVector3D();
	
			for(unsigned pointIndex = 0; pointIndex < featuresCloud.pointCloud->points.size(); pointIndex++)
				{
				pcl::PointXYZ point = featuresCloud.pointCloud->points.at(pointIndex);
				pcl::Histogram<SIZE> feature = featuresCloud.featureCloud->points.at(pointIndex);
				VisualPointFeatureVector3DWrapper::AddPoint(*conversion, point.x, point.y, point.z);
				for(unsigned componentIndex = 0; componentIndex < featuresCloud.descriptorSize; componentIndex++)
					{
					AddDescriptorComponent(*conversion, pointIndex, feature.histogram[componentIndex]);
					}
				}

			return conversion;
			}

		template <class FeatureType>
		const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DSharedConstPtr ConvertShared(const SupportTypes::PointCloudWithFeatures<FeatureType>& featuresCloud)
			{
			VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr conversion = Convert(featuresCloud);
			VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DSharedConstPtr sharedConversion(conversion);
			return sharedConversion;
			}

		const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr Convert(const SupportTypes::PointCloudWithFeatures<pcl::SHOT352 >& featuresCloud);
		const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr Convert(const SupportTypes::PointCloudWithFeatures<pcl::PFHSignature125 >& featuresCloud);

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

/* PclPointCloudToVisualPointFeatureVector3DConverter.hpp */
/** @} */
