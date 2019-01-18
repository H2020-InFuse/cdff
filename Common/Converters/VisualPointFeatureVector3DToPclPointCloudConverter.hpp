/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector3DToPclPointCloudConverter.hpp
 * @date 19/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class for type conversion from VisualPointFeatureVector3D to PclPointCloud.
 *  
 *
 * @{
 */

#ifndef VISUAL_POINT_FEATURE_3D_VECTOR_TO_PCL_POINT_CLOUD_CONVERTER_HPP
#define VISUAL_POINT_FEATURE_3D_VECTOR_TO_PCL_POINT_CLOUD_CONVERTER_HPP


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
#include <boost/make_shared.hpp>


namespace Converters {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class VisualPointFeatureVector3DToPclPointCloudConverter
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		template <class FeatureType>
		const SupportTypes::PointCloudWithFeatures<FeatureType> Convert(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& featuresVector)
			{
			SupportTypes::PointCloudWithFeatures<FeatureType> conversion;
			if (VisualPointFeatureVector3DWrapper::GetNumberOfPoints(*featuresVector) == 0)
				{
				return conversion;
				}
			ASSERT( VisualPointFeatureVector3DWrapper::GetVectorType(*featuresVector) == VisualPointFeatureVector3DWrapper::ALL_POSITIONS_VECTOR, 
				"VisualPointFeatureVector3DToPclPointCloudConverter: non empty input feature vector must have all positions-defined points");

			conversion.pointCloud = ExtractPointCloud(featuresVector);
			ExtractFeaturesCloud(featuresVector, conversion.featureCloud); 

			return conversion;
			}

		template <class FeatureType>
		const SupportTypes::PointCloudWithFeatures<FeatureType> ConvertShared(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DSharedConstPtr& featuresVector)
			{
			return Convert<FeatureType>(featuresVector.get());
			}

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

		pcl::PointCloud<pcl::PointXYZ>::ConstPtr ExtractPointCloud(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& featuresVector);

		void ExtractFeaturesCloud(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& featuresVector, SupportTypes::PointCloudWithFeatures<pcl::SHOT352>& conversion);
		void ExtractFeaturesCloud(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& featuresVector, 
			SupportTypes::PointCloudWithFeatures<pcl::PFHSignature125>& conversion);

		template <unsigned SIZE>
		void ExtractFeaturesCloud(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& featuresVector, 
			SupportTypes::PointCloudWithFeatures<pcl::Histogram<SIZE> >& conversion)
			{
			typename pcl::PointCloud<pcl::Histogram<SIZE> >::Ptr featureCloud = boost::make_shared<pcl::PointCloud<pcl::Histogram<SIZE> > >();

			conversion.featureCloud = featureCloud;	
			conversion.descriptorSize = SIZE;

			for(int pointIndex = 0; pointIndex < VisualPointFeatureVector3DWrapper::GetNumberOfPoints(*featuresVector); pointIndex++)
				{
				ASSERT(conversion.descriptorSize == VisualPointFeatureVector3DWrapper::GetNumberOfDescriptorComponents(*featuresVector, pointIndex), 
					"VisualPointFeatureVector3DToPclPointCloudConverter: histogram descriptor with bad size found");
				pcl::Histogram<SIZE> newFeature;
				for(int componentIndex = 0; componentIndex < conversion.descriptorSize; componentIndex++)
					{
					newFeature.histogram[componentIndex] = VisualPointFeatureVector3DWrapper::GetDescriptorComponent(*featuresVector, pointIndex, componentIndex);
					}
				featureCloud->points.push_back(newFeature);		
				}
			}
	};

}

#endif

/* VisualPointFeatureVector3DToPclPointCloudConverter.hpp */
/** @} */
