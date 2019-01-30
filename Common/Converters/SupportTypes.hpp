/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file SupportTypes.hpp
 * @date 19/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 *  This is the class that contains support types and constants for the converters.
 *  
 *
 * @{
 */

#ifndef CONVERTERS_SUPPORT_TYPES_HPP
#define CONVERTERS_SUPPORT_TYPES_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Converters
	{

	namespace SupportTypes
		{
		/* --------------------------------------------------------------------------------
		 *
		 * Support Types for Pcl Point Cloud to Visual Point Feature Vector 3D Converters
		 *
		 * --------------------------------------------------------------------------------
		 */

		const unsigned MAX_HISTOGRAM_SIZE = 352;
		typedef pcl::Histogram<MAX_HISTOGRAM_SIZE> MaxSizeHistogram;

		template <class FeatureType>	
		struct PointCloudWithFeatures
			{
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointCloud;
			typename pcl::PointCloud<FeatureType>::ConstPtr featureCloud;
			unsigned descriptorSize;
			};
		};

	}
#endif

/* SupportTypes.hpp */
/** @} */
