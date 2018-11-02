/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDFILTERING_STATISTICALOUTLIERREMOVAL_HPP
#define POINTCLOUDFILTERING_STATISTICALOUTLIERREMOVAL_HPP

#include "PointCloudFilteringInterface.hpp"

#include <PointCloud.hpp>
#include <Pose.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/harris_3d.h>
#include <yaml-cpp/yaml.h>

#include <stdlib.h>
#include <string>

namespace CDFF
{
namespace DFN
{
namespace PointCloudFiltering
{
	/**
	 * Applying statistical outlier removal algorithm as implemented in PCL library.
	 *
	 * @param numberOfNearestNeighbours, number of nearest neighbours to use for mean distance estimation;
	 * @param standardDeviationMultiplier, the standard deviation multiplier for distance threshold calculation;
	 * @param takeOutliersOnly, whether the filtering will produce the point cloud of outliers (true) or the filtering will produce the cloud without outliers (false);
	 */
	class StatisticalOutlierRemoval : public PointCloudFilteringInterface
	{
		public:

			StatisticalOutlierRemoval();
			virtual ~StatisticalOutlierRemoval();

			virtual void configure();
			virtual void process();

		private:
			typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;

			struct StatisticalOutlierRemovalOptionsSet
			{
				int numberOfNearestNeighbours;
				double standardDeviationMultiplier;
				bool takeOutliersOnly;
			};

			Helpers::ParametersListHelper parametersHelper;
			StatisticalOutlierRemovalOptionsSet parameters;
			static const StatisticalOutlierRemovalOptionsSet DEFAULT_PARAMETERS;

			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;
			Converters::PclPointCloudToPointCloudConverter pclPointCloudToPointCloud;

			pcl::PointCloud<pcl::PointXYZ>::ConstPtr FilterCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
			void ValidateParameters();
	};
}
}
}

#endif // POINTCLOUDFILTERING_STATISTICALOUTLIERREMOVAL_HPP

/** @} */
