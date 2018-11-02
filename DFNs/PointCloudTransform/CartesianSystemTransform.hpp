/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDTRANSFORM_CARTESIANSYSTEMTRANSFORM_HPP
#define POINTCLOUDTRANSFORM_CARTESIANSYSTEMTRANSFORM_HPP

#include "PointCloudTransformInterface.hpp"

#include <PointCloud.hpp>
#include <Pose.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
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
namespace PointCloudTransform
{
	/**
	 * Applying cartesian transform in order to view the point cloud in the external reference system
	 *
	 * @param placeholder, no meaning just a placeholder so that it is easier to add parameters later
	 */
	class CartesianSystemTransform : public PointCloudTransformInterface
	{
		public:

			CartesianSystemTransform();
			virtual ~CartesianSystemTransform();

			virtual void configure();
			virtual void process();

		private:
			typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;

			struct CartesianSystemTransformOptionsSet
			{
				bool placeholder;
			};

			Helpers::ParametersListHelper parametersHelper;
			CartesianSystemTransformOptionsSet parameters;
			static const CartesianSystemTransformOptionsSet DEFAULT_PARAMETERS;

			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;

			AffineTransform ConvertCloudPoseToInversionTransform(const PoseWrapper::Pose3D& cloudPoseInSystem);
			BaseTypesWrapper::Point3D TransformPoint(const BaseTypesWrapper::Point3D& point, const AffineTransform& affineTransform);
			void ValidateParameters();
	};
}
}
}

#endif // POINTCLOUDTRANSFORM_CARTESIANSYSTEMTRANSFORM_HPP

/** @} */
