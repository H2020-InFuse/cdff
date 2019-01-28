/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDTRANSFORM_CARTESIANSYSTEMTRANSFORM_HPP
#define POINTCLOUDTRANSFORM_CARTESIANSYSTEMTRANSFORM_HPP

#include "PointCloudTransformationInterface.hpp"

#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Pose.hpp>
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
namespace PointCloudTransformation
{
	/**
	 * Applying cartesian transform in order to view the point cloud in the external reference system
	 *
	 * @param placeholder, no meaning just a placeholder so that it is easier to add parameters later
	 */
	class CartesianSystemTransform : public PointCloudTransformationInterface
	{
		public:

			CartesianSystemTransform();
			virtual ~CartesianSystemTransform();

			virtual void configure() override;
			virtual void process() override;

		private:
			//DFN Parameters
			typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> AffineTransform;

			struct CartesianSystemTransformOptionsSet
			{
				bool placeholder;
			};

			Helpers::ParametersListHelper parametersHelper;
			CartesianSystemTransformOptionsSet parameters;
			static const CartesianSystemTransformOptionsSet DEFAULT_PARAMETERS;

			//Internal variable
			pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;

			//External conversion helpers
			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;

			//Type conversion methods
			AffineTransform ConvertCloudPoseToInversionTransform(const PoseWrapper::Pose3D& cloudPoseInSystem);

			//Core computation methods
			BaseTypesWrapper::Point3D TransformPoint(const BaseTypesWrapper::Point3D& point, const AffineTransform& affineTransform);

			//Input Validation methods
			void ValidateParameters();
	};
}
}
}

#endif // POINTCLOUDTRANSFORM_CARTESIANSYSTEMTRANSFORM_HPP

/** @} */
