/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef REGISTRATION3D_ICP3D_HPP
#define REGISTRATION3D_ICP3D_HPP

#include "Registration3DInterface.hpp"

#include <Types/CPP/Pose.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/EigenTransformToTransform3DConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace CDFF
{
namespace DFN
{
namespace Registration3D
{
	/**
	 * Registration of pointclouds using the ICP algorithm
	 *
	 * @param MaxCorrespondenceDistance
	 * @param MaximumIterations
	 * @param TransformationEpsilon
	 * @param EuclideanFitnessEpsilon
	 */
	class Icp3D : public Registration3DInterface
	{
		public:

			Icp3D();
			virtual ~Icp3D();

			virtual void configure() override;
			virtual void process() override;

		private:

			struct IcpOptionsSet
			{
				double maxCorrespondenceDistance;
				int maximumIterations;
				double transformationEpsilon;
				double euclideanFitnessEpsilon;
			};

			Helpers::ParametersListHelper parametersHelper;
			IcpOptionsSet parameters;
			static const IcpOptionsSet DEFAULT_PARAMETERS;

			Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloud;
			Converters::EigenTransformToTransform3DConverter eigenTransformToTransform3D;

			PoseWrapper::Pose3DConstPtr ComputeTransform(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud);

			void ValidateParameters();
			void ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud);
			void ValidateCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	};
}
}
}

#endif // REGISTRATION3D_ICP3D_HPP

/** @} */
