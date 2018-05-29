/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef ICP3D_HPP
#define ICP3D_HPP

#include "Registration3DInterface.hpp"

#include <PointCloud.hpp>
#include <Pose.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <EigenTransformToTransform3DConverter.hpp>
#include <Helpers/ParametersListHelper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/harris_3d.h>
#include <yaml-cpp/yaml.h>

#include <stdlib.h>
#include <string>

namespace dfn_ci
{
	/**
	 * Registration of Point clouds using the ICP algorithm
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

			virtual void configure();
			virtual void process();

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

		Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloudConverter;
		Converters::EigenTransformToTransform3DConverter eigenTransformToTransform3dConverter;

		PoseWrapper::Transform3DConstPtr ComputeTransform(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud);

		void ValidateParameters();
		void ValidateInputs(pcl::PointCloud<pcl::PointXYZ>::ConstPtr sourceCloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr sinkCloud);
		void ValidateCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	};
}

#endif // ICP3D_HPP

/** @} */
