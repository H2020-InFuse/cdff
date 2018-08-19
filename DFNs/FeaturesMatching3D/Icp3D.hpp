/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESMATCHING3D_ICP3D_HPP
#define FEATURESMATCHING3D_ICP3D_HPP

#include "FeaturesMatching3DInterface.hpp"

#include <VisualPointFeatureVector3D.hpp>
#include <VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Pose.hpp>
#include <SupportTypes.hpp>
#include <Helpers/ParametersListHelper.hpp>

namespace CDFF
{
namespace DFN
{
namespace FeaturesMatching3D
{
	/**
	 * 3D feature matching using ICP (provided by PCL): detect and find the
	 * pose of a 3D model pointcloud in a 3D scene pointcloud.
	 *
	 * The best geometric transformation that matches the source (model)
	 * keypoints to the sink (scene) keypoints is defined as the transformation
	 * with the largest number of inliers.
	 *
	 * @param maxCorrespondenceDistance
	 *        largest distance allowed between the transformed point from the
	 *        model pointcloud and a point from the scene pointcloud before
	 *        they are no longer considered an acceptable match
	 * @param maximumIterations
	 *        maximum number of iterations that the algorithm can run before
	 *        returning a result
	 * @param transformationEpsilon
	 * @param euclideanFitnessEpsilon
	 */
	class Icp3D : public FeaturesMatching3DInterface
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

			Converters::VisualPointFeatureVector3DToPclPointCloudConverter
				visualPointFeatureVector3DToPclPointCloud;
			PoseWrapper::Pose3DConstPtr Convert(
				Eigen::Matrix4f eigenTransform);

			PoseWrapper::Pose3DConstPtr ComputeTransform(
				Converters::SupportTypes::PointCloudWithFeatures sourceCloud,
				Converters::SupportTypes::PointCloudWithFeatures sinkCloud);

			void ValidateParameters();
			void ValidateInputs(
				Converters::SupportTypes::PointCloudWithFeatures sourceCloud,
				Converters::SupportTypes::PointCloudWithFeatures sinkCloud);
			void ValidateCloud(
				Converters::SupportTypes::PointCloudWithFeatures cloud);
	};
}
}
}

#endif // FEATURESMATCHING3D_ICP3D_HPP

/** @} */
