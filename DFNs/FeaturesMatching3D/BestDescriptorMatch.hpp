/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESMATCHING3D_BESTDESCRIPTORMATCH_HPP
#define FEATURESMATCHING3D_BESTDESCRIPTORMATCH_HPP

#include "FeaturesMatching3DInterface.hpp"

#include <VisualPointFeatureVector3D.hpp>
#include <Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Pose.hpp>
#include <Converters/SupportTypes.hpp>
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
	class BestDescriptorMatch : public FeaturesMatching3DInterface
	{
		public:

			BestDescriptorMatch();
			virtual ~BestDescriptorMatch();

			virtual void configure();
			virtual void process();

		private:

			struct BestDescriptorMatchOptionsSet
			{
				double maxCorrespondenceDistance;
				int maximumIterations;
				double transformationEpsilon;
				double euclideanFitnessEpsilon;
			};

			Helpers::ParametersListHelper parametersHelper;
			BestDescriptorMatchOptionsSet parameters;
			static const BestDescriptorMatchOptionsSet DEFAULT_PARAMETERS;

			Converters::VisualPointFeatureVector3DToPclPointCloudConverter
				visualPointFeatureVector3DToPclPointCloud;
			PoseWrapper::Pose3DConstPtr Convert(
				Eigen::Matrix4f eigenTransform);

			void ComputeBestMatches(
				Converters::SupportTypes::PointCloudWithFeatures sourceCloud,
				Converters::SupportTypes::PointCloudWithFeatures sinkCloud,
				pcl::PointCloud<pcl::PointXYZ>::Ptr bestMatchSourceCloud,
				pcl::PointCloud<pcl::PointXYZ>::Ptr bestMatchSinkCloud);
				
			PoseWrapper::Pose3DConstPtr ComputeTransform(
				pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
				pcl::PointCloud<pcl::PointXYZ>::Ptr sinkCloud);

			float ComputeDistance(const Converters::SupportTypes::FeatureType& feature1, const Converters::SupportTypes::FeatureType& feature2, int numberOfFeatureComponents);

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

#endif // FEATURESMATCHING3D_BESTDESCRIPTORMATCH_HPP

/** @} */
