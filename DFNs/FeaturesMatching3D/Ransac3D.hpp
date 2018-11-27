/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFNs
 * @{
 */

#ifndef FEATURESMATCHING3D_RANSAC3D_HPP
#define FEATURESMATCHING3D_RANSAC3D_HPP

#include "FeaturesMatching3DInterface.hpp"

#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Types/CPP/Pose.hpp>
#include <Converters/SupportTypes.hpp>
#include <Helpers/ParametersListHelper.hpp>

namespace CDFF
{
namespace DFN
{
namespace FeaturesMatching3D
{
	/**
	 * 3D feature matching using RANSAC (provided by PCL): detect and find the
	 * pose of a 3D model pointcloud in a 3D scene pointcloud.
	 *
	 * The best geometric transformation that matches the source (model)
	 * keypoints to the sink (scene) keypoints is defined as the transformation
	 * with the largest number of inliers.
	 *
	 * @param similarityThreshold
	 *        smallest amount of similarity between two keypoints to consider
	 *        them a possible match
	 * @param inlierFraction
	 *        smallest fraction of points from the model pointcloud that must
	 *        be in the scene pointcloud, after geometric transformation of the
	 *        model pointcloud into the coordinate system of the scene
	 *        pointcloud, to consider that transformation suitable
	 * @param correspondenceRandomness
	 * @param numberOfSamples
	 *        number of random samples that are selected in the initial phase
	 *        of RANSAC to make up a model
	 * @param maximumIterations
	 *        maximum number of iterations that the algorithm can run before
	 *        returning a result
	 * @param maxCorrespondenceDistance.searchRadius
	 *        largest distance allowed between the transformed point from the
	 *        model pointcloud and a point from the scene pointcloud before
	 *        they are no longer considered an acceptable match
	 */
	class Ransac3D : public FeaturesMatching3DInterface
	{
		public:

			Ransac3D();
			virtual ~Ransac3D();

			virtual void configure();
			virtual void process();

		private:

			struct RansacOptionsSet
			{
				float similarityThreshold;
				float inlierFraction;
				int correspondenceRandomness;
				int numberOfSamples;
				int maximumIterations;
				float maxCorrespondenceDistance;
			};

			Helpers::ParametersListHelper parametersHelper;
			RansacOptionsSet parameters;
			static const RansacOptionsSet DEFAULT_PARAMETERS;

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

#endif // FEATURESMATCHING3D_RANSAC3D_HPP

/** @} */
