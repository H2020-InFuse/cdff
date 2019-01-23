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

			virtual void configure() override;
			virtual void process() override;

		private:

			//DFN Parameters
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

			//External conversion helpers
			Converters::VisualPointFeatureVector3DToPclPointCloudConverter
				visualPointFeatureVector3DToPclPointCloud;

			//Type conversion methods
			pcl::PointCloud<pcl::PointXYZ>::Ptr Convert(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& features);
			PoseWrapper::Pose3DConstPtr InverseConvert(Eigen::Matrix4f eigenTransformSceneInModel);

			//Core computation methods
			PoseWrapper::Pose3DConstPtr ComputeTransform(
				pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud,
				pcl::PointCloud<pcl::PointXYZ>::Ptr sinkCloud);

			//Input Validation methods
			void ValidateParameters();
			void ValidateInputs();
			void ValidateCloud(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& features);
	};
}
}
}

#endif // FEATURESMATCHING3D_ICP3D_HPP

/** @} */
