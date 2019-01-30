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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

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

			virtual void configure() override;
			virtual void process() override;

		private:

			//DFN Parameters
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

			//External conversion helpers
			Converters::VisualPointFeatureVector3DToPclPointCloudConverter
				visualPointFeatureVector3DToPclPointCloud;

			//Type conversion methods
			PoseWrapper::Pose3DConstPtr InverseConvert(Eigen::Matrix4f eigenTransformSceneInModel);

			//Input Validation methods
			void ValidateParameters();
			void ValidateInputs();
			void ValidateCloud(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3D& features);


			// Core processing methods:
			void ProcessOnShotDescriptors();
			void ProcessOnPfhDescriptors();

			/**
 			* This function detects if the source pointcloud is within the sink pointcloud,
 			* and computes the geometric transformation that turns the source cloud into
 			* its position in the sink cloud.
 			*
 			* Observe that in PCL terminology, the sink cloud is called target cloud.
 			*
 			* Note: a few more parameters used to be given for SampleConsensusPrerejective:
 			* RANSACOutlierRejectionThreshold, RANSACIterations, TransformationEpsilon,
 			* EuclideanFitnessEpislon, SearchMethodTarget, and SearchMethodSource. However,
 			* besides the search method, those parameters don't seem to be used by the
 			* algorithm. The set method is set in a standard way in the algorithm.
 			*/
			template <class FeatureType>
			PoseWrapper::Pose3DConstPtr ComputeTransform(
				Converters::SupportTypes::PointCloudWithFeatures<FeatureType> sourceCloud,
				Converters::SupportTypes::PointCloudWithFeatures<FeatureType> sinkCloud)
				{
				// Setup PCL's RANSAC algorithm
				typename pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, FeatureType>::Ptr ransac(
					new pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, FeatureType>
					);

				ransac->setSourceFeatures(sourceCloud.featureCloud);
				ransac->setTargetFeatures(sinkCloud.featureCloud);
				ransac->setInputSource(sourceCloud.pointCloud);
				ransac->setInputTarget(sinkCloud.pointCloud);

				ransac->setSimilarityThreshold(parameters.similarityThreshold);
				ransac->setInlierFraction(parameters.inlierFraction);
				ransac->setCorrespondenceRandomness(parameters.correspondenceRandomness);
				ransac->setNumberOfSamples(parameters.numberOfSamples);
				ransac->setMaximumIterations(parameters.maximumIterations);
				ransac->setMaxCorrespondenceDistance(parameters.maxCorrespondenceDistance);

				// Setup output
				pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);

				try 
					{
					// Run RANSAC
					ransac->align(*outputCloud);

					// Check convergence
					outSuccess = ransac->hasConverged();
					}
				catch ( ... )
					{
					VERIFY(false, "Ransac failed with exception!");
					outSuccess = false;
					}

				if (outSuccess)
					{
					Eigen::Matrix4f eigenTransformSceneInModel = ransac->getFinalTransformation();
					return InverseConvert(eigenTransformSceneInModel);
					}
				else
					{
					PoseWrapper::Pose3DPtr transform = new PoseWrapper::Pose3D;
					PoseWrapper::Reset(*transform);
					return transform;
					}
				}

	};
}
}
}

#endif // FEATURESMATCHING3D_RANSAC3D_HPP

/** @} */
